#!/usr/bin/env python3
"""
twin_gen.py - physical-parameters -> fixed-point SystemVerilog package generator
for the Verlet digital-twin engines (buck and DAB look-ahead solvers).

Goal: a user prototypes a new plant by naming PHYSICAL design parameters
(L, C/Co, dt, switching steps-per-period, ...) and a fixed-point FORMAT; the tool
derives every RTL coefficient, checks it fits, and emits the SV package that the
solver `import`s.  No more hand-editing localparams.

Why Python and not pure-SV elaboration: Verlet coefficients are polynomial in the
physical params (no matrix exponential), so the *arithmetic* is easy either way -
but the guardrails a prototyping tool needs (auto fixed-point fit, overflow
refusal, look-ahead symmetry / decomposition-error checks, accuracy-vs-dt
warnings) live far more naturally here, and a generated package of plain
constants synthesises identically on every tool.

Separation of concerns:
    physics  (L, C, dt, k, R-range)  -> coefficients   (this tool)
    structure(steps/period, widths)  -> RTL params / PWM generator
`steps_per_period` never enters the coefficients; it only sets f_sw = 1/(steps*dt)
for the PWM/switch generator.  It is reported here (and the matching ref-model
config emitted) so the twin and its golden reference can't drift.

Usage:
    twin_gen.py dab  --L 20e-6 --Co 470e-6 --k 1.75 --dt 50e-9 --steps 200 \
                     --state-fmt Q8.24 --coef-fmt Q4.28 --out dab_la_pkg.sv
    twin_gen.py buck --L 2.25e-6 --C 1.125e-6 --dt 10e-9 --steps 100 \
                     --kr-scale 3 --state-fmt Q6.12 --coef-fmt Q2.16 --out verlet_pkg.sv

    # check only (no file), or dump the ref-model config for the golden model:
    twin_gen.py dab ... --check
    twin_gen.py dab ... --emit-ref-config dab_params.json
"""
import argparse
import json
import sys
import numpy as np


# Fixed-point format  (Q m.n  ->  m+n bits signed, m integer bits incl. sign)
class Fmt:
    def __init__(self, int_bits, frac_bits):
        self.int_bits = int_bits          # includes the sign bit
        self.frac = frac_bits
        self.width = int_bits + frac_bits

    @classmethod
    def parse(cls, s):
        s = s.strip().upper().lstrip("Q")
        m, n = s.split(".")
        return cls(int(m), int(n))

    def __str__(self):
        return f"Q{self.int_bits}.{self.frac}"

    @property
    def max_real(self):
        return ((1 << (self.width - 1)) - 1) / float(1 << self.frac)

    @property
    def min_real(self):
        return -(1 << (self.width - 1)) / float(1 << self.frac)

    @property
    def lsb(self):
        return 1.0 / float(1 << self.frac)

    def quantize(self, x, name=""):
        """Round-to-nearest (half away from zero) into a signed integer; raise on overflow."""
        scale = 1 << self.frac
        q = int(np.floor(np.abs(x) * scale + 0.5))
        q = -q if x < 0 else q
        lim = 1 << (self.width - 1)
        if q >= lim or q < -lim:
            raise OverflowError(
                f"coefficient '{name}' = {x:.6e} does not fit {self} ({self.width}-bit signed, "
                f"range [{self.min_real:.4g}, {self.max_real:.4g}]). "
                f"Increase integer bits, or apply a scale factor (see --kr-scale)."
            )
        return q

    def hexlit(self, x, name=""):
        q = self.quantize(x, name)
        if q < 0:
            q += (1 << self.width)
        digits = (self.width + 3) // 4
        return f"{self.width}'sh{q:0{digits}X}"


# Topology models  (each returns: list of (name, value, comment), pkg skeleton,
#                   and a validation/report dict)
def _mcomment(v):
    return f"{v:+.6e}"


#  Buck: 2-step look-ahead Verlet, single switch, kR-decomposed damping 
def buck_model(p):
    L, C, dt, S = p["L"], p["C"], p["dt"], p["kr_scale"]
    hL = dt / (2.0 * L)          # half-step inductor coupling
    kC = dt / C                  # capacitor charge coupling
    a = hL * kC                  # = dt^2/(2LC)

    # Single-step M(kR) = M0 + kR*M1 ;  input vector u(kR=0) = [a, hL(2-a)]
    M0 = np.array([[1.0 - a,            kC      ],
                   [-hL * (2.0 - a),    1.0 - a ]])
    M1 = np.array([[-1.0, 0.0],
                   [ hL,  0.0]])        # dM/dkR
    uhat = np.array([a, hL * (2.0 - a)])

    # 2-step look-ahead:  x_{k+2} = M^2*x_k + (M*u)*s_k*Vin + u*s_{k+1}*Vin
    # M^2 ~ M2_base + kR*D     (kR^2 dropped, like the DAB gamma-decomposition)
    M2 = M0 @ M0
    D = M0 @ M1 + M1 @ M0       # coefficient of kR in M^2
    B1 = M0 @ uhat              # first  sub-step input (registered switch s_k)
    B2 = uhat                   # second sub-step input (current switch s_{k+1})

    # D-rows are scaled by 1/S so the runtime kR*(D*x) product can't overflow the
    # state word at worst-case |x|; the engine drives kR_in = S*dt/(R*C).
    entries = [
        ("C_VA",  M2[0, 0], "M2_base[0,0]"),
        ("C_VB",  M2[0, 1], "M2_base[0,1]"),
        ("C_VC1", B1[0],    "B1[0]  first  sub-step input (s_k, registered)"),
        ("C_VC2", B2[0],    "B2[0]  second sub-step input (s_k+1, current)"),
        ("C_VD",  D[0, 0] / S, f"D[0,0]/S  (S={S})"),
        ("C_VE",  D[0, 1] / S, f"D[0,1]/S"),
        ("C_IA",  M2[1, 0], "M2_base[1,0]"),
        ("C_IB",  M2[1, 1], "M2_base[1,1]"),
        ("C_IC1", B1[1],    "B1[1]  first  sub-step input"),
        ("C_IC2", B2[1],    "B2[1]  second sub-step input"),
        ("C_ID",  D[1, 0] / S, f"D[1,0]/S"),
        ("C_IE",  D[1, 1] / S, f"D[1,1]/S"),
    ]

    sfmt, cfmt = p["state_fmt"], p["coef_fmt"]
    pkg = dict(
        name="verlet_pkg",
        typedefs=[
            (f"logic signed [{sfmt.width - 1}:0]", "q6_12", f"Runtime states (v, i, Vin) {sfmt}"),
            (f"logic signed [{cfmt.width - 1}:0]", "q2_16", f"Static core matrix constants {cfmt}"),
        ],
        coef_type="q2_16",
        extra_params=[("int unsigned", "KR_SCALE", str(S),
                       "kR_in = KR_SCALE * dt/(R*C); valid kR_phys in [0, 2/S)")],
        section_title="Buck 2-step look-ahead Verlet coefficients (single switch).",
    )

    # kR validity range that keeps M2~M2_base+kR*D well-posed (kR_phys < 2/S region).
    f_lc = 1.0 / (2.0 * np.pi * np.sqrt(L * C))
    report = dict(
        derived=dict(hL=hL, kC=kC, a=a),
        f_lc=f_lc,
        notes=[f"kR_phys (=dt/RC) valid range ~ [0, {2.0 / S:.3f}); kR_in scaled by S={S}."],
    )
    return entries, pkg, report


#  DAB: 2-step look-ahead Verlet, dual-bridge, gamma-decomposed runtime R 
def dab_model(p):
    L, Co, k, dt = p["L"], p["Co"], p["k"], p["dt"]
    alpha = dt / (2.0 * L)
    beta = dt / Co

    def M0(p2):
        s = p2 * p2
        return np.array([[1.0 - alpha * beta * k * k * s,                 beta * k * p2],
                         [-alpha * k * p2 * (2.0 - alpha * beta * k * k * s),
                          1.0 - alpha * beta * k * k * s]])

    def M1(p2):                       # dM/dgamma ; rank-1, right column zero
        return np.array([[-1.0, 0.0],
                         [alpha * k * p2, 0.0]])

    def bvec(p2):                     # input coupling (V1*p1 factored out)
        s = p2 * p2
        return np.array([alpha * beta * k * p2,
                         2.0 * alpha - alpha * alpha * beta * k * k * s])

    bases = [("PP", +1, +1), ("PN", +1, -1), ("PZ", +1, 0),
             ("ZP", 0, +1), ("ZZ", 0, 0)]

    entries = []
    # C0 = M0(b)M0(a) ; C1 = M0(b)M1(a)+M1(b)M0(a) ; D0 = M0(b)b(a) ; D1 = M1(b)b(a)
    for tag, a, b in bases:
        C0 = M0(b) @ M0(a)
        for nm, val in (("VV", C0[0, 0]), ("VI", C0[0, 1]), ("IV", C0[1, 0]), ("II", C0[1, 1])):
            entries.append((f"C0_{tag}_{nm}", val, f"C0({tag})"))
    for tag, a, b in bases:
        C1 = M0(b) @ M1(a) + M1(b) @ M0(a)
        for nm, val in (("VV", C1[0, 0]), ("VI", C1[0, 1]), ("IV", C1[1, 0]), ("II", C1[1, 1])):
            entries.append((f"C1_{tag}_{nm}", val, f"C1({tag})"))
    for tag, a, b in bases:
        D0 = M0(b) @ bvec(a)
        entries.append((f"D0_{tag}_V", D0[0], f"D0({tag})"))
        entries.append((f"D0_{tag}_I", D0[1], f"D0({tag})"))
    for tag, a, b in bases:
        D1 = M1(b) @ bvec(a)
        entries.append((f"D1_{tag}_V", D1[0], f"D1({tag})"))
        entries.append((f"D1_{tag}_I", D1[1], f"D1({tag})"))
    bP, bZ = bvec(+1), bvec(0)
    entries += [("E0_P_V", bP[0], "b(P)"), ("E0_P_I", bP[1], "b(P)"),
                ("E0_Z_V", bZ[0], "b(Z)"), ("E0_Z_I", bZ[1], "b(Z)")]

    sfmt, cfmt = p["state_fmt"], p["coef_fmt"]
    pkg = dict(
        name="dab_la_pkg",
        typedefs=[
            (f"logic signed [{sfmt.width - 1}:0]", "q8_24", f"state {sfmt}"),
            (f"logic signed [{cfmt.width - 1}:0]", "q4_28", f"coefficient {cfmt}"),
            ("logic signed [1:0]", "b_pol", "bridge polarity: +1, 0, -1"),
        ],
        coef_type="q4_28",
        extra_params=[(None, "C_ONE", cfmt.hexlit(1.0, "C_ONE"), "1.0"),
                      (None, "C_ZERO", cfmt.hexlit(0.0, "C_ZERO"), "0")],
        section_title="DAB 2-step look-ahead Verlet, gamma-decomposed runtime R.",
    )

    #  Validation: DMD symmetry + gamma-decomposition truncation error 
    D = np.diag([1.0, -1.0])
    E = np.diag([-1.0, 1.0])
    P2 = [+1, 0, -1]
    wM = max(np.max(np.abs(M0(-q) - D @ M0(q) @ D)) for q in P2)
    wB = max(np.max(np.abs(bvec(-q) - E @ bvec(q))) for q in P2)
    gchk = dt / (p.get("r_check", 10.0) * Co)
    x0 = np.array([5.0, 1.0])
    wC = wD = 0.0
    for a in P2:
        for b in P2:
            xi = (M0(a) + gchk * M1(a)) @ x0 + bvec(a) * 48.0
            xt = (M0(b) + gchk * M1(b)) @ xi + bvec(b) * 48.0
            C0 = M0(b) @ M0(a); C1 = M0(b) @ M1(a) + M1(b) @ M0(a)
            D0 = M0(b) @ bvec(a); D1 = M1(b) @ bvec(a)
            xd = (C0 + gchk * C1) @ x0 + (D0 + gchk * D1) * 48.0 + bvec(b) * 48.0
            wC = max(wC, abs(xt[0] - xd[0])); wD = max(wD, abs(xt[1] - xd[1]))
    f_lc = 1.0 / (2.0 * np.pi * np.sqrt(L * Co))
    report = dict(
        derived=dict(alpha=alpha, beta=beta),
        f_lc=f_lc,
        checks={
            "|M0(-p)-D*M0*D|_inf": wM,
            "|b(-p)-E*b|_inf": wB,
            f"decomp |dV2| @R={p.get('r_check', 10.0)}": wC,
            "decomp |di|": wD,
        },
        notes=[f"gamma = dt/(R*Co) is a runtime input; gamma^2 term dropped (~{gchk**2:.1e})."],
    )
    return entries, pkg, report


TOPOLOGIES = {"buck": buck_model, "dab": dab_model}


# Emitter
def emit_package(topo, p, entries, pkg, report):
    cfmt, sfmt = p["coef_fmt"], p["state_fmt"]
    ct = pkg["coef_type"]
    L = []
    L.append(f"// {pkg['name']} - AUTO-GENERATED by tools/twin_gen.py - DO NOT HAND-EDIT")
    L.append(f"// {pkg['section_title']}")
    L.append("//")
    L.append(f"// Regenerate:  python tools/twin_gen.py {topo} \\")
    L.append("//   " + " ".join(_argecho(topo, p)))
    L.append("//")
    L.append("// Physical params:  " + _phys_str(topo, p))
    L.append(f"// Formats:          state {sfmt} ({sfmt.width}b)  coeff {cfmt} ({cfmt.width}b)"
             f"   [LSB: state {sfmt.lsb:.2e}, coeff {cfmt.lsb:.2e}]")
    f_clk = 1.0 / p["dt"]
    f_sw = 1.0 / (p["steps"] * p["dt"]) if p.get("steps") else float("nan")
    L.append(f"// Timing:           f_clk={f_clk/1e6:.3f} MHz (dt={p['dt']*1e9:.3g} ns), "
             f"steps/period={p.get('steps')}  ->  f_sw={f_sw/1e3:.3f} kHz")
    L.append(f"// Plant resonance:  f_LC={report['f_lc']/1e3:.3f} kHz   "
             f"(f_sw/f_LC={f_sw/report['f_lc']:.2f},  dt*f_LC={p['dt']*report['f_lc']:.2e})")
    if "checks" in report:
        L.append("// Look-ahead validation:")
        for kk, vv in report["checks"].items():
            L.append(f"//   {kk:<28s} = {vv:.3e}")
    for n in report.get("notes", []):
        L.append(f"// note: {n}")
    L.append(f"package {pkg['name']};")
    for base, name, comment in pkg["typedefs"]:
        L.append(f"    typedef {base} {name};   // {comment}")
    L.append("")
    for typ, name, val, comment in pkg["extra_params"]:
        decl = f"localparam {ct} {name}" if typ is None else f"localparam {typ} {name}"
        L.append(f"    {decl} = {val};  // {comment}")
    if pkg["extra_params"]:
        L.append("")

    namew = max(len(e[0]) for e in entries) + 1
    for name, val, comment in entries:
        lit = cfmt.hexlit(val, name)
        L.append(f"    localparam {ct} {name:<{namew}s}= {lit}; // {_mcomment(val)}  {comment}")
    L.append(f"endpackage")
    L.append("")
    return "\n".join(L)


def _phys_str(topo, p):
    if topo == "buck":
        return f"L={p['L']*1e6:g}u  C={p['C']*1e6:g}u  dt={p['dt']*1e9:g}n  KR_SCALE={p['kr_scale']}"
    return f"L={p['L']*1e6:g}u  Co={p['Co']*1e6:g}u  k={p['k']}  dt={p['dt']*1e9:g}n"


def _argecho(topo, p):
    out = []
    if topo == "buck":
        out += [f"--L {p['L']:g}", f"--C {p['C']:g}", f"--dt {p['dt']:g}",
                f"--steps {p['steps']}", f"--kr-scale {p['kr_scale']}"]
    else:
        out += [f"--L {p['L']:g}", f"--Co {p['Co']:g}", f"--k {p['k']:g}",
                f"--dt {p['dt']:g}", f"--steps {p['steps']}"]
    out += [f"--state-fmt {p['state_fmt']}", f"--coef-fmt {p['coef_fmt']}"]
    return out


# CLI
def main(argv=None):
    ap = argparse.ArgumentParser(description="Verlet digital-twin SV package generator")
    sub = ap.add_subparsers(dest="topo", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--L", type=float, required=True, help="inductance [H]")
    common.add_argument("--dt", type=float, required=True, help="algorithmic timestep = clock period [s]")
    common.add_argument("--steps", type=int, required=True, help="update steps per switching period")
    common.add_argument("--state-fmt", default="Q8.24", help="state fixed-point format, e.g. Q8.24")
    common.add_argument("--coef-fmt", default="Q4.28", help="coefficient fixed-point format, e.g. Q4.28")
    common.add_argument("--out", help="output .sv package path (default: stdout)")
    common.add_argument("--check", action="store_true", help="validate + report only, do not write")
    common.add_argument("--emit-ref-config", help="also write a JSON config for the golden ref model")

    pb = sub.add_parser("buck", parents=[common])
    pb.add_argument("--C", type=float, required=True, help="capacitance [F]")
    pb.add_argument("--kr-scale", type=int, default=3, help="kR damping scale factor S")

    pd = sub.add_parser("dab", parents=[common])
    pd.add_argument("--Co", type=float, required=True, help="output capacitance [F]")
    pd.add_argument("--k", type=float, required=True, help="transformer turns ratio")
    pd.add_argument("--r-check", type=float, default=10.0, help="R for decomposition-error check [ohm]")

    args = ap.parse_args(argv)

    p = dict(
        L=args.L, dt=args.dt, steps=args.steps,
        state_fmt=Fmt.parse(args.state_fmt), coef_fmt=Fmt.parse(args.coef_fmt),
    )
    if args.topo == "buck":
        p.update(C=args.C, kr_scale=args.kr_scale)
    else:
        p.update(Co=args.Co, k=args.k, r_check=args.r_check)

    model = TOPOLOGIES[args.topo]
    try:
        entries, pkg, report = model(p)
        text = emit_package(args.topo, p, entries, pkg, report)   # also runs every quantize() -> overflow check
    except OverflowError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2

    # Accuracy advisory: Verlet error grows when dt is not << the LC period.
    dt_flc = p["dt"] * report["f_lc"]
    warn = []
    if dt_flc > 0.05:
        warn.append(f"dt*f_LC = {dt_flc:.3f} (>0.05): timestep is coarse vs the LC resonance; "
                    f"expect noticeable Verlet phase error. Consider a smaller dt.")
    if args.topo == "dab" and report["checks"].get("decomp |dV2| @R=10.0", 0) > p["state_fmt"].lsb:
        warn.append("gamma-decomposition error exceeds one state LSB at the check R.")

    sys.stderr.write(f"[twin_gen] {args.topo}: {len(entries)} coeffs, "
                     f"state {p['state_fmt']}, coeff {p['coef_fmt']}; "
                     f"f_LC={report['f_lc']/1e3:.2f} kHz, dt*f_LC={dt_flc:.3f}\n")
    if "checks" in report:
        for kk, vv in report["checks"].items():
            sys.stderr.write(f"[twin_gen]   check {kk} = {vv:.3e}\n")
    for w in warn:
        sys.stderr.write(f"[twin_gen] WARNING: {w}\n")

    if args.emit_ref_config:
        cfg = {k: (str(v) if isinstance(v, Fmt) else v) for k, v in p.items()}
        cfg["topology"] = args.topo
        with open(args.emit_ref_config, "w") as f:
            json.dump(cfg, f, indent=2)
        sys.stderr.write(f"[twin_gen] wrote ref config {args.emit_ref_config}\n")

    if args.check:
        sys.stderr.write("[twin_gen] --check: package validated, not written.\n")
        return 0

    if args.out:
        with open(args.out, "w") as f:
            f.write(text)
        sys.stderr.write(f"[twin_gen] wrote {args.out}\n")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
