#!/usr/bin/env python3
"""Decode the buck experiment buffer dumped by dump_results.tcl.

Reads results.bin (a flat little-endian int32 stream pulled from DDR), splits it
into the three experiment sections, writes CSVs, and (if matplotlib is present)
plots the setpoint sweep and the closed-loop run.

Layout — must match vitis/buck_ctrl/src/main.c:
  word 0x0000 HDR_MAGIC 0xBC0FFEE2   0x0002/3 COUNTS_PER_SECOND (lo/hi)
  word 0x0040 PROFILE   : MAGIC, n_iters, read_ps, write_ps, rtrip_ps
  word 0x0400 SWEEP     : MAGIC, n_set, n_samp, samp_us, duty, kR, R*1000, _,
                          then per setpoint { vin_q12, n_samp*(vout,iout) }
  word 0x4000 CONTROL   : MAGIC, n_ticks, tick_us, vref_q12, 4, _,_,_,
                          then per tick { vin_q12, vref_q12, vout, duty }
"""
import sys
import struct

Q12 = 1 << 12

HDR_MAGIC, PROF_MAGIC, SWEEP_MAGIC, CTRL_MAGIC = (
    0xBC0FFEE2, 0x50524F46, 0x53574550, 0x4354524C)
W_HDR, W_PROF, W_SWEEP, W_CTRL = 0x0000, 0x0040, 0x0400, 0x4000


def q12(v):
    return v / Q12


def load(path):
    with open(path, "rb") as f:
        raw = f.read()
    n = len(raw) // 4
    # signed for state read-backs; we re-interpret magics as unsigned where needed
    return list(struct.unpack("<%di" % n, raw[:n * 4]))


def u32(x):
    return x & 0xFFFFFFFF


def main(path="results.bin"):
    w = load(path)

    if u32(w[W_HDR]) != HDR_MAGIC:
        print("WARNING: header magic = 0x%08X (expected 0x%08X) — "
              "buffer may be stale/partial." % (u32(w[W_HDR]), HDR_MAGIC))
    cps = (u32(w[2]) | (u32(w[3]) << 32)) or 100_000_000
    print("tick counter = %d Hz  (%.3f ns/tick)" % (cps, 1e9 / cps))

    # ── Profile ──────────────────────────────────────────────────────────────
    if u32(w[W_PROF]) == PROF_MAGIC:
        n, rps, wps, rtps = w[W_PROF + 1], w[W_PROF + 2], w[W_PROF + 3], w[W_PROF + 4]
        print("\n== AXI timing profile (avg over %d) ==" % n)
        print("  read latency        : %7.1f ns" % (rps / 1000.0))
        print("  posted-write latency: %7.1f ns" % (wps / 1000.0))
        print("  write+read round-tr : %7.1f ns" % (rtps / 1000.0))
    else:
        print("\n(no profile section)")

    # ── Sweep ────────────────────────────────────────────────────────────────
    if u32(w[W_SWEEP]) == SWEEP_MAGIC:
        n_set, n_samp, samp_us, duty = w[W_SWEEP + 1:W_SWEEP + 5]
        print("\n== Setpoint sweep: %d setpoints, %d samples @ %d us, duty=%d/100 =="
              % (n_set, n_samp, samp_us, duty))
        p = W_SWEEP + 8
        sweeps = []
        for s in range(n_set):
            vin = q12(w[p]); p += 1
            vout = [q12(w[p + 2 * k]) for k in range(n_samp)]
            iout = [q12(w[p + 2 * k + 1]) for k in range(n_samp)]
            p += 2 * n_samp
            sweeps.append((vin, vout, iout))
            print("  Vin=%5.2f V -> Vout_final=%6.3f V  Iout_final=%6.3f A"
                  % (vin, vout[-1], iout[-1]))

        with open("sweep.csv", "w") as f:
            f.write("t_us," + ",".join(
                "Vout_Vin%.1f,Iout_Vin%.1f" % (v, v) for v, _, _ in sweeps) + "\n")
            for k in range(n_samp):
                cols = ["%.5f,%.5f" % (vo[k], io[k]) for _, vo, io in sweeps]
                f.write("%d,%s\n" % (k * samp_us, ",".join(cols)))
        print("  -> sweep.csv")

        try:
            import matplotlib.pyplot as plt
            t = [k * samp_us for k in range(n_samp)]
            fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(9, 7))
            for vin, vo, io in sweeps:
                a1.plot(t, vo, label="Vin=%.1f V" % vin)
                a2.plot(t, io, label="Vin=%.1f V" % vin)
            a1.set_ylabel("Vout (V)"); a1.grid(True); a1.legend()
            a1.set_title("Buck setpoint sweep (duty=%d%%)" % duty)
            a2.set_ylabel("Iout (A)"); a2.set_xlabel("time (us)"); a2.grid(True)
            fig.tight_layout(); fig.savefig("sweep.png", dpi=130)
            print("  -> sweep.png")
        except ImportError:
            pass

    # ── Control ──────────────────────────────────────────────────────────────
    if u32(w[W_CTRL]) == CTRL_MAGIC:
        n_ticks, tick_us, vref_q = w[W_CTRL + 1], w[W_CTRL + 2], w[W_CTRL + 3]
        vref = q12(vref_q)
        print("\n== Closed-loop control: %d ticks @ %d us, Vref=%.2f V =="
              % (n_ticks, tick_us, vref))
        p = W_CTRL + 8
        t, vin, vout, duty = [], [], [], []
        for k in range(n_ticks):
            vin.append(q12(w[p])); vout.append(q12(w[p + 2]))
            duty.append(w[p + 3]); t.append(k * tick_us)
            p += 4
        with open("control.csv", "w") as f:
            f.write("t_us,Vin,Vref,Vout,duty\n")
            for k in range(n_ticks):
                f.write("%d,%.4f,%.4f,%.5f,%d\n"
                        % (t[k], vin[k], vref, vout[k], duty[k]))
        print("  -> control.csv")

        try:
            import matplotlib.pyplot as plt
            fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(9, 7))
            a1.plot(t, vin, label="Vin", color="0.6")
            a1.plot(t, vout, label="Vout")
            a1.axhline(vref, ls="--", color="r", label="Vref")
            a1.set_ylabel("V"); a1.grid(True); a1.legend()
            a1.set_title("Closed-loop Vout regulation under Vin steps")
            a2.plot(t, duty, color="g"); a2.set_ylabel("duty (counts)")
            a2.set_xlabel("time (us)"); a2.grid(True)
            fig.tight_layout(); fig.savefig("control.png", dpi=130)
            print("  -> control.png")
        except ImportError:
            pass


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "results.bin")
