# Gate oversampling-and-averaging vs. fine-step pipelining — a proof

**Question (supervisor).** With macro time-step `H` (e.g. 20 ns), sample the gate
`N` times (e.g. `N=4`, so `h = H/N = 5 ns`), average the `N` samples, and drive
the coarse-`H` solver with that average. Is this *equivalent* to a pipelined
solver running at the fine step `h` — without inserting pipeline registers into
the recurrence?

**Answer.** Equivalent **to first order in `H`**, and *exactly* equal on the part
that the fine step was wanted for (the gate's duty / charge). The two schemes are
**not** identical: they differ by a residual `R` that we derive in closed form;
`R = O(H²)`, and `R ≡ 0` in every step where the gate does not switch. So the
correct claim to take to the supervisor is *first-order equivalence with a
characterised second-order residual*, not exact equivalence (which is false).

Everything below is for the **Buck in CCM**, whose switch enters the plant
**only through the input** (it gates `Vin` into the LC filter); the state matrix
is switch-independent. This is exactly the structure produced by
`tools/twin_gen.py:buck_model` — `M = M0 + kR·M1` has no switch dependence; the
gate multiplies only the input vector `û`. The DAB case (switch in the state
matrix, via `p²`) is discussed at the end.

---

## 1. Model

Continuous-time switched plant, state `x = [v_C, i_L]ᵀ`, gate `s(t) ∈ {0,1}`:

```
ẋ(t) = A x(t) + b · s(t) · Vin ,        A, b constant   (switch is in the input only)
```

For the Buck (CCM): `A = [[-1/RC, 1/C], [-1/L, 0]]`, `b = [0, 1/L]ᵀ`.

Define the exact one-step operators for a step of length `τ`:

```
Φ(τ) = e^{Aτ}
Γ(τ) = ∫₀^τ e^{Aρ} dρ · b = A⁻¹(Φ(τ) − I) b
```

so that, for a **constant** gate `s` over `τ`, the exact update is
`x⁺ = Φ(τ) x + Γ(τ) · s · Vin`. Using the *exact* operators (matrix exponential)
is deliberate: it removes all integrator-truncation effects, so the comparison
isolates **gate handling only**. (The practical Verlet integrator adds a separate
O(H²) term — see §7 — which is *not* part of this question.)

Over one macro-step `[t, t+H]` the exact (variation-of-constants) solution is

```
x(t+H) = Φ(H) x(t) + Vin ∫₀^H Φ(H−σ) b · s(t+σ) dσ .            (1)
```

The homogeneous term `Φ(H) x(t)` contains **no gate dependence at all**. Every
appearance of the gate is inside the one input integral

```
I[s] := Vin ∫₀^H Φ(H−σ) b · s(t+σ) dσ .                         (2)
```

So the entire question reduces to: *how well does each scheme approximate the
single integral (2)?*

---

## 2. The two schemes as discrete maps

Both schemes consume the **same** `N` fine gate samples `s₀,…,s_{N−1} ∈ {0,1}`
(`sⱼ` = gate on sub-interval `[t+jh, t+(j+1)h)`). Let `s̄ = (1/N)Σⱼ sⱼ`.

**Fine pipeline** (`N` exact steps of length `h`, gate `sⱼ` per sub-step):

```
x_fine(t+H) = Φ(h)^N x(t) + Vin Σⱼ₌₀^{N−1} Φ(h)^{N−1−j} Γ(h) sⱼ .   (3)
```

**Averaged** (one exact step of length `H`, gate = the average `s̄`):

```
x_avg(t+H) = Φ(H) x(t) + Vin Γ(H) s̄ .                              (4)
```

---

## 3. Lemma (semigroup / telescoping identity)

```
Φ(h)^N = Φ(H) ,        and        Σⱼ₌₀^{N−1} Φ(h)^{N−1−j} Γ(h) = Γ(H).   (5)
```

*Proof.* `Φ(h)^N = (e^{Ah})^N = e^{A(Nh)} = e^{AH} = Φ(H)`. For the second,
split the integral defining `Γ(H)` over the `N` sub-intervals and pull out the
exponential:

```
Γ(H) = ∫₀^H e^{Aρ} b dρ = Σⱼ₌₀^{N−1} ∫_{jh}^{(j+1)h} e^{Aρ} b dρ
     = Σⱼ₌₀^{N−1} e^{A·jh} ∫₀^h e^{Aρ'} b dρ' = Σⱼ₌₀^{N−1} Φ(h)^j Γ(h).
```

Re-indexing `m = N−1−j` gives `Σ_m Φ(h)^m Γ(h)`, i.e. the sum in (3). ∎

So the homogeneous parts of (3) and (4) are **identically equal** (no
approximation), and `Γ(H) s̄ = (Σⱼ Φ(h)^{N−1−j} Γ(h)) s̄`.

---

## 4. Theorem (the residual, in closed form)

Subtract (4) from (3). The homogeneous parts cancel exactly. Using (5) on the
averaged input term:

```
R := x_fine(t+H) − x_avg(t+H)
   = Vin Σⱼ₌₀^{N−1} Φ(h)^{N−1−j} Γ(h) (sⱼ − s̄).                    (6)
```

This is **exact** — no truncation yet. Three consequences:

**(a) Identical when the gate does not switch.** If `s₀ = … = s_{N−1}` then each
`sⱼ − s̄ = 0`, so `R = 0` *exactly*. In any macro-step with no switching edge the
averaged and pipelined schemes produce the **same state, bit for bit**.

**(b) The residual is O(H²).** Expand the operators for small `h`:

```
Φ(h)^{N−1−j} = I + (N−1−j) A h + O(h²),     Γ(h) = b h + ½ A b h² + O(h³).
⇒ Φ(h)^{N−1−j} Γ(h) = b h + [(N−1−j) + ½] A b · h² + O(h³).
```

Substitute into (6) and use `Σⱼ (sⱼ − s̄) = 0` (definition of the mean), which
kills the **entire O(h) term**:

```
R = Vin · h² · A b · Σⱼ₌₀^{N−1} (N−1−j)(sⱼ − s̄) + O(h³).            (7)
```

Since `h = H/N`, `R = O(H²)`. The surviving factor `Σⱼ (N−1−j)(sⱼ − s̄)` is the
**first moment of the gate about the window** — i.e. *where inside the step the
edge sits*. That, and only that, is what the average throws away.

**(c) First-order equivalence.** Combining (a)/(b) with (5): the two state
sequences agree in their homogeneous evolution exactly and in their input drive
through O(H); the leading discrepancy is the O(H²) term (7), supported only on
edge steps. Hence the schemes are **equivalent to first order in `H`**. ∎

---

## 5. Why averaging is the *right* thing — moment interpretation

Expand the exact kernel in (2): `Φ(H−σ) = Σ_{m≥0} A^m (H−σ)^m / m!`. Then

```
I[s] = Vin Σ_{m≥0} (A^m b / m!) · μ_m ,     μ_m = ∫₀^H (H−σ)^m s(t+σ) dσ      (8)
```

— the input integral is a weighted sum of the **temporal moments** `μ_m` of the
gate over the window.

- `m = 0`: `μ₀ = ∫₀^H s dσ = H · d`, where `d` = duty (charge delivered).
- The averaged scheme uses `s̄` in place of `s`, giving moments
  `μ_m^avg = s̄ ∫₀^H (H−σ)^m dσ = s̄ · H^{m+1}/(m+1)`.

At `m = 0`: `μ₀^avg = s̄ H = d H = μ₀` (since `s̄ = d`, the two schemes share the
same fine samples). **The zeroth moment is reproduced exactly.** At `m = 1` the
moments disagree unless the gate is uniformly spread, and the mismatch enters at
`O(A·H²)` — exactly the residual (7).

> **Averaging the gate reproduces its zeroth moment (duty / charge) exactly and
> nothing higher. The fine pipeline additionally reproduces the higher moments
> (the edge position). They are identical through the first-order term and diverge
> only at second order.**

This is sub-step **state-space averaging** (Middlebrook–Ćuk) made exact at the
moment level: legitimate precisely because the gate enters *linearly*.

---

## 6. Hardware reading — "critical path `dt/4`" and pipeline registers

The recurrence is `x[k+1] = Φ x[k] + (input)`. The loop-carried dependency — the
thing that blocks pipelining and motivated the look-ahead transform — lives in
the **homogeneous** term `Φ x[k]`. The gate lives **only in the input term**,
which is **feed-forward**: it does not depend on `x`.

- *Pipelining* (register insertion) shortens the critical path **without changing
  the computed function**; but you cannot freely pipeline the recurrence because
  of that loop-carried dependency.
- *Averaging the gate* is **not** pipelining the recurrence. It is a feed-forward
  reconstruction of the input term at `h = H/N` resolution: a small accumulator
  `Σ sⱼ` that sits **outside** the recurrence loop. It changes the computed
  function (a coarse-`H` solver with fine duty) but delivers the **duty fidelity**
  you wanted the fine `dt` for, at a critical path = the `H` datapath + a cheap
  adder tree — no extra loop-carried latency, no look-ahead needed for it.

So "equivalent to a pipelined `dt/4` system" is **true in the duty/PWM-fidelity
metric** (proved: zeroth moment exact, §5) and **false as bit-equality** (residual
(6)). The hardware *benefit* of the fine step on the input path is obtained
without paying the recurrence-pipelining cost.

Note your existing look-ahead Buck is the `N=2` instance of (3): it already
samples `s_k`, `s_{k+1}` and weights them by `M·û`, `û`. Collapsing them to `s̄`
(averaging) incurs exactly `R = Vin (M − I) û (s_k − s_{k+1})/2 = O(dt²)`, the
specialisation of (6)/(7) to your coefficients.

---

## 7. What the fine step buys that averaging does *not*

Two **distinct** O(H²) effects — keep them separate:

1. **Input-moment residual `R` (6)/(7).** Present even with an exact integrator.
   This is the gate-handling difference. Edge-localised, O(H²).
2. **Integrator truncation.** With a *real* integrator (your Verlet), one `H`-step
   and `N` `h`-steps of the homogeneous map are not equal: `M_V(H) ≠ M_V(h)^N`,
   differing at O(H²). The fine pipeline integrates the dynamics more accurately.
   This has **nothing to do with the gate** — it is the ODE-solver order.

Averaging addresses only the gate axis (1). It does **not** improve (2). So
averaging is a faithful stand-in for fine-stepping **iff the reason you wanted a
small `dt` was duty/PWM resolution, not integration accuracy of fast dynamics.**
For this Buck the package header reports `dt·f_LC ≈ 1e-3` — the homogeneous
integration is already vastly over-resolved, so (2) is negligible and (1) (which
averaging fixes) is the only axis that matters.

---

## 8. DAB caveat (switch in the state matrix)

For the DAB the bridge polarity `p2` enters the **state matrix** `M0(p2)`, and via
`p2²`. The affine-gate argument of §1–§5 then applies **only after linearising in
the gate**, with one trap: for single-phase-shift `p2 ∈ {+1,−1}`, so `p2² ≡ 1` and
`M0` is affine in `p2` (constant diagonal, off-diagonals ∝ `p2`). Averaging works
— **but you must average the affine gate-coefficients, not compute
`M0(mean(p2))`** — because `(mean p2)² ≠ mean(p2²)`: a mid-step polarity flip gives
`mean(p2)=0`, which would wrongly null the LC coupling (`1 − αβk²·0 = 1`), whereas
the true average keeps `p2² = 1`. The Buck has no such trap (purely linear gate).

---

## 9. Numerical confirmation (subordinate to the proof)

`oversample_vs_pipeline.py` evaluates (6) directly with the exact operators and
confirms the two **sharp, falsifiable** predictions of the theorem (not "the error
is small"):

| Prediction (this proof)                  | Measured                       |
|------------------------------------------|--------------------------------|
| `R ≡ 0` when no edge in the step (4a)    | `0.00e+00` (machine zero)      |
| `‖R‖ = O(H²)`  →  log-log slope = 2 (4b) | slope = **2.00**               |

(The 7 mV-vs-28 mV duty-error figures are *illustrative* of §5/§6, not the proof.)
