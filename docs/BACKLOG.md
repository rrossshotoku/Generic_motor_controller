# Backlog — deferred feature ideas (not yet decided / not scheduled)

Ideas captured for later. These are **not** accepted decisions — when one is taken up, promote it to
an ADR (per the mandatory doc process) and remove/annotate it here. Newest at the bottom.

---

## BL-001: Low-speed stick-slip velocity band (hysteretic minimum speed)

- **Raised**: 2026-07-22
- **Status**: idea — deferred (design discussed, ownership + scope not settled)
- **Relates to**: ADR-066 (low-speed dither), ADR-042 (velocity slew ramp), ADR-071 (position deadband)

### Idea

The current actuator stick-slips below a threshold speed. Mitigate by forbidding a *continuous*
velocity setpoint inside a low-speed band `(0, V_limit)` — the axis may only be **stopped** or
running **≥ V_limit**, and may **transition through** the band but not dwell in it:

- Demand lands in the band from rest → effective demand ramps **up to `V_limit`** (skip the sticky
  zone, run at the lowest clean speed).
- "Running", demand still in the band → **latched at `V_limit`**.
- Demand must **return to ~0** before the effective demand ramps back down **through** the band to
  stop (a return-to-zero latch, so it can't chatter at the band edge). Reversal therefore passes
  through 0.

### Caveats to weigh before building (from the design discussion)

1. **Loses sub-band velocity control** — no running slower than `V_limit`. For a broadcast pan/tilt/
   focus axis, a hard minimum-speed floor is visible in slow on-air creeps. Decide: is slow motion
   on this axis already unusably juddery (floor is a win), or is smooth slow creep a needed
   capability? If needed, **friction feedforward** or tuning the existing **dither (ADR-066)**
   preserve slow moves; this band gives them up. (Dither and this band are opposite strategies —
   dither *enables* slow moves, the band *avoids* them.)
2. **Fights position-mode settling** — a position move's velocity naturally decays through the band
   near target; forcing a minimum speed there causes overshoot/hunting. Must be **scoped to
   velocity/jog mode**, off during position moves (where ADR-071's deadband does the parking).

### Ownership (leaning motor-side)

Recommendation: **motor-owned, velocity-mode-only, off by default** — it's a continuous transform on
the velocity demand driven by *this mechanism's* friction, same class as the motor-owned dither and
slew ramp. It plugs into the existing motor-side velocity-demand pipeline just upstream of the slew
limiter (the up/down ramps mostly fall out of the slew limiter once a small `stopped`/`running` state
machine sets the target to `0` or `V_limit`), and it applies to every command source (joystick,
bench, position — gated). Sketch: OD params by the dither block (e.g. `0x2320:6/7` band limit +
return-to-zero threshold).

**The one thing that flips it to the CMC:** if this is purely joystick *jog feel* rather than a hard
"never dwell at low speed" mechanism constraint. The CMC shaping `velocity_setpoint` naturally scopes
it to manual jogs and keeps feel at the axis layer (consistent with the ADR-072 holding move). Open
question to settle first: **jog-feel preference (→ CMC) or axis-wide mechanism constraint (→ motor)?**

### Next step

Settle (a) the slow-creep capability tradeoff and (b) the jog-feel-vs-mechanism-constraint question,
then write the ADR. Do not implement before those two are decided.
