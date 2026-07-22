# ADR-069: Soft max-demand current limit in the current loop (below the OC trip)

- Status: Accepted
- Date: 2026-07-22
- Related: ADR-029 (over-current trip), ADR-039 (brushed backend), ADR-047 (frequency sweep),
  ADR-065 (thermal derate of the operational current limit)

## Context

The only bound on current today is the **over-current trip** (`current_trip_a`, 0x2600:2): the fast
loop compares the measured peak phase current `i_max` against it and, on exceed, **latches a fault
and blocks the drive** (ADR-029). That is a protective backstop, not a working limit — hitting it
stops the axis. The velocity cascade has its own `vel_current_limit_a` (0x2300:4), but that only
clamps velocity/position-derived commands; it does nothing in torque mode, for a direct current
command, or for the tuning sweep overlay.

The operator wants a **soft ceiling on the demanded current** that they can set *below* the OC trip,
so the axis saturates at a chosen current instead of tripping — a normal working limit distinct from
the protective trip.

## Decision

Add a **current-command clamp at the current-loop input** in the fast loop, applied to every command
source. New OD entry `current_demand_limit_a` (0x2400:8, F32 RW PERSIST, **0 = disabled** default).

- **Clamp point:** the fast loop's two backend command expressions — FOC `cmd.iq_a` and brushed
  `i_cmd` — are wrapped with `clamp_i_demand()`. This is downstream of every source (velocity/
  position cascade via `s_iq_cmd_published`, torque mode `s_eff_iq_cmd`, and the frequency-sweep
  overlay), so one clamp bounds them all. `clamp_i_demand(x) = |limit|>0 ? clamp(x, ±limit) : x`.
- **Quantity:** the clamp bounds the **torque-producing current** (iq for FOC, armature for
  brushed). With id ≈ 0 in normal operation, peak phase current ≈ |iq|, so this is directly
  comparable to the OC-trip quantity (`i_max`) — which is what makes "set it under the trip"
  meaningful. `id` (commissioning/field-weakening only) is left unclamped; the OC trip still
  backstops that path.
- **Placement in the arch:** the fast loop never reads the OD directly — `od_apply_gains` (slow
  loop) copies `g_od.current_demand_limit_a` into a `volatile float s_i_demand_max_a`, which the
  fast loop reads (float access is atomic on Cortex-M4). Same pattern as the OC-trip threshold.
- **Chosen 0x2400 (current loop) block**, not 0x2600 (faults): this is a control limit, not a fault
  threshold, and it belongs with the current-loop gains in the GUI ("Current loop gains").
- **No cross-validation** enforced between the demand limit and the trip: the operator sets both.
  If the demand limit is set above the trip it is simply never reached (the trip fires first); if
  below, the current saturates there. The clamp intentionally also bounds the **tuning sweep** —
  "max demanded current" means all demanded current — so a sweep can't be driven past it (set the
  limit above the sweep amplitude when running one).

## Consequences

- The axis can be run against a chosen current ceiling without arming the protective trip; the trip
  remains the untouched hard backstop.
- Applies to both backends and every command mode via a single fast-loop seam.
- Additive non-PDO OD entry → no `MC_IF_PROTOCOL_VERSION` bump; PERSIST blob is additive-safe (TLV
  keyed by index/sub), so existing saved config is preserved.
- The clamp is a hard saturation with no anti-windup coupling to the upstream velocity integrator —
  which already limits torque to `torque_limit_nm`, so sustained saturation is bounded there.
- **Follow-up:** if a future need arises to bound the true current *magnitude* under a non-zero id
  (field weakening), promote the clamp to scale the (id, iq) vector rather than iq alone.
