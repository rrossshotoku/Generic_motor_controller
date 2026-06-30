# ADR-055: Backend-relevant motor-config gray-out (PC tool)

- Status: Accepted
- Date: 2026-06-30
- Related: ADR-039 (brushed backend), ADR-049/052 (brushed gains/encoder), ADR-004 (per-board config)

## Context

The Motor Config tab shows every tuning param regardless of the selected drive backend. Several are
backend-specific (FOC current gains + electrical alignment vs the brushed armature PI), so showing
them all invites editing values that don't apply.

## Decision

GUI-only: the Motor Config panel **grays out (disables) the value fields not used by the backend
selected in the combo** (`0x2000:6`), live on selection — and after a read-back, which sets the combo,
so it also reflects the motor's actual backend. **Conservative** classification — only clearly
backend-specific entries; everything else stays editable:
- **FOC-only:** `0x2400:1-5` (id/iq PI + voltage limit), `0x2000:5` (pole pairs), `0x2700:3/4` (electrical align).
- **Brushed-only:** `0x2400:6/7` (armature PI), `0x2500:8` (quad counts/rev).
- **Shared (always editable):** R/L, Kt, inertia, position + velocity gains, observer/velocity-filter,
  motion envelope, notch, trajectory, holding current.

The field is disabled; its **label stays enabled with a tooltip** explaining why (a disabled widget
won't show one). Also added the **`0x2500:8 quad_counts_per_rev` row**, which was in the contract
(ADR-052) but missing from the GUI.

## Consequences

- GUI-only, **no contract change, no `MC_IF_PROTOCOL_VERSION` bump**. Lives in the Lightweight_CMC GUI.
- When unsure, an entry is left editable (err toward enabled) — worst case a stray field, not a lockout.
- `quad_counts_per_rev` is brushed-only today; it becomes shared once FOC-on-quad exists — revisit then.
