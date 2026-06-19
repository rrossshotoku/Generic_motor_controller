# ADR-003: Velocity feedback — position-tracking observer as default

## Status

Accepted

## Date

2026-06-19

## Context

Spec 11 specifies velocity by finite-difference of position with low-pass filtering. The
proven controller instead uses an Ellis-style velocity observer
(`FOC_VelocityObserver_Update`, *Observers in Control Systems*, Fig 18-11): a PI
compensator on encoder **position error** plus velocity damping, double-integrated to
produce a smooth, low-lag velocity. It is **not** a back-EMF observer — it is sensored, has
no electrical model, and the optional `Kt/J·iq` feedforward term is disabled in the build
(friction/stiction corrupts the model term). As built it depends only on encoder position,
so it is **motor-parameter-independent**.

## Decision

The state estimator's **default** velocity output is the position-tracking observer.
Finite-difference + low-pass (per spec 11) is provided as a **runtime-selectable fallback**,
mirroring the old `use_observer_velocity` switch.

## Reasoning

The observer gives lower phase lag and less noise than raw differentiation, and because it
needs no motor parameters it ports cleanly to any axis in a generic framework. Keeping the
finite-difference path preserves a simple, dependency-free fallback and an A/B comparison on
hardware.

## Consequences

- `MC_StateEstimator` config gains an estimator-source selector and observer gains
  (kp, ki, kv) plus output/derivative filter coefficients.
- Spec 11 must be updated: observer is the default, finite-difference is the fallback (it
  currently describes only finite-difference).
- The misleading comment in the old repo (`foc.h:267`, "Back-EMF based velocity observer")
  is wrong; do not carry that label forward.

## Files affected

- docs/spec/11_feedback_and_encoders.md
- include/mc_state_estimator.h
- requirements.yaml

## Open questions

- Default observer gains for the reference axis (port from old config; confirm on hardware).
