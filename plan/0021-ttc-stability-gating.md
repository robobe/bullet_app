# TTC stability gating

## Goal

Prevent noisy, tiny visual detections from driving descent-feasibility decisions before TTC is stable.

## Implementation steps

- Require a configurable minimum bbox area and consistent positive growth in `TtcEstimator`.
- Require the fit to remain qualified for the configured 0.3 s TTC dwell before returning a usable estimate.
- Apply the gate to existing descent-feasibility supervision without adding an AGL interface.
- Update all shipped YAML profiles and add estimator regression coverage.

## Interface changes

Add `tuning.ttc_min_area_px2` with a default of 256 pixels². Existing fields and message formats remain unchanged.

## Verification

`los-tracking`: 124 tests passed. `bullet_app`: 153 tests passed. Latest-recording replay produced no qualified TTC during the early 30–31 s false-positive window.

## Scope/defaults

Use visual stability only. Require at least 60% positive size-growth steps. Keep existing TTC residual, history, and dwell settings.

## Status

Implemented

## Implementation commit

`504fc99` (`los_tracking`), `4241615` (`bullet_app`), `744968a` (workspace profiles)
