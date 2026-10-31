# LOS roll alignment

## Goal

Preserve yaw-only target centering, then ensure the LOS controller produces and publishes the expected roll maneuver when horizontal target error develops.

## Implementation steps

- Verify the mission transition keeps roll/pitch level during `YAW_CENTER` and hands control to `LosNavigator` at LOS acquisition.
- Verify azimuth-to-roll sign, demand propagation, RC encoding, and roll/slew limits without changing the MAVLink or payload contracts.
- Add regression tests for yaw-only centering, nonzero LOS roll direction, and configured roll limits.
- Run focused tests and the heavy Betaflight scenario/replay checks.

## Interface changes

None. Existing mission diagnostics remain the verification surface.

## Verification

Focused search/mission tests: 51 passed. Full `bullet_app` suite: 153 passed. Implementation commit is pending.

## Scope/defaults

Use yaw first for image centering. Use the heavy Betaflight profile's existing 30 degree roll limit and 60 degree/second slew limit. Do not retune yaw in this change.

## Status

Pending

## Implementation commit

Pending
