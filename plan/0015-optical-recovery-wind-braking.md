# Wind-robust optical LOS recovery

## Goal

Keep roll and vertical corrections active during bounded visual recovery so
wind-driven drift does not immediately carry the target to both image edges.

## Implementation steps

1. Reuse the last filtered LOS bearing and rate during `RECOVER`, extrapolated
   only within the existing recovery deadline.
2. Feed those optical errors into the existing controller with approach pitch
   disabled, preserving roll and vertical correction without adding a velocity
   or feedback-schema dependency.
3. Add navigator regression tests for clipped-target recovery and recovery
   timeout behavior.
4. Run focused and full tracking/app test suites.

## Interface changes

No public message, feedback, MAVLink, or configuration schema changes.

## Verification

Focused navigator tests, full `los_tracking` tests, and `bullet_app` mission
and telemetry tests must pass.

## Scope and defaults

Recovery uses the existing `recovery_timeout_s` as the maximum optical
extrapolation window. If no bearing history exists, the current neutral
recovery behavior remains unchanged.

## Status

Implemented; tests passed

## Implementation commit

`abc7814` (`los_tracking`)
