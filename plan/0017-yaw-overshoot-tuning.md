# Plan 0017: Yaw overshoot tuning

## Goal

Reduce first-approach yaw overshoot in the heavy-quad Betaflight mission profile using a parameter-only change.

## Implementation steps

1. Lower `yaw_gain_s_inv` in the active `betaflight_los_heavy_quad` profile from 4.0 to 2.5.
2. Preserve the yaw-rate limit and all controller code.
3. Run focused mission/search tests and validate the YAML configuration.

## Interface changes

None. This changes only a mission tuning value.

## Verification

Run the focused `bullet-app` mission and search tests, then load the YAML through the existing configuration parser.

## Scope and defaults

The change applies to the heavy-quad Betaflight profile used by the reviewed run. Other profiles remain unchanged.

## Status

Implemented

## Implementation commit

`3db6fbf`
