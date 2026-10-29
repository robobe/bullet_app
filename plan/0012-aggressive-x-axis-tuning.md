# Aggressive X-axis tracking tuning

## Goal

Increase lateral target-centering authority for the heavy quad after the
latest flight reached the roll limit and lost the target on the right side.

## Implementation steps

1. Raise heavy-quad roll authority and roll-reference slew.
2. Increase azimuth proportional response and reduce excess derivative
   damping in both internal and Betaflight heavy-quad mission profiles.
3. Keep the visual azimuth rejection envelope and combined tilt safety limit
   unchanged.
4. Parse the profiles and run focused mission/configuration tests.

## Interface changes

No code or schema changes. Existing tuning fields receive new heavy-quad
profile values.

## Verification

Parse both YAML profiles and run the focused `bullet_app` mission/configuration
tests.

## Scope and defaults

Only `bullet_profile_heavy_quad.yaml` and `betaflight_los_heavy_quad.yaml` are
changed. Racer and standard Betaflight profiles retain their current tuning.

## Status

Pending

## Implementation commit

Pending
