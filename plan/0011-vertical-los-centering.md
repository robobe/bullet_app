# Vertical LOS centering and braking

## Goal

Keep the tracked target on the image Y centerline and arrest excessive
downward velocity with faster vertical reference response and additional
collective thrust.

## Implementation steps

1. Add optional elevation-reference and braking-slew tuning fields to the LOS
   configuration and shipped profiles.
2. Track zero optical elevation while retaining the acquired bearing for target
   association and rejection.
3. Use the faster braking slew when measured descent requires a climb response.
4. Add focused navigator, control, and closed-loop regression coverage.

## Interface changes

The shared LOS tuning YAML accepts `track_elevation_deg` and
`vertical_brake_slew_m_s2`; both are optional in code with safe defaults.

## Verification

`los_tracking` full test suite and focused `bullet_app` mission/RC/telemetry
tests pass.

## Scope and defaults

Default tracking elevation is 0 degrees. Existing horizontal yaw-centering,
association limits, thrust limits, and recovery behavior remain in place.

## Status

Implemented; commit pending.

## Implementation commit

Pending
