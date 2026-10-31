# MAVLink AGL telemetry

## Goal

Propagate the physics rangefinder AGL through flight feedback, publish standard MAVLink DISTANCE_SENSOR telemetry, and preserve the value for MCAP recording.

## Implementation steps

1. Add an optional rangefinder field to the app-facing flight feedback contract.
2. Populate it from the physics observation and publish DISTANCE_SENSOR.
3. Update profile configuration and recorder/contract tests.

## Interface changes

- Optional feedback `rangefinder` object with `distance_cm` and `valid`.
- MAVLink `DISTANCE_SENSOR` uses centimetres, invalid sentinel, downward orientation, and source timestamp.

## Verification

- Validate legacy and new feedback packets, MAVLink valid/invalid readings, and MCAP recording finalization.

## Status

Implemented; focused tests passed

## Implementation commit

`7a5774f`
