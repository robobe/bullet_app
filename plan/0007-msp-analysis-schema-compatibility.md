# MSP Analysis Schema Compatibility

## Goal

Allow the shared strict `analysis_msp` recording contract to accept the current
Betaflight delivery and runtime-health diagnostics, so valid MSP analysis does
not leave an MCAP recording partial.

## Implementation

1. Add optional attitude and altitude delivery metric objects to the shared
   MSP analysis schema, using the existing request/response timing fields.
2. Add optional input and output queue/drop diagnostics to `sensor_feedback`.
3. Extend the strict shared-schema fixture with the complete current packet.
4. Add an MCAP recorder readback regression proving that packet finalizes a
   `.mcap` recording.

## Interface changes

- `analysis_msp` accepts the current optional diagnostics while retaining
  `additionalProperties: false` at every affected object.
- No app topic, payload field, configuration, or dependency changes.

## Verification

Passed: `uv run --locked pytest -q` in `bullet_msgs` (149 passed).

Passed: `uv run --locked pytest -q tests/test_analysis_recording.py` (4
passed) and `uv run --locked pytest -q` in `mcap_bagger` (86 passed).

Blocked: the isolated Betaflight hover/recording check could not start because
an existing user-owned `run_flight.py` session already held MSP port 5761. The
new recorder regression directly verifies that the complete current MSP payload
finalizes as `.mcap` without a `.partial` file.

## Scope and defaults

- New fields are optional to preserve older packet and recording validity.
- Existing partial recordings are not migrated or repaired.

## Status

Implemented

## Implementation commit

`2aa461a` (`bullet_msgs`), `4f2be3e` (`mcap_bagger`)
