# MSP Telemetry Scheduler

## Goal

Move periodic Betaflight status, attitude, altitude, and RC coordination into
one application-owned scheduler. Publish cached current state through
`MspFlight.snapshot()`.

## Implementation

1. Replace the RC-only scheduler with an MSP scheduler that maintains the
   periodic task schedule and latest telemetry state.
2. Request status at 10 Hz, attitude at 50 Hz, and altitude at 20 Hz, while
   preserving safe latest-value RC delivery and reconnect handling.
3. Keep `MspFlight` as the lifecycle facade and make `snapshot()` return the
   stale-filtered cached attitude, altitude, and status state.
4. Update focused scheduler tests and retain runtime consumption through the
   existing snapshot interface.

## Interface changes

- `MspFlight.snapshot()` now includes the latest altitude sample as its second
  tuple member.
- `RcScheduler` is replaced internally by `MspScheduler`; no application
  runtime interface changes.

## Verification

Passed: `uv run --locked --package bullet-app pytest -q --ignore=tests/test_rc_adapter.py`
(134 passed). The excluded adapter test has a pre-existing import of missing
`bullet_app.msp_flight.body_to_world`.

## Scope and defaults

- Status remains at 10 Hz, attitude is 50 Hz, and altitude is 20 Hz.
- Samples older than 0.2 seconds and all disconnected state are omitted from
  snapshots.
- No configuration, dependency, or telemetry-schema changes are introduced.

## Status

Implemented

## Implementation commit

`af360c7`
