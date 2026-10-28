# Runtime Blackboard

## Goal

Keep cross-thread application state in one stamped runtime blackboard and move
ZMQ subscription into a dedicated receiver worker, so the mission control loop
never waits for incoming telemetry.

## Implementation

1. Add a lock-protected runtime blackboard for stamped config, feedback,
   terminal state/end events, vision/display, mission status, and receiver
   health.
2. Add a receiver worker that exclusively owns the ZMQ SUB socket, validates
   inbound messages, and writes them to the blackboard.
3. Keep ordered feedback in a capacity-eight FIFO, dropping its oldest entry
   on overflow; keep terminal events lossless.
4. Refactor the control loop into input application, control, and publication
   steps, draining at most four feedback frames per 100-Hz tick.
5. Publish feedback queue depth and drop count in mission and MSP diagnostics.

## Interface changes

- ZMQ topics and payloads are unchanged.
- `MissionRuntime` no longer exposes its generic lock or direct state fields;
  consumers read display/status from its blackboard.

## Verification

Passed: `uv run --locked --package bullet-app pytest -q --ignore=tests/test_rc_adapter.py`
(136 passed). The excluded adapter test has a pre-existing import of missing
`bullet_app.msp_flight.body_to_world`.

## Scope and defaults

- Blackboard receipt times use local monotonic seconds and retain all payload
  domain timestamps.
- Mission remains the sole owner of control/filter/phase state.
- No dependencies or configuration fields are added.

## Status

Pending

## Implementation commit

Pending
