# MAVLink Telemetry Adapter

## Goal

Publish mission feedback to MAVLink without coupling flight control to the
external telemetry consumer.

## Implementation

1. Add an optional MAVLink endpoint and the `pymavlink` dependency.
2. Retain a latest-feedback snapshot on the runtime blackboard for independent
   readers.
3. Run a MAVLink-owned publisher worker that sends heartbeat, attitude, and
   altitude at the agreed rates from that snapshot.
4. Convert Bullet FLU/Z-up attitude to MAVLink FRD/NED, omit stale or missing
   sensor messages, and isolate runtime adapter failures from mission control.
5. Add unit and lifecycle coverage for the blackboard, packet conversion,
   scheduling, endpoint validation, and shutdown.

## Interface changes

- `bullet-app --mavlink-endpoint CONNECTION` selects the publish-only MAVLink
  connection; it defaults to `udpout:127.0.0.1:14550` for mission runs.
- No existing ZMQ topics, recording schemas, commands, or mission behavior
  change.

## Verification

Passed: `uv run --locked pytest -q --ignore=tests/test_rc_adapter.py` (144
passed). The excluded adapter test has a pre-existing import of missing
`bullet_app.msp_flight.body_to_world`.

## Scope and defaults

- Heartbeat: 1 Hz, attitude and altitude: 20 Hz.
- Feedback is the sole source; stale data older than 0.2 s is not sent.
- MAVLink uses system ID 1 and `MAV_COMP_ID_AUTOPILOT1`.
- Override the default output with `--mavlink-endpoint` when another consumer
  endpoint is required.

## Status

Implemented

## Implementation commit

`76aa876`
