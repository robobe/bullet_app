# Runtime Publisher

## Goal

Move mission-runtime ZMQ publication into one dedicated worker, keeping control
free of socket ownership and outbound serialization.

## Implementation

1. Extend the runtime blackboard with latest RC/status/config/MSP publication
   state, bounded ordered output queues, per-topic drop counters, and a
   priority final-disarm command.
2. Add a publisher worker that exclusively owns the command and analysis PUB
   sockets, sends latest RC/status/config/MSP state at their existing rates,
   and drains bbox/LOS/MSP-delivery events nonblocking.
3. Make LOS packet scheduling transport-free: control prepares ready packets
   and queues them instead of using a socket-owning LOS publisher.
4. Refactor control and vision to write publication intents to the blackboard,
   then coordinate shutdown so LOS shutdown and final disarm flush first.
5. Publish output queue/drop diagnostics through mission and MSP telemetry.

## Interface changes

- External ZMQ topics and payloads remain unchanged.
- `RuntimeBlackboard` gains internal publication-state and event methods.

## Verification

Passed: `uv run --locked --package bullet-app pytest -q --ignore=tests/test_rc_adapter.py`
(137 passed). The excluded adapter test has a pre-existing import of missing
`bullet_app.msp_flight.body_to_world`.

## Scope and defaults

- Latest RC is published at 100 Hz; expired non-final commands are skipped.
- Bbox, LOS, and MSP delivery queues have capacity 64 and drop oldest on
  overflow; final disarm is retained separately.
- No dependency or configuration changes are introduced.

## Status

Implemented

## Implementation commit

`0e440ed`
