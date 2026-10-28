# MSP RC Scheduler

## Goal

Keep the latest RC command streaming through MSP at 50 Hz, independently of
the mission loop, beginning in an all-1000-PWM safe state and returning to it
after an MSP disconnect.

## Implementation

1. Add an application-owned, heap-scheduled RC worker with a latest-value
   command slot and a 20-ms repeating task.
2. Start and stop that worker with `MspFlight`; make `MspFlight.update` replace
   the worker's command instead of writing MSP RC directly.
3. Define named PWM, channel-count, rate, and safe-command constants.
4. Record each successfully transport-sent MSP RC frame in the shared client,
   drain those records in the mission runtime, and publish them as an MCAP
   analysis topic through the existing recorder path.
5. Poll Betaflight attitude at 25 Hz and include it in the MSP snapshot and
   MCAP analysis stream for takeoff-failure diagnosis.
6. Add focused tests for safe startup, latest-command replacement, disconnect
   reset, transport delivery records, MSP attitude polling, and their MCAP
   schema.

## Interface changes

- `MspFlight.update(channels)` remains the runtime-facing interface, but it is
  asynchronous and latest-command-wins.
- `msp_link.Client.drain_rc_deliveries()` exposes RC frames that successfully
  reached the transport. `analysis/app/msp_rc` is a new 50-Hz MCAP topic.
- `MspFlight.snapshot()` returns the latest Betaflight attitude when available.

## Verification

- Passed: `uv run --locked --package msp-link pytest -q` (9 passed).
- Passed: `uv run --locked --package bullet-msgs pytest -q` (149 passed).
- Passed: `uv run --locked --package mcap-bagger pytest -q tests/test_config.py`
  (29 passed), and the rendered recorder config resolves `analysis/app/msp_rc`.
- Passed: `uv run --locked --package bullet-app pytest -q
  --ignore=tests/test_rc_adapter.py` (130 passed). The excluded test has a
  pre-existing import of missing `bullet_app.msp_flight.body_to_world`.

## Scope and defaults

- The scheduler requests delivery every 20 ms (50 Hz); it skips missed slots
  rather than bursts frames.
- The safe command uses neutral roll/pitch/yaw, minimum throttle and arm, a
  high mode channel, and minimum PWM for remaining AUX channels.
- Reconnects remain safe until a later `update` call.
- MCAP records actual transport delivery, not only the mission's requested RC
  topic. No configuration option or dependency is added.
- Betaflight attitude is requested every 40 ms and remains subject to the
  existing stale-snapshot handling.

## Status

Pending

## Implementation commit

Pending
