# Configurable Betaflight control connection

## Goal

Configure Betaflight MSP control as TCP or serial from the mission YAML, while keeping MAVLink telemetry configuration separate.

## Implementation steps

- Add typed `control.target` and `control.connection` settings to `los_tracking.NavigatorConfig` with strict transport-specific validation.
- Build `msp_link.TcpTransport` or `SerialTransport` from those settings in `bullet_app`.
- Preserve internal-backend behavior, update Betaflight profiles with documented defaults, and make profile reload replace MSP transport when connection settings change.
- Add configuration, transport-construction, and reload regression tests plus documentation.

## Interface changes

- Mission YAML accepts `control.backend`, `control.target`, and `control.connection.transport` with TCP (`host`, `port`) or serial (`path`, `baud_rate`) fields.
- `MspFlight` accepts an injected transport/client while retaining its default TCP behavior.

## Verification

- Run focused `los_tracking` and `bullet_app` tests.
- Run the workspace test suites and `git diff --check`.

## Scope and defaults

- Betaflight defaults to SITL over TCP at `127.0.0.1:5761`.
- MAVLink CLI endpoints are unchanged.

## Status

Pending.

## Implementation commit

Pending.
