# Bullet App SOLID + CLI Refactor

## Goal

Keep the existing Bullet CLI and flight protocols stable while separating Click
argument parsing, application mode selection, detector execution, and mission
execution.

## Implementation

1. Add a Click-based `CliParser` that returns typed application options without
   loading configuration, launching the app, or configuring diagnostics.
2. Make `main.py` a composition root that configures Loguru once and delegates
   to an application service.
3. Move detector-only and mission UI orchestration into dedicated runtime
   components, keeping `Mission` as the domain state machine and existing ZMQ
   adapters/protocols unchanged.
4. Add direct Click and Loguru dependencies and document the maintained CLI
   contract through focused tests.

## Interface changes

- `bullet-app` retains every existing option, default, validation rule, and
  exit code.
- `bullet_app.main.main(argv=None)` remains callable by tests and examples.
- The new parser and options types are internal Python interfaces only.

## Verification

- Passed: `uv run --locked --package bullet-app pytest -q` for the focused
  CLI, application, detector runtime, mission, MSP, telemetry, status,
  receiver, detector, pipeline, recording, and search suites (111 passed).
- Full package collection remains blocked by the pre-existing
  `test_rc_adapter.py` import of missing `bullet_app.msp_flight.body_to_world`.

## Scope and defaults

- Diagnostics use Loguru; recorded telemetry remains ZMQ/MCAP data.
- No subcommands, renamed flags, new protocols, or mission behavior changes.

## Status

Implemented

## Implementation commits

- `ffc057d49ac0fa6b35c1dfb04c01638cfd1cb9f6` — Bullet App refactor.
- `5c3f22195f6103b42ba6d3088ae214cf83d1d085` — workspace lockfile update.
