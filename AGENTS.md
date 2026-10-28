# Project guidance

These instructions apply throughout this repository.

## SOLID design guide

- **Single responsibility:** separate CLI parsing, configuration loading, simulation orchestration, environment lifecycle, physics, rendering, and telemetry.
- **Open/closed:** extend physics models and consumers through focused interfaces without rewriting orchestration.
- **Liskov substitution:** implementations must preserve interface units, lifecycle, stepping semantics, and error behavior.
- **Interface segregation:** keep interfaces small; physics consumers must not depend on rendering or plotting operations.
- **Dependency inversion:** orchestration depends on project-owned contracts; concrete library integrations stay in adapters.

Prefer simple composition and small contracts. Introduce abstractions only when they express a real component boundary.

## Constants

- Replace unexplained numeric literals (magic numbers) with descriptive named constants, especially for physical values, PID gains, limits, tolerances, durations, and channel assignments.
- Use `UPPER_SNAKE_CASE` for constant names. Include units where applicable, such as `TARGET_ALTITUDE_M` or `HOLD_DURATION_S`, so each value's meaning is clear.
- Group all module-level constants at the top of the module, immediately after the imports and before classes or functions. Keep constants out of function bodies and avoid scattering them throughout the module.


## Libraries and tooling

- Every field in a YAML configuration file must have an adjacent comment explaining its meaning and how to use it. Include units, accepted values or ranges, and default or omission behavior where applicable. Keep these comments current whenever configuration fields change.
- Use **Click** inside a `CliParser` helper class. The helper returns parsed arguments; it must not launch simulation, load YAML, or configure logging.
- Use **Loguru** for application diagnostics. Configure logging once at each example's application entry point. Library components must not configure global sinks.
- Keep diagnostic logging separate from simulation telemetry recording. Do not use diagnostic messages as the telemetry storage format.
- Use **uv** for dependency management and command execution. Update `pyproject.toml` and `uv.lock` together when dependencies change.


## Implementation plans and commit records

- Before starting an authorized implementation, save the agreed plan as `plan/NNNN-short-name.md`. Use increasing four-digit numbers and descriptive filenames.
- Each plan records its goal, implementation steps, interface changes, verification, scope/defaults, status, and implementation commit hash. Use `Pending` until an implementation commit exists; do not invent a hash.
- Maintain [plan/README.md](plan/README.md) as the plan menu. Add a link, short description, status, and implementation commit hash for each plan.
- Keep the saved plan aligned with agreed scope changes, and update its status and verification outcome when implementation is complete.
- After committing an implementation, obtain the actual hash using Git and update it in both the plan and `plan/README.md`. Record additional implementation commits when a plan spans multiple commits.
- Commit these post-commit metadata updates in a separate documentation commit when committing is requested. Record the implementation commit, not the metadata commit; do not amend a commit merely to insert its own hash.
- Saving plans does not authorize implementation, committing, or pushing. In Plan mode, do not write files; persist the plan once execution mode is enabled.
