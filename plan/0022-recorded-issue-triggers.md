# Recorded tracking issue triggers

## Goal
Explain tracking issues with triggering measurements and active parameters,
not the resulting navigator state. Preserve onset evidence before recovery resets.

## Implementation
Add optional vision/navigation trigger diagnostics with reason, condition,
measurements and parameters. Capture freshness, geometry, alignment, tilt,
authority, timeout and nonnumeric rejection evidence at decision branches.
Carry these through existing mission recording without changing flight decisions.
FlightLens uses a Qt-independent formatter and preceding same-episode recorded
app configuration for legacy evidence. Missing evidence is explicit, never filled
with current defaults. Preserve event grouping and onset details.

## Verification
Controller branch/boundary tests, serialization/reader tests, formatter and
 offscreen UI tests; run los_tracking, bullet_app and flight_lens.

## Scope/defaults
No tuning or new issue categories. Old recordings remain readable; reconstructed
comparisons are labeled and cannot recover unrecorded values.

## Status
Pending. Implementation commit: Pending.
