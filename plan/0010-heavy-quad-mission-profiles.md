# Heavy quad mission profiles

## Goal

Keep the mission control profile aligned with the selected physics drone so heavy_quad runs are accepted by the strict platform contract.

## Implementation

- Add explicit internal and Betaflight heavy_quad mission profiles with the existing tuning and camera settings.
- Set the heavy vehicle limits to 5 kg and 72.1035 N total thrust.
- Map each backend/drone selector combination to matching physics and mission profiles in the workspace task.
- Add mission configuration coverage for both heavy profiles.

## Verification

- Load and configure both heavy profiles without a platform/control mismatch.
- Preserve rejection of intentionally mismatched platform values.
- Validate the workspace JSON and run focused mission and external-runner tests.

## Scope and defaults

- Heavy_quad initially reuses racer mission tuning.
- Betaflight PWM calibration remains the existing provisional calibration until measured for heavy_quad.

## Status

Implemented and verified with focused mission tests.

## Implementation commit

`32adb71` (bullet_app), `ecf9300` (workspace profiles)
