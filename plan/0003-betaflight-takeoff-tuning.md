# Betaflight Takeoff Tuning

## Goal

Prevent the simulated Betaflight vehicle from striking the ground during motor
spool-up, then calibrate and record the lowest safe takeoff and hover PWM pair.

## Implementation

1. Add Betaflight-only takeoff and hover PWM tuning fields with validation.
2. Add a confirmed-ascent takeoff spool phase, bounded fault handling, and
   hover-PWM feed-forward around the existing physical controller output.
3. Extend the headless flight check to accept a candidate mission profile and
   add a tuner that sweeps candidates, validates the final pair three times,
   writes the profile, and saves a report.
4. Add focused state-machine, mapping, configuration, and tuner tests.

## Interface changes

- The dedicated Betaflight hover YAML profile gains `betaflight_takeoff_pwm`
  and `betaflight_hover_pwm` tuning fields.
- `check_app_flight.py --hover-check` accepts `--mission-config` as its
  candidate profile.

## Verification

- Passed: `uv run --locked --package bullet-app pytest -q --ignore=tests/test_rc_adapter.py`
  (132 passed). The excluded adapter test has its pre-existing missing
  `body_to_world` import.
- Passed: `uv run --locked pytest -q` in `los_tracking` (113 passed) and
  `bullet_render` (35 passed).
- Passed: three full simulated Betaflight hover checks at 1700 takeoff PWM and
  1500 hover PWM. Peak altitude errors were 0.244 m, 0.369 m, and 0.414 m;
  all completed the 20-second hover dwell.

## Scope and defaults

- Only Betaflight uses the calibrated fields; all other backends preserve their
  existing RC conversion.
- The vehicle must rise 0.25 m with at least 0.1 m/s upward velocity within
  2 seconds of spool-up or fault and disarm.
- Select the lowest pair that passes three complete 20-second hover checks.

## Status

Pending

## Implementation commit

Pending
