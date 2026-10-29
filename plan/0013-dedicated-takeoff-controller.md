# Dedicated aggressive takeoff controller

## Goal

Give heavy-quad takeoff its own application-owned vertical PID so it can
command up to 4 m/s during the 20 m climb without changing hover, target
search, or LOS control behavior.

## Implementation steps

1. Add a pure `bullet_app.controller.TakeoffController` with bounded integral
   state and vertical-reference slew limiting.
2. Add optional takeoff tuning fields to the shared LOS configuration and set
   aggressive values in both heavy-quad mission profiles.
3. Add a vertical-reference override to the reusable hold/control path.
4. Use the takeoff controller only in `TAKEOFF` and `TAKEOFF_SPOOL`, then hand
   off to the existing hold controller without a reference discontinuity.
5. Add controller, configuration, mission, and handoff regression coverage.

## Interface changes

New application-owned controller interface. Shared tuning accepts optional
`takeoff_kp_s_inv`, `takeoff_ki_s2_inv`, `takeoff_max_climb_m_s`, and
`takeoff_vertical_slew_m_s2` fields with legacy-safe defaults.

## Verification

Run focused `bullet_app` mission/settling tests and the `los_tracking` control
tests, plus profile parsing and controller unit tests.

## Scope and defaults

Heavy internal and Betaflight profiles use Kp 1.5, Ki 0.25, 4 m/s maximum
climb, and 8 m/s² reference slew. Racer profiles retain the existing default.

## Status

Pending

## Implementation commit

Pending
