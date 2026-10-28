# Configurable yaw search timeout

## Goal

Expose the yaw-search deadline in navigator tuning and set shipped profiles to
60 simulation seconds.

## Implementation

1. Add `yaw_search_timeout_s` to `los_tracking.Tuning` and all shipped profiles.
2. Use the configured value in `bullet_app` mission timeout handling.
3. Update status text, documentation, and focused tests.

## Interface changes

- Navigator tuning accepts `yaw_search_timeout_s` in seconds.
- Existing profiles default to 60 seconds; no protocol or CLI changes.

## Verification

Pending

## Status

Pending

## Implementation commit

Pending
