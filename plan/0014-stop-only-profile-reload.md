# Stop-only mission profile reload

## Goal

Let Bullet View request a mission-profile reload over MAVLink while the
scenario is stopped, so tuning YAML can be edited and retested without a full
process rebuild.

## Implementation

- Add shared MAVLink reload-command constants.
- Add a dedicated control listener on UDP 14551 while preserving telemetry on
  UDP 14550.
- Reload `--mission-config` only in the mission thread and only for stopped
  phases; reject active-flight requests.
- Add a Bullet View reload button and ACK/status feedback.

## Verification

Run shared contract, bullet-app, and bullet-view tests, then verify the
Stop → edit YAML → Reload profile → Start workflow.

## Status

Pending

## Implementation commit

Pending
