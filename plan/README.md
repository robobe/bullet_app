# Plans

| Plan | Description | Status | Implementation commit |
| --- | --- | --- | --- |
| [0001 Bullet App SOLID + CLI Refactor](0001-bullet-app-solid-cli-refactor.md) | Separate Click parsing and application/runtime orchestration while preserving the CLI. | Implemented | `ffc057d` (`5c3f221` lockfile) |
| [0002 MSP RC Scheduler](0002-msp-rc-scheduler.md) | Stream and record latest MSP RC delivery at 50 Hz with a firmware-compatible safe fallback. | Pending | Pending |
| [0003 Betaflight Takeoff Tuning](0003-betaflight-takeoff-tuning.md) | Calibrate safe Betaflight takeoff and hover PWM in simulation. | Pending | Pending |
| [0004 MSP Telemetry Scheduler](0004-msp-telemetry-scheduler.md) | Schedule MSP status, attitude, and altitude polling with cached snapshots. | Implemented | `af360c7` |
| [0005 Runtime Blackboard](0005-runtime-blackboard.md) | Isolate ZMQ reception from mission control with stamped shared runtime state. | Implemented | `c95784a` |
| [0006 Runtime Publisher](0006-runtime-publisher.md) | Isolate ZMQ publication from mission control with scheduled shared output state. | Implemented | `0e440ed` |
| [0007 MSP Analysis Schema Compatibility](0007-msp-analysis-schema-compatibility.md) | Keep current MSP diagnostics compatible with strict MCAP recording schemas. | Implemented | `2aa461a` (`bullet_msgs`), `4f2be3e` (`mcap_bagger`) |
| [0008 MAVLink Telemetry Adapter](0008-mavlink-telemetry-adapter.md) | Publish isolated MAVLink heartbeat, attitude, and altitude from mission feedback. | Implemented | `76aa876` |
| [0009 Configurable yaw search timeout](0009-configurable-yaw-search-timeout.md) | Expose and extend the yaw-search deadline through navigator tuning. | Implemented | `6ace504`, `93cb08d`, `f3d0d74` |
| [0010 Heavy quad mission profiles](0010-heavy-quad-mission-profiles.md) | Align mission vehicle limits with the selected heavy_quad physics profile. | Implemented | `32adb71` (`ecf9300` workspace profiles) |
| [0011 Vertical LOS centering](0011-vertical-los-centering.md) | Keep the target on the image Y centerline and brake excessive descent with thrust. | Implemented; tests passed | `7a76c43`, `acb0b5d` |
| [0012 Aggressive X-axis tracking tuning](0012-aggressive-x-axis-tuning.md) | Increase heavy-quad lateral tracking authority after roll saturation in the latest flight. | Pending | Pending |
| [0013 Dedicated aggressive takeoff controller](0013-dedicated-takeoff-controller.md) | Use an application-owned vertical PID for a 4 m/s heavy-quad takeoff. | Implemented; tests passed | `89049df`, `4fe4147`, `84a11d0` |
