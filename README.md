# Bullet app

This project is a member of the [Bullet workspace](../README.md). All six
projects share `../.venv` and `../uv.lock`; run uv commands with the
workspace flags below to retain every project and optional dependency.

Standalone Python 3.12+ uv application, intended to own the drone application
state machine. This version only runs the camera → red detector → bounding-box
publisher pipeline. Flight commands and state transitions are deferred.

Keep `bullet_app`, `los_tracking`, `bullet_msgs`, `mcap_bagger`, and `bullet_render`
as sibling folders under `~/projects/bullet`.

```bash
cd ~/projects/bullet/bullet_app
uv sync --locked --all-packages --all-extras --all-groups
uv run --locked --all-packages --all-extras --all-groups bullet-app --transport-config ../bullet_render/configs/transport.json --debug
# Equivalent example entry point:
uv run --locked --all-packages --all-extras --all-groups python examples/live_los.py --transport-config ../bullet_render/configs/transport.json --debug
```

The Bullet workspace includes **Bullet: App (debug)**. **Bullet: Physics + render
+ app** and **Bullet: Run all** launch it with Godot, physics, and the MCAP
logger. Click Start in Godot to produce images and start episode recording.
The debug application provides the camera and bounding-box preview.

The receiver moved here from `los_tracking`; it uses `bullet_msgs.FrameReader`
for exact topic matching, SHM validation, private BGR copies, duplicate/stale
skips, row padding, and publisher restarts. The red detector now lives in
`bullet_app.detector`; its image-to-bbox behavior is unchanged. `los_tracking`
now owns the separate bbox/barometer/attitude-to-RC library. This live app still
runs detection and recording only; sensor reception and flight-command transport
are deferred. See the [navigator calling contract](../los_tracking/README.md).
The former live example also moved here.

## Input and output

`--transport-config` is required. `--topic` defaults to `camera/front/image`.
`--bbox-endpoint` defaults to `tcp://127.0.0.1:5558`; the app binds a PUB socket
and MCAP connects. It must differ from the renderer/physics/control endpoints.
When changing this port, also change the bbox topic's `endpoint` in
`../bullet_render/configs/recorder.yaml`.

Every processed frame emits exactly two ZMQ parts: UTF-8 topic
`tracking/front/bbox` and a JSON payload using `bullet_msgs:bbox.json`:

```json
{"schema_version":1,"image_topic":"camera/front/image","session":"0123456789abcdef0123456789abcdef","sequence":42,"capture_ns":123456789,"bbox":{"x":100,"y":80,"width":40,"height":30}}
```

Position is the bounding box's top-left pixel, x right and y down, with width and
height in pixels. It is not a 3D target pose. `bbox: null` explicitly reports no
red target in that frame. Session, sequence, and capture_ns come unchanged from
the source image descriptor. capture_ns is monotonic time, not Unix time. MCAP
uses arrival time; source identity can match a bbox to a descriptor but recorded
RawImage messages do not preserve that identity, so exact MCAP image association
is not guaranteed.

No new frame means no new result. The app waits for the renderer and resumes on
restart. Debug mode shows yellow boxes and Waiting/Live/Stale status (stale after
one second without a new image). A no-target frame clears the box. Q, Esc,
window close, or Ctrl+C releases sockets and mappings. Normal mode opens no
window. Malformed input or bind failures exit with a diagnostic and nonzero code.

Publication uses nonblocking PUB/SUB with a 64-message send high-water mark.
Disconnected/slow subscribers and subscription startup can lose results; there
are no delivery acknowledgments or retries. MCAP's existing episode lifecycle
still controls when results are recorded. HSV detection selects the largest red
object and can select unrelated red objects. SHM requires the same Linux host/user
as Godot; the debug view requires a graphical display.

## Checks

```bash
uv run --locked --all-packages --all-extras --all-groups pytest -q
```

Tests include TCP/SHM reception, debug/non-GUI paths, JSON contracts, and a real
image-to-detection-to-MCAP round trip using synthetic images.

## Red detector ownership

`from bullet_app.detector import detect_red_box` accepts a nonempty H×W×3 uint8
BGR array and returns `(x, y, width, height)` or `None`, preserving the input.
It selects the largest red contour using HSV hue 0–10 or 170–180, saturation
>=100, value >=80, 3×3 opening, contour area >=4 and width/height >=3 pixels.
This replaces the former `los_tracking.detect_red_box` import.

## App-owned flight mission

The workspace **Bullet: Physics + render + app** and **Bullet: Run all** tasks now
run the external-RC physics example and the mission-enabled app. Start physics,
app and Godot, wait for `Mission READY`, then press Start in Godot. Start is
rejected by physics until fresh disarmed app readiness frames arrive.

```bash
# bullet_drone terminal
uv run --locked --all-packages --all-extras --all-groups python -m examples.external_rc.main \
  --render-config ../bullet_render/configs/physics.json
# bullet_app terminal
uv run --locked --all-packages --all-extras --all-groups bullet-app \
  --transport-config ../bullet_render/configs/transport.json \
  --mission-config ../los_tracking/examples/bullet_profile.yaml --debug
```

Each Start creates a new episode and runs `SENSOR_WARMUP → ARMING → TAKEOFF →
HOVER → WAIT_TARGET → YAW_SEARCH → YAW_CENTER → YAW_SETTLE → LOS_ACQUIRE → LOS_APPROACH`.
A target already visible after hover skips the blind scan but must still center
and settle before LOS. The app first sends
disarmed Angle frames so grounded physics produces sensor samples, then waits for applied
arming confirmation. It climbs 20 m above the initial barometer reading with a
1 m/s vertical reference limit. Hover requires altitude error <=0.25 m
(<=0.5 m with Betaflight),
|settling trend speed| <=0.2 m/s and roll/pitch/heading errors <=5 degrees continuously
for 3 simulation seconds; deviations restart the countdown. Takeoff/settling has
a 60 simulation second deadline. After hover, only a bbox captured after hover
completion can start LOS. If absent after a fresh frame check (at most 100 ms),
the app searches with positive yaw at 15 deg/s. One measured full circle without
a usable target freezes the episode. LOS faults, stale sensors, withdrawn intent
and expired commands also freeze it. Start/Reset creates fresh state; no automatic running takeover
is allowed after an app restart. Godot-confirmed target contact completes the
scenario; the app does not infer collision from bbox size or implement landing.

The controller receives barometer altitude and explicitly **ideal simulator
attitude**, with no XY position or velocity feedback. This is altitude/heading
hold, so wind or bias can cause horizontal drift. The LOS library receives the
hold filter state, bounded attitude history, heading, reference angles/vertical
speed and thrust integral through its public `ControlHandoff`. Camera calibration
and thrust/RC/rate settings must match the advertised physics platform.

The mission thread owns feedback, control, bbox and status sockets at 100 Hz.
A separate latest-frame worker owns image sockets/SHM and detection. The main
thread displays debug frames; GUI delays cannot block RC publication. Omitting
`--mission-config` preserves detector-only operation, also available as the
**Bullet: Detector only** task. **Bullet: Position flight** retains the square.

Transport ports: physics feedback/config/pose 5555, images 5556, Godot lifecycle
5557, bbox/mission diagnostics 5558, app RC 5559. RC uses all 18 integer PWM
channels, an episode owner and monotonic sequence, with validity <=100 ms
(shorter for LOS). Physics steps at 240 Hz and checks expiry before every step;
feedback publishes at 120 Hz. Sensor/image freshness and expiry use the same
host's monotonic clock; mission durations use simulation time. Each feedback and
stamped pose reuse one source identity and measurement timestamp. Image v2
`capture_ns` equals that source's `measurement_us * 1000`, independent of render
arrival/GPU latency. Legacy image v1 is detector-only, not mission eligible.

MCAP records intent, applied feedback, platform config, mission diagnostics,
poses, bboxes and images using the render recorder profile. PlotJuggler may
subscribe to multiple ports; use `source.simulation_time_s` for feedback and
`simulation_time_s` for mission plots. Bearing, LOS rates and TTC appear under
mission `diagnostics`. Requested RC and applied acknowledgements are separate.

## Confirmed scenario completion

The mission thread subscribes to `scenario/ended` and simulation state v2 on
physics telemetry (5555). A result for the current physics/episode/stream latches
`COMPLETED / target_hit`. It stops calling LOS and publishes inactive/disarmed RC.
Stale feedback, platform timeout, subsequent stopped packets and repeated events
cannot erase completion. A fresh episode clears the latch and controller state.
Stopped feedback with reason target_hit also supports late app startup while the
full confirmed result is recovered from repeated telemetry. No extra app port or
participant shutdown is required; the debug preview stays open.

## Focused LOS telemetry in degrees

Mission-enabled `bullet_app` publishes **tracking/front/los** on its existing app
endpoint (default `tcp://127.0.0.1:5558`). The default render recorder profile
records this JSON topic to MCAP. Detector-only mode does not create LOS results.
Normal LOS samples publish at **30 Hz**; mission/navigation state, stage, reason
and activity changes publish immediately. The publisher borrows the mission
thread's existing socket and adds no port or thread.

| Fields | Meaning / units |
|---|---|
| elevation_deg, azimuth_deg | Rotation-compensated LOS bearing, degrees; elevation positive down, azimuth positive left relative to held heading |
| elevation_rate_deg_s, azimuth_rate_deg_s | LOS angular rates in degrees/second |
| bearing_valid, rate_observable | Whether the corresponding bearing/rate estimates are currently usable |
| optical_ttc_s, ttc_valid | Optical TTC in seconds and qualified-estimate flag |
| ttc_reason, ttc_relative_residual | Qualification/rejection reason and dimensionless fit residual; smaller residual indicates a closer fit, not a probability |
| navigator_state, stage, reason | Actual navigator state, BEARING/TTC/TERMINAL stage (lowercase on wire), and current navigation/lifecycle reason |
| mission_phase, active | App mission phase and whether this packet contains a current LOS result |
| generated_us, navigation_sequence | Active result generation time in host monotonic microseconds and its library sequence |
| source | Physics/episode/stream identity, step, physics measurement_us and simulation_time_s |
| app_id, sequence | Publisher identity and telemetry publication sequence, distinct from library result sequence |

Inactive state-change markers are emitted before LOS starts and on Stop, fault,
collision completion, reset and graceful app shutdown. They clear bearing/rate/TTC
estimates and residual, set validity flags false, and have null navigation_sequence.
They retain the last actual navigator state/stage for the ending episode; a new
episode clears these. generated_us is the transition time for an inactive marker.
Repeated unchanged idle state is not sent as a periodic sample.

Unavailable or invalid estimates are **null**, never invented zeros or stale
values marked valid. A supplied TTC residual is retained for an active but
unqualified fit (for example noisy_growth). TTC comes from apparent bbox growth;
collision confirmation remains the separate renderer/physics scenario event.

Existing `app/drone_1/mission` diagnostics retain their native radians unchanged.
The focused topic converts only logged bearings/rates using degrees=rad*180/pi;
controller calculations remain in radians. For PlotJuggler, subscribe to this topic
on port5558, use source.simulation_time_s for simulation-time plots, and select the
*_deg, *_deg_s and optical_ttc_s values. MCAP timestamps remain Unix arrival time;
generated_us and source.measurement_us are host monotonic times.

## Scenario takeoff height

The renderer Scenario dialog selects a relative takeoff height together with the
starting position. The physics runner confirms it and includes it in v2 flight
feedback. The app latches that height for each new episode and climbs from the
initial barometer altitude to datum + selected height, then uses the existing
3 s settled hover and LOS flow. V1 feedback keeps the 20 m default. A running
episode's goal cannot change through duplicate or later feedback updates.

## Mission errors and LOS angle limits

The default/Bullet profile allows target azimuth within **±35 degrees** of the
held heading; elevation remains **±60 degrees**. The camera can see farther
sideways than this control envelope. Rejected detections cannot become valid
bearings or drive LOS control.

The console logs phase and cause changes, including rejections during acquisition:
`Mission LOS_ACQUIRE: Acquiring target: target rejected: azimuth +36.20 deg exceeds +/-35.00 deg`.
An acquisition timeout reports the configured duration and last visual issue.
The camera debug overlay shows the same message with wrapped, readable text.
READY after stopped feedback shows the physics reason, such as Stop command,
RC expiry, controller loss or Reset. A fault remains latched until a new episode.

Machine reason codes remain unchanged. `app/drone_1/mission` diagnostics retain
`vision_reason`, separate `rejected_azimuth_rad` / `rejected_elevation_rad`, angular
limits, physics state/reason and acquisition timeout. Rejected angles never set
`bearing_valid`. Console/display angles use degrees; library diagnostic angles
remain radians. The focused `tracking/front/los` schema and valid-bearing/null
semantics are unchanged.

## Yaw target search

After settled hover, the app checks the next fresh target frame, waiting at most
100 ms. A usable target enters centering immediately. Otherwise `YAW_SEARCH` turns at
**15 deg/s positive body yaw** (counterclockwise from above), with level roll/pitch
and the selected altitude held. The default scan takes about 24 s. It uses
measured attitude to unwrap a net full circle, including across +/-180 degrees;
command time alone cannot claim a complete scan. No XY truth is used.

A fresh, current-episode, unclipped target with timestamp-aligned attitude
and bearing within the existing +/-35-degree azimuth and +/-60-degree elevation
limits switches the scan to `YAW_CENTER`. Edge/clipped/stale/out-of-envelope
detections do not stop the scan. Yaw must bring the bbox center within **+/-10
pixels of image width/2** before search can finish; a visible off-center target
never starts LOS. `YAW_SETTLE` commands zero yaw rate and requires the target
to remain in that band, yaw speed <=5 deg/s and the existing altitude/vertical-
speed/level-attitude tolerances continuously for **0.3 simulation seconds**.
Drift outside the band resumes centering and resets the dwell. Losing the
candidate resumes the remaining scan without resetting its angle or timer.

After settling, the app saves the current heading and transfers the existing
filter/controller state to LOS. Only images captured after settling can enter
LOS acquisition, which still needs its normal dwell before approach. The drone
does not return to its original heading, and yaw continues to hold the new
heading during approach rather than center the bbox. A full circle without a
usable target latches `search_exhausted`; **34 simulation seconds**, including
centering and settling interruptions, latches `search_timeout` if the scan cannot finish.
Both withdraw active RC and freeze the episode until a new Start/Reset.

Mission diagnostics include `search_swept_deg`, `search_remaining_deg`,
`search_commanded_yaw_rate_deg_s`, `search_measured_yaw_rate_deg_s`,
`search_elapsed_s` and the visual rejection cause. Console/debug status shows
search/settling and degrees; focused LOS telemetry remains inactive until
acquisition. No new port or message version is introduced. Custom profiles must
allow at least the 15 deg/s search rate; the app rejects incompatible limits.

### Horizontal centering before acquisition

The centering error is `bbox.x + bbox.width / 2 - image.width / 2`. At the
default 640-pixel width the goal is X=320, within +/-10 pixels. Only horizontal
centering is performed; altitude hold and level roll/pitch continue.

`YAW_CENTER` uses `yaw_rate = -1.5 * atan(error_x / camera.fx_px)`, bounded to
+/-15 deg/s. A target on the right requests negative yaw, a target on the left
requests positive yaw, and yaw slows as the target nears the center. Inside the
band the yaw-rate reference is zero. Overshoot reverses the correction. Gain
and tolerance are named constants in `bullet_app/search.py` for later tuning.

Once centered and settled, save the new heading and run normal LOS acquisition
using new post-settle images. Yaw centering does not continue during approach.
The existing one-circle/34 s bounds and watchdogs remain active. Mission
diagnostics add `target_center_x_px`, `image_center_x_px`, `target_error_x_px`,
`target_center_tolerance_px` and `target_centered`; the commanded search yaw
rate now reports the actual centering reference as well as the blind-scan rate.

### Sensor-derived attitude feedback

The flight config accepts both `attitude_model=ideal` and `imu_complementary`.
The app consumes the supplied quaternion through the existing attitude path; it
does not run another filter or add noise. Configure sensor noise, filter gains
and pose sources in the physics runner's YAML, passed with `--config`. See
[sensor pose configuration](../bullet_drone/docs/design/sensors.md#sensor-pose-selection-and-attitude-estimation).

### Noise-tolerant hover confirmation

Hover and centered-yaw settling use a one-second linear trend of fresh barometer
altitude readings for their vertical-motion check. The trend must remain within
+/-0.2 m/s, alongside the existing +/-0.25 m estimated-altitude and +/-5 degree
attitude/heading limits. Hover still requires three continuous simulation seconds;
centered-yaw settling still requires 0.3 seconds and its existing yaw-rate limit.
An unavailable trend prevents confirmation; any failed condition resets the dwell.

The history is bounded to 241 readings and retains the boundary sample covering
one second. It uses source simulation timestamps, requires at least three samples
spanning the full window, ignores duplicate feedback and restarts on missing
barometer readings, nonincreasing simulation time or gaps exceeding the barometer
freshness limit. Accepted source/episode changes clear it with the mission state.
Foreign feedback is still ignored. Repeating a control tick cannot manufacture
new measurements or advance simulation-time dwell.

Mission hold diagnostics expose `settling_vertical_speed_m_s` (null until ready)
and `settling_window_s`. The existing top-level `vertical_speed_m_s` remains the
controller's instantaneous estimate. The controller and LOS handoff continue to
use that original estimate; this change affects transition decisions only.

The trend window is a fixed app constant, not an additional YAML setting. It
estimates average motion and introduces up to about one second of response lag;
it does not guarantee instantaneous stability. Regression checks include real
physics with the current noisy/filtered YAML and a synthetic camera, plus motion,
excursion, reset and duplicate/missing-sample cases. Hardware and full renderer
flight validation were not performed.

### Betaflight hover-only check

Launch `bullet-app` with its usual mission config plus `--hover-check` to run the
same Betaflight MSP altitude loop, climb 20 m above the starting barometer datum,
and hold for 20 continuous seconds after the normal stability checks pass. The
tuned gains and SITL freshness limits are in the
[dedicated hover profile](configs/betaflight_hover_profile.yaml). The check allows
75 seconds to reach stable hover; any excursion restarts the timer. At completion,
the app withdraws RC and the simulation freezes at altitude; it does not land.
MCAP ground truth can be checked with
`bullet_render/tools/check_app_flight.py --physics-config ../config/basic.yaml
--hover-check`.

## Recorded flight diagnostics

The standard workspace launcher records MCAP and separate process logs automatically.
Mission diagnostics now identify the visual input actually processed by navigation
(stream, frame sequence, capture time, bbox), signed left/top/right/bottom edge
clearances in pixels, and touched edges. The existing `clipped_target` policy is
unchanged. Latest detected-frame metadata is separate from processed-frame identity;
its end-to-end vision processing latency uses the shared host monotonic clock.
App configuration snapshots repeat on `analysis/app/config` with source identity,
resolved camera/vehicle/tuning values and source revisions. Worker exceptions retain
tracebacks in the app process log. Offline MCAP reports live in the sibling
`mcap_bagger` package; ground-truth analysis topics never enter mission feedback.

Mission diagnostics expose optional `takeoff_goal_altitude_m` and
`barometer_datum_m` snapshots for FlightLens. These are the initialized controller
reference and sensor datum, not a new control input. Older recordings lacking
these fields remain readable and show the takeoff goal as unavailable.

## MCAP output directory

The standard **Bullet: Run all** launcher reads
[`configs/app.json`](configs/app.json):

```json
{
  "recording": {
    "output_dir": "../../recordings"
  }
}
```

`recording.output_dir` is the directory for **future MCAP files**. It must be a
nonempty path string. Absolute paths are accepted; relative paths resolve beside
`app.json`, regardless of the launch working directory. `~` expands to the user's
home directory. The supplied value saves MCAPs in the shared workspace recordings directory.
Malformed JSON, unknown/missing fields and invalid values stop launch.

For one run, from `bullet_render`, override the directory:

```bash
uv run --locked --all-packages --all-extras --all-groups python tools/run_flight.py \
  --mcap-output-dir /data/flights
```

Use `--app-config /path/to/app.json` to select another configuration file. The
CLI directory takes precedence and resolves relative paths from the launch
working directory. These options belong to the launcher, which supervises the
recorder; starting `bullet-app` alone continues without starting a recorder.

The recorder creates a missing directory and tests a temporary write before
signalling readiness. A path/write failure stops launch before Godot, physics
and the flight app start. The effective directory is printed at startup.
Automatic filenames and one MCAP per flight are preserved. Existing recordings
are not moved, configuration files are not rewritten, and changing the setting
requires a new launch. Process logs stay in
`bullet_render/recordings/launch_<UTC>/`.

### Betaflight sensor and MSP diagnostics

The Betaflight app uses the simulation feedback IMU attitude and barometer
altitude. MSP polls status at 10 Hz and streams RC commands; it does not poll
attitude or altitude. `analysis/app/msp.sensor_feedback` reports expected and
observed feedback rates, sensor ages, image age, and image-to-attitude alignment.
`analysis/app/msp.msp_delivery` reports MSP request and response rates and
transport timing. The console summary prints once per second; MCAP snapshots
remain at 20 Hz.

`image_attitude_offset_s` is latest image capture time minus the attitude sample
time used by the mission; a positive value means the image is newer.
`vision_reason` and `mission_phase`
show the associated alignment or acquisition state. Rates describe trailing
one-second windows and ramp up during the first second.
