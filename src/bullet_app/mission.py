"""Sensor-only, deterministic takeoff/settle/LOS mission; sockets live in runtime."""
from dataclasses import asdict
from copy import deepcopy
from collections import deque
from bullet_msgs.scenario import validate_end
from bullet_msgs.spawn import DEFAULT_TAKEOFF_HEIGHT_M
from math import isclose, radians
from uuid import uuid4

from bullet_msgs.flight import validate_config, validate_feedback, COMMAND_LIFETIME_US
from los_tracking import (CameraCalibration, AltitudeHeadingHold, AttitudeSample, BarometerSample,
                          LosNavigator, NavigationInput, RcChannels, State, TargetObservation)
from los_tracking.geometry import euler, wrap, world_bearing, box_edges
from .search import (YawSearch, SEARCH_RATE_RAD_S, SEARCH_TIMEOUT_S, YAW_SETTLE_DURATION_S,
                     YAW_SETTLE_RATE_RAD_S, TARGET_CENTER_TOLERANCE_PX, centering_command, ANGLE_EPSILON_RAD)


TAKEOFF_HEIGHT_M = DEFAULT_TAKEOFF_HEIGHT_M
HOVER_DURATION_S = 3.0
SETTLING_WINDOW_S = 1.0
SETTLING_MIN_SAMPLES = 3
# One second at the supported 240 Hz physics rate, including the boundary sample.
SETTLING_HISTORY_CAPACITY = 241
TIME_EPSILON_S = 1e-9
TARGET_CHECK_DURATION_S = 0.1
TAKEOFF_TIMEOUT_S = 60.0
ALTITUDE_TOLERANCE_M = 0.25
VERTICAL_SPEED_TOLERANCE_M_S = 0.2
ATTITUDE_TOLERANCE_RAD = radians(5)
FEEDBACK_TIMEOUT_S = 0.1
COMMAND_PERIOD_S = 0.01
# Leave one tick for scheduling jitter and transport after the next control tick.
COMMAND_HORIZON_S = 2 * COMMAND_PERIOD_S
DISARMED_AETR = RcChannels(1500, 1500, 1000, 1500)


class Mission:
    def __init__(self, config, *, app_id=None):
        if config.tuning.yaw_rate_limit_rad_s < SEARCH_RATE_RAD_S:
            raise ValueError('yaw search requires a yaw limit of at least 15 deg/s')
        self.config = config
        self.app_id = app_id or uuid4().hex
        self.platform = self.feedback = None
        self.sequence = 0
        self.identity = None
        self.phase, self.reason = 'READY', 'waiting_platform'
        self.diagnostics = {}
        self._reset()

    def _reset(self):
        self.diagnostics = {}
        self.terminal_result = None
        self.hold = AltitudeHeadingHold(self.config)
        self.navigator = LosNavigator(self.config)
        self.datum = self.goal = None
        self.takeoff_height_m = TAKEOFF_HEIGHT_M
        self.started_sim = self.settled_sim = self.hover_completed_sim = self.hover_completed_us = None
        self.last_step = -1
        self.last_input = NavigationInput()
        self.nav_result = None
        self.search = YawSearch()
        self.los_started_us = None
        self.arm_sequence = None
        self.running_seen = False
        self.relative_altitude = self.vertical_speed = None
        self._settling_altitudes = deque(maxlen=SETTLING_HISTORY_CAPACITY)

    def configure(self, platform):
        validate_config(platform)
        c, v = self.config.camera, self.config.vehicle
        expected = dict(mass_kg=v.mass_kg, gravity_m_s2=v.gravity_m_s2, max_thrust_n=v.max_thrust_n,
                        rc_angle_scale_rad=v.rc_angle_scale_rad, rc_yaw_rate_scale_rad_s=v.rc_yaw_rate_scale_rad_s,
                        physics_hz=240, feedback_hz=120, command_hz=100)
        if any(not isclose(platform[k], value, rel_tol=1e-6) for k, value in expected.items()):
            raise ValueError('platform/control profile mismatch')
        if platform['aetr_channels'] != [1, 2, 3, 4] or (platform['arm_channel'], platform['mode_channel']) != (5, 6):
            raise ValueError('unsupported RC mapping')
        camera = platform['camera']
        advertised = CameraCalibration.from_horizontal_fov(camera['width_px'], camera['height_px'], radians(camera['fov_x_deg']))
        if any(not isclose(getattr(c, k), getattr(advertised, k), rel_tol=1e-8) for k in ('width_px', 'height_px', 'fx_px', 'fy_px', 'cx_px', 'cy_px')) or c.camera_to_body_xyzw != advertised.camera_to_body_xyzw:
            raise ValueError('camera calibration mismatch')
        self.platform = platform

    def observe(self, feedback):
        validate_feedback(feedback)
        if self.platform is None:
            return
        source = feedback['source']
        if source['physics_id'] != self.platform['physics_id'] or source['stream_id'] != self.platform['stream_id']:
            return
        if self.feedback is not None and source['measurement_us'] < self.feedback['source']['measurement_us']:
            return
        identity = (source['physics_id'], source['episode_id'], source['stream_id'])
        if identity != self.identity:
            self._reset()
            self.identity = identity
            self.phase, self.reason = 'READY', 'ready'
            self.takeoff_height_m = feedback.get('takeoff_height_m', TAKEOFF_HEIGHT_M)
        if source['step_index'] < self.last_step:
            return
        # State can change while the frozen source sample remains identical.
        self.feedback = feedback
        if source['step_index'] == self.last_step:
            return
        self.last_step = source['step_index']
        timestamp = source['measurement_us'] / 1e6
        attitude = AttitudeSample(source['stream_id'], source['step_index'], timestamp, tuple(feedback['orientation_xyzw']))
        barometer = (None if feedback['barometer_altitude_m'] is None else
                     BarometerSample(source['stream_id'], source['step_index'], timestamp, feedback['barometer_altitude_m']))
        self.last_input = NavigationInput(attitude=attitude, barometer=barometer)
        self.hold.observe(self.last_input)
        samples = self._settling_altitudes
        sim = source['simulation_time_s']
        if barometer is None:
            samples.clear()
        else:
            if samples and not 0 < sim - samples[-1][0] <= self.config.tuning.max_barometer_age_s:
                samples.clear()
            samples.append((sim, barometer.altitude_m))
            while len(samples) > 2 and samples[1][0] <= sim - SETTLING_WINDOW_S:
                samples.popleft()
        self.search.observe(euler(attitude.orientation_xyzw)[2], source["simulation_time_s"])
        if self.datum is None and barometer is not None:
            self.datum = barometer.altitude_m
            self.goal = self.datum + self.takeoff_height_m

    def complete(self, result):
        validate_end(result)
        source = result['final_source']
        if (source['physics_id'], source['episode_id'], source['stream_id']) != self.identity:
            return False
        if self.terminal_result is None:
            self.terminal_result = deepcopy(result)
        self.phase, self.reason = 'COMPLETED', 'target_hit'
        return True

    def fail(self, reason):
        if self.phase != 'COMPLETED':
            self.phase, self.reason = 'FAULT', reason

    def tick(self, *, now_s, detection=None):
        if self.platform is None or self.feedback is None:
            return None
        f, source = self.feedback, self.feedback['source']
        now_us = int(now_s * 1e6)
        active = False
        armed = False
        command = DISARMED_AETR
        lifetime_us = COMMAND_LIFETIME_US
        if self.phase == 'COMPLETED' or (f['state'] == 'stopped' and f['reason'] == 'target_hit'):
            self.phase, self.reason = 'COMPLETED', 'target_hit'
        elif f['state'] == 'running':
            if not self.running_seen:
                self.running_seen = True
                self.phase, self.reason = 'SENSOR_WARMUP', 'warming_sensors'
                self.started_sim = source['simulation_time_s']
            if f['owner_id'] != self.app_id:
                self.fail('foreign_owner')
            elif not 0 <= now_s - source['measurement_us'] / 1e6 <= FEEDBACK_TIMEOUT_S:
                self.fail('feedback_stale')
            if self.phase != 'FAULT':
                active = True
                sim = source['simulation_time_s']
                if self.phase == 'SENSOR_WARMUP' and self.hold.ready(now_s):
                    self.phase, self.reason = 'ARMING', 'waiting_applied_arm'
                    self.arm_sequence = self.sequence + 1
                if self.phase == 'ARMING':
                    armed = True
                    if f['armed'] and f['mode'] == 'angle' and f['last_applied_sequence'] >= self.arm_sequence:
                        self.phase, self.reason = 'TAKEOFF', 'climbing'
                if self.phase in ('TAKEOFF', 'HOVER', 'WAIT_TARGET', 'YAW_SEARCH', 'YAW_CENTER', 'YAW_SETTLE', 'LOS_ACQUIRE', 'LOS_APPROACH'):
                    armed = True
                    if not self.hold.ready(now_s):
                        self.fail('sensors_stale')
                    elif self.phase.startswith('LOS_'):
                        target = self._target(detection, now_s)
                        result = self.navigator.update(NavigationInput(attitude=self.last_input.attitude,
                            barometer=self.last_input.barometer, target=target), now_s=now_s, enabled=True,
                            command_horizon_s=COMMAND_HORIZON_S)
                        self.nav_result = result
                        self.diagnostics = asdict(result.diagnostics)
                        if result.diagnostics.altitude_m is not None:
                            self.relative_altitude = result.diagnostics.altitude_m - self.datum
                        self.vertical_speed = result.diagnostics.vertical_speed_m_s
                        if not result.usable or result.state == State.FAULT:
                            self.fail('los_' + result.reason)
                        else:
                            command = result.command
                            lifetime_us = min(lifetime_us, int(result.valid_until_s * 1e6) - now_us)
                            self.phase = 'LOS_APPROACH' if result.state == State.TRACK else 'LOS_ACQUIRE'
                            self.reason = result.reason
                    else:
                        command = self._hold_tick(now_s, detection)
                if self.phase in ('SENSOR_WARMUP', 'ARMING') and sim - self.started_sim >= TAKEOFF_TIMEOUT_S:
                    self.fail('warmup_or_arm_timeout')
        else:
            if self.phase != 'FAULT':
                self.phase, self.reason = 'READY', f['reason']
        if self.phase in ('FAULT', 'COMPLETED'):
            active, armed, command = False, False, DISARMED_AETR
        if lifetime_us <= 0:
            self.fail('command_expired')
            active, armed, command, lifetime_us = False, False, DISARMED_AETR, COMMAND_LIFETIME_US
        channels = [*command.as_tuple(), 2000 if armed else 1000, 2000, *([1000] * 12)]
        self.sequence += 1
        return dict(schema_version=1, physics_id=source['physics_id'], episode_id=source['episode_id'], app_id=self.app_id,
                    sequence=self.sequence, issued_us=now_us, valid_until_us=now_us + lifetime_us, active=active, channels=channels)

    def _settling_motion(self):
        samples = self._settling_altitudes
        span = samples[-1][0] - samples[0][0] if samples else 0.0
        if len(samples) < SETTLING_MIN_SAMPLES or span < SETTLING_WINDOW_S - TIME_EPSILON_S:
            return None, span
        origin = samples[0][0]
        mean_time = sum(time - origin for time, _ in samples) / len(samples)
        mean_altitude = sum(altitude for _, altitude in samples) / len(samples)
        covariance = sum((time - origin - mean_time) * (altitude - mean_altitude)
                         for time, altitude in samples)
        variance = sum((time - origin - mean_time) ** 2 for time, _ in samples)
        return covariance / variance, span

    def _hold_tick(self, now_s, detection):
        sim = self.feedback['source']['simulation_time_s']
        now_us = int(now_s * 1e6)
        altitude, vz = self.hold.estimate(now_s)
        self.relative_altitude, self.vertical_speed = altitude - self.datum, vz
        settling_vz, settling_window = self._settling_motion()
        roll, pitch, yaw = euler(self.last_input.attitude.orientation_xyzw)
        level = (abs(altitude - self.goal) <= ALTITUDE_TOLERANCE_M
                 and settling_vz is not None and abs(settling_vz) <= VERTICAL_SPEED_TOLERANCE_M_S
                 and max(abs(roll), abs(pitch)) <= ATTITUDE_TOLERANCE_RAD)
        if self.phase in ('TAKEOFF', 'HOVER'):
            settled = level and abs(wrap(yaw - self.hold.heading_rad)) <= ATTITUDE_TOLERANCE_RAD
            if not settled:
                self.settled_sim = None
                self.phase, self.reason = 'TAKEOFF', 'climbing_or_settling'
            else:
                self.phase, self.reason = 'HOVER', 'settling'
                if self.settled_sim is None:
                    self.settled_sim = sim
                if sim - self.settled_sim >= HOVER_DURATION_S:
                    self.phase, self.reason = 'WAIT_TARGET', 'waiting_post_hover_bbox'
                    self.hover_completed_sim, self.hover_completed_us = sim, now_us
            if sim - self.started_sim >= TAKEOFF_TIMEOUT_S:
                self.fail('takeoff_timeout')

        begin_los = False
        vision = {}
        yaw_reference = None
        if self.phase in ('WAIT_TARGET', 'YAW_SEARCH', 'YAW_CENTER', 'YAW_SETTLE'):
            candidate, vision = self._search_target(detection, now_s)
            vision.update(target_center_x_px=None, image_center_x_px=self.config.camera.width_px / 2,
                          target_error_x_px=None, target_center_tolerance_px=TARGET_CENTER_TOLERANCE_PX,
                          target_centered=False)
            if self.phase == 'WAIT_TARGET':
                if candidate is not None:
                    self.search.start(sim)
                    self.phase, self.reason = 'YAW_CENTER', 'centering_target'
                elif self._target(detection, now_s) is not None or sim - self.hover_completed_sim >= TARGET_CHECK_DURATION_S:
                    self.search.start(sim)
                    self.phase, self.reason = 'YAW_SEARCH', 'searching_target'
            if self.phase == 'YAW_SEARCH':
                if candidate is not None:
                    self.search.settled_sim = None
                    self.phase, self.reason = 'YAW_CENTER', 'centering_target'
                elif self.search.exhausted:
                    self.fail('search_exhausted')
            if self.phase in ('YAW_CENTER', 'YAW_SETTLE'):
                if candidate is None:
                    self.search.start(sim)  # Resume; retain the original angle and deadline.
                    self.phase, self.reason = 'YAW_SEARCH', 'searching_target'
                    if self.search.exhausted:
                        self.fail('search_exhausted')
                else:
                    error_px, yaw_reference = centering_command(candidate.bbox, self.config.camera)
                    centered = abs(error_px) <= TARGET_CENTER_TOLERANCE_PX
                    vision.update(target_center_x_px=candidate.bbox[0] + candidate.bbox[2] / 2,
                                  target_error_x_px=error_px, target_centered=centered)
                    if not centered:
                        self.search.settled_sim = None
                        self.phase, self.reason = 'YAW_CENTER', 'centering_target'
                    else:
                        self.phase, self.reason = 'YAW_SETTLE', 'settling_centered_target'
                        settled = level and abs(self.search.yaw_rate_rad_s) <= YAW_SETTLE_RATE_RAD_S
                        if not settled:
                            self.search.settled_sim = None
                        elif self.search.settled_sim is None:
                            self.search.settled_sim = sim
                        elif sim - self.search.settled_sim >= YAW_SETTLE_DURATION_S:
                            self.hold.set_heading(yaw)
                            begin_los = True
            if self.search.started_sim is not None and self.search.elapsed(sim) >= SEARCH_TIMEOUT_S:
                self.fail('search_timeout')
                begin_los = False
            if self.phase == 'YAW_SEARCH':
                yaw_reference = SEARCH_RATE_RAD_S
            elif self.phase == 'FAULT':
                yaw_reference = None

        command = self.hold.command(now_s=now_s, target_altitude_m=self.goal,
                                   yaw_rate_rad_s=yaw_reference)
        self.diagnostics = {**self.hold.control.last_diagnostics, **vision,
                            'settling_vertical_speed_m_s': settling_vz,
                            'settling_window_s': settling_window}
        if self.search.started_sim is not None:
            self.diagnostics.update(self.search.diagnostics(sim, rotating=self.phase == 'YAW_SEARCH',
                                                            commanded_yaw_rate_rad_s=yaw_reference or 0.))
        if begin_los and self.phase != 'FAULT':
            self.navigator.reset(handoff=self.hold.handoff(now_s=now_s, target_altitude_m=self.goal))
            self.los_started_us = now_us
            self.phase, self.reason = 'LOS_ACQUIRE', 'acquiring'
        return command

    def _search_target(self, detection, now_s):
        """Prequalify geometry only; LOS still owns its acquisition/filter dwell."""
        t, camera = self.config.tuning, self.config.camera
        diagnostics = dict(vision_reason='not_observed', max_azimuth_rad=t.max_azimuth_rad,
                           max_elevation_rad=t.max_elevation_rad,
                           rejected_azimuth_rad=None, rejected_elevation_rad=None)
        target = self._target(detection, now_s)
        if target is None:
            diagnostics['vision_reason'] = 'stale_or_foreign_image' if detection is not None else 'not_observed'
            return None, diagnostics
        if target.bbox is None:
            diagnostics['vision_reason'] = 'no_target'
            return None, diagnostics
        margins, edges = box_edges(target.bbox, camera)
        diagnostics.update(processed_image_stream_id=target.stream_id,
                           processed_image_sequence=target.sequence, processed_image_timestamp_s=target.timestamp_s,
                           processed_bbox=target.bbox, edge_margins_px=margins, touched_edges=edges)
        x, y, width, height = target.bbox
        if x <= 0 or y <= 0 or x + width >= camera.width_px or y + height >= camera.height_px:
            diagnostics['vision_reason'] = 'clipped_target'
            return None, diagnostics
        attitude = self.hold.attitude_at(target.timestamp_s)
        if attitude is None:
            diagnostics['vision_reason'] = 'attitude_unaligned'
            return None, diagnostics
        heading = self.hold.heading_rad if self.phase == 'WAIT_TARGET' else self.search.yaw_rad
        elevation, azimuth = world_bearing(target.bbox, camera, attitude, heading)
        if abs(elevation) > t.max_elevation_rad + ANGLE_EPSILON_RAD or abs(azimuth) > t.max_azimuth_rad + ANGLE_EPSILON_RAD:
            diagnostics.update(vision_reason='bearing_outside_envelope',
                               rejected_elevation_rad=elevation, rejected_azimuth_rad=azimuth)
            return None, diagnostics
        diagnostics['vision_reason'] = 'usable'
        return target, diagnostics

    def _target(self, detection, now_s):
        if detection is None or self.hover_completed_us is None:
            return None
        metadata, bbox = detection
        source = metadata.get('source')
        if (source is None or (source['physics_id'], source['episode_id'], source['stream_id']) != self.identity
                or source['measurement_us'] <= (self.los_started_us if self.los_started_us is not None else self.hover_completed_us)
                or not 0 <= now_s - source['measurement_us'] / 1e6 <= self.config.tuning.max_image_age_s):
            return None
        return TargetObservation(source['stream_id'], metadata['sequence'], source['measurement_us'] / 1e6, bbox)

    def status(self, now_s):
        if self.feedback is None:
            return None
        s = self.feedback['source']
        sim = s['simulation_time_s']
        return dict(schema_version=1, physics_id=s['physics_id'], episode_id=s['episode_id'], app_id=self.app_id,
                    sequence=self.sequence, measurement_us=int(now_s * 1e6), simulation_time_s=sim,
                    phase=self.phase, reason=self.reason, relative_altitude_m=self.relative_altitude,
                    vertical_speed_m_s=self.vertical_speed,
                    hover_remaining_s=max(0., HOVER_DURATION_S - (0 if self.settled_sim is None else sim - self.settled_sim)),
                    target_remaining_s=(max(0., TARGET_CHECK_DURATION_S - (sim - self.hover_completed_sim))
                                        if self.phase == 'WAIT_TARGET' else 0.),
                    navigator_state=None if self.nav_result is None else self.nav_result.state.value,
                    diagnostics={**self.diagnostics, 'takeoff_goal_altitude_m': self.goal,
                                 'barometer_datum_m': self.datum, 'physics_state': self.feedback['state'],
                                 'physics_reason': self.feedback['reason'],
                                 'acquisition_timeout_s': self.config.tuning.acquisition_timeout_s})
