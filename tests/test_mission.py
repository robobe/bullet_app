"""Mission transitions and a real sensor-only PyBullet takeoff."""
from pathlib import Path
from math import radians

import pytest
from bullet_app.mission import Mission
from los_tracking import NavigatorConfig


ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'los_tracking/examples/bullet_profile.json'


def platform(config):
    v, c = config.vehicle, config.camera
    return dict(schema_version=1, physics_id='physics', stream_id='stream',
                mass_kg=v.mass_kg, gravity_m_s2=v.gravity_m_s2, max_thrust_n=v.max_thrust_n,
                rc_angle_scale_rad=v.rc_angle_scale_rad, rc_yaw_rate_scale_rad_s=v.rc_yaw_rate_scale_rad_s,
                aetr_channels=[1, 2, 3, 4], arm_channel=5, mode_channel=6,
                camera=dict(width_px=c.width_px, height_px=c.height_px, fov_x_deg=90.),
                attitude_model='ideal', physics_hz=240, feedback_hz=120, command_hz=100)


def feedback(step, now, altitude, *, state='running', episode='episode', armed=False, applied=0):
    return dict(schema_version=1, source=dict(physics_id='physics', episode_id=episode, stream_id='stream',
                step_index=step, simulation_time_s=step / 240, measurement_us=int(now * 1e6)),
                barometer_altitude_m=altitude, orientation_xyzw=[0, 0, 0, 1], armed=armed, mode='angle',
                last_applied_sequence=applied, owner_id='app' if state == 'running' else None,
                state=state, reason='test')


def mission():
    config = NavigatorConfig.from_json(PROFILE)
    m = Mission(config, app_id='app')
    m.configure(platform(config))
    return m


def test_ready_warmup_arm_confirmation_and_stale_freeze():
    m = mission()
    m.observe(feedback(0, 1000., 4., state='stopped'))
    assert not m.tick(now_s=1000.)['active']
    for step in range(1, 40):
        now = 1000 + step / 240
        m.observe(feedback(step, now, 4.))
        rc = m.tick(now_s=now)
        assert rc['channels'][2] == 1000
    assert m.phase == 'ARMING'
    assert rc['channels'][4:6] == [2000, 2000]
    assert m.goal == 24.
    m.observe(feedback(40, 1000 + 40 / 240, 4., armed=True, applied=rc['sequence']))
    assert m.tick(now_s=1000 + 40 / 240)['channels'][2] > 1000
    assert m.phase == 'TAKEOFF'
    rc = m.tick(now_s=1001.)
    assert not rc['active'] and rc['channels'][4] == 1000
    assert m.reason == 'feedback_stale'


def test_profile_mismatch_refuses_readiness():
    m = mission()
    config = platform(m.config)
    config['mass_kg'] = 2
    with pytest.raises(ValueError, match='mismatch'):
        m.configure(config)


@pytest.mark.parametrize('target_found', [False, True])
@pytest.mark.parametrize('noisy_profile', [False, True], ids=['ideal', 'noisy_filtered'])
def test_sensor_only_physics_takeoff_hover_yaw_search(target_found, noisy_profile):
    from dataclasses import replace
    from bullet_drone.config import DroneSettings, SimulationConfig, SimulationSettings
    from bullet_drone.config import YamlConfigLoader
    from bullet_drone.commands import RcInput
    from bullet_drone.controllers import InternalControlSettings, PidSettings
    from bullet_drone.simulation import RcSimulation
    from bullet_drone.pose import resolve_pose
    m = mission()
    inner = InternalControlSettings((4., 4., 4.), (2., 2., 2.),
                                    (PidSettings(kp=.05, ki=0., kd=0., output_limit=.1, integral_limit=0.),) * 3)
    config = SimulationConfig(SimulationSettings(1 / 240, 20000, (0., 0., -9.81), gui=False),
                              DroneSettings(1., (0., 0., .015)))
    if noisy_profile:
        config = YamlConfigLoader().load(ROOT / 'bullet_drone/configs/basic.yaml')
        config = replace(config, simulation=replace(config.simulation, gui=False))
        assert config.sensors.barometer.filter_enabled and config.sensors.barometer.pressure_stddev_pa > 0
        m.configure(dict(platform(m.config), attitude_model='imu_complementary'))
    sim = RcSimulation(config, inner)
    phases = set()
    rc = None
    detection = None
    try:
        obs = sim.reset()
        m.observe(feedback(0, 1000., obs.sensors.barometer.altitude_m))
        for step in range(1, 16000):
            now = 1000 + (step - 1) / 240
            if rc is None or step % 12 in (0, 2, 5, 7, 10):
                rc = m.tick(now_s=now, detection=detection)
                phases.add(m.phase)
            if m.phase == 'FAULT' or m.phase == 'LOS_APPROACH':
                break
            obs = sim.step(RcInput(tuple(rc['channels'])))
            if step % 2 == 0:
                pose = resolve_pose(obs, config.pose_sources)
                # Feedback and camera use the same configured sensor pose as the runner.
                f = feedback(step, 1000 + step / 240, obs.sensors.barometer.altitude_m,
                             armed=rc['channels'][4] >= 1500, applied=rc['sequence'])
                f['orientation_xyzw'] = list(pose.orientation_xyzw)
                m.observe(f)
                if target_found:
                    # Test scene projects a fixed box behind the spawn. Only bbox and
                    # sensor packets enter the mission; truth is used by this camera fixture.
                    from los_tracking.geometry import rotate
                    q = pose.orientation_xyzw
                    position = pose.position_m
                    bx, by, bz = rotate((-q[0], -q[1], -q[2], q[3]),
                                        (-50. - position[0], -position[1], 2. - position[2]))
                    bbox = None
                    if bx > 0:
                        cx, cy = 320 - 320 * by / bx, 240 - 320 * bz / bx
                        half = 480 / bx
                        left, top = max(0, round(cx - half)), max(0, round(cy - half))
                        right, bottom = min(640, round(cx + half)), min(480, round(cy + half))
                        if right > left and bottom > top:
                            bbox = (left, top, right - left, bottom - top)
                    detection = (dict(sequence=step, source=dict(f['source'])), bbox)
        assert {'SENSOR_WARMUP', 'ARMING', 'TAKEOFF', 'HOVER', 'WAIT_TARGET', 'YAW_SEARCH'} <= phases, (m.phase, m.reason)
        if target_found:
            assert {'YAW_SETTLE', 'LOS_ACQUIRE', 'LOS_APPROACH'} <= phases, (m.phase, m.reason)
            from math import degrees
            assert abs(abs(degrees(m.hold.heading_rad)) - 180) < 5
            assert m.nav_result.diagnostics.bearing_valid
            assert abs(rc['channels'][3] - 1500) < 5
        else:
            assert m.reason == 'search_exhausted', (m.phase, m.reason, m.relative_altitude, m.vertical_speed)
            assert m.search.swept_rad >= radians(360)
            assert m.search.elapsed(obs.ground_truth.simulation_time_s) < 34.
        assert abs(m.relative_altitude - 20.) < .25
        assert obs.ground_truth.simulation_time_s < 60
        assert max(abs(v) for v in obs.ground_truth.position_m[:2]) < .1
    finally:
        sim.close()


def test_hover_reset_post_hover_identity_and_los_handoff():
    m = mission()
    rc = None
    hover_seen = False
    reset_seen = False
    old_detection = None
    for step in range(1800):
        now = 1000 + step / 240
        altitude = 0. if step < 50 else 20.
        # Once settled hover begins, disturb altitude and require a fresh dwell.
        if hover_seen and not reset_seen:
            altitude = 21.
        f = feedback(step, now, altitude, armed=rc is not None and rc['channels'][4] == 2000,
                     applied=0 if rc is None else rc['sequence'])
        m.observe(f)
        rc = m.tick(now_s=now, detection=old_detection)
        if m.phase == 'HOVER' and not hover_seen:
            hover_seen = True
            old_detection = (dict(sequence=1, source=dict(f['source'])), (300, 260, 30, 25))
        if hover_seen and m.phase == 'TAKEOFF':
            reset_seen = True
            assert m.settled_sim is None
        if m.phase == 'WAIT_TARGET':
            break
    assert hover_seen and reset_seen and m.phase == 'WAIT_TARGET'
    assert m._target(old_detection, now) is None
    source = dict(f['source'], measurement_us=int((now + .01) * 1e6), episode_id='foreign')
    assert m._target((dict(source=source, sequence=2), (300, 260, 30, 25)), now + .01) is None
    prior_throttle = rc['channels'][2]
    for offset in range(1, 180):
        step += 1
        now = 1000 + step / 240
        f = feedback(step, now, 20., armed=True, applied=rc['sequence'])
        m.observe(f)
        detection = (dict(sequence=offset + 2, source=dict(f['source'])), (300, 260, 30, 25))
        rc = m.tick(now_s=now, detection=detection)
        assert rc['active'], (m.phase, m.reason)
        if m.phase == 'LOS_ACQUIRE':
            assert abs(rc['channels'][2] - prior_throttle) <= 5
        prior_throttle = rc['channels'][2]
        if m.phase == 'LOS_APPROACH':
            break
    assert m.phase == 'LOS_APPROACH'
    assert m.nav_result.diagnostics.bearing_valid
    # Keep sensors live while an image approaches its deadline between command ticks.
    recovered = False
    for _ in range(23):
        step += 1
        now = 1000 + step / 240
        m.observe(feedback(step, now, 20., armed=True, applied=rc['sequence']))
        rc = m.tick(now_s=now, detection=detection)
        assert rc['active'] and m.phase != 'FAULT'
        if m.reason == 'image_deadline':
            recovered = True
            assert m.phase == 'LOS_ACQUIRE'
            assert rc['valid_until_us'] - rc['issued_us'] >= 20_000
    assert recovered
    # Restart feedback gets fresh state and cannot reuse previous filters/arming.
    m.observe(feedback(0, now + .01, 4., state='stopped', episode='fresh'))
    rc = m.tick(now_s=now + .01)
    assert m.phase == 'READY' and not rc['active']
    assert m.goal == 24. and m.arm_sequence is None
    m.observe(feedback(100, 1000., 20.))
    assert m.identity[1] == 'fresh' and m.phase == 'READY'


def test_terminal_result_latches_until_fresh_episode():
    m = mission()
    f = feedback(0, 1000., 4.)
    m.observe(f)
    rc = m.tick(now_s=1000.)
    report = dict(schema_version=1, event_id='hit', controller_id='godot', issued_us=1_000_000_000,
                  source=dict(f['source']), target_id='red_truck', drone_position_m=[50., 0., 4.])
    end = dict(schema_version=1, event_id='hit', collision=report, final_source=dict(f['source']),
               outcome='success', reason='target_hit', ended_us=1_000_000_000)
    assert m.complete(end)
    m.fail('feedback_stale')
    # Completion can arrive before stopped feedback, and may outlive all freshness limits.
    rc = m.tick(now_s=1002.)
    assert m.phase == 'COMPLETED' and not rc['active'] and rc['channels'][2:3] == [1000]
    stopped = dict(f, state='stopped', reason='target_hit', owner_id=None)
    m.observe(stopped)
    m.tick(now_s=1003.)
    assert m.phase == 'COMPLETED'
    m.observe(feedback(0, 1004., 4., state='stopped', episode='fresh'))
    assert not m.complete(end)
    m.tick(now_s=1004.)
    assert m.phase == 'READY' and m.terminal_result is None


def test_scenario_takeoff_height_is_latched_per_episode_and_v1_defaults():
    m = mission()
    m.observe(dict(feedback(0, 1000., 30., state='stopped'), schema_version=2, takeoff_height_m=6.5))
    assert m.goal == 36.5 and m.takeoff_height_m == 6.5
    # A duplicate source/state update cannot silently change an episode's goal.
    m.observe(dict(feedback(0, 1000., 30., state='running'), schema_version=2, takeoff_height_m=10))
    assert m.goal == 36.5
    m.observe(dict(feedback(0, 1001., 4., state='stopped', episode='new'), schema_version=2, takeoff_height_m=8))
    assert m.goal == 12 and m.takeoff_height_m == 8
    m.observe(feedback(0, 1002., 7., state='stopped', episode='legacy'))
    assert m.goal == 27 and m.takeoff_height_m == 20


def test_stopped_feedback_retains_cause_and_fault_diagnostics_until_new_episode():
    m = mission()
    packet = feedback(0, 1000., 4., state='stopped')
    packet['reason'] = 'stop'
    m.observe(packet)
    m.tick(now_s=1000.)
    assert m.phase == 'READY' and m.reason == 'stop'
    s = m.status(1000.)
    assert s['diagnostics']['physics_reason'] == 'stop'
    m.diagnostics = {'vision_reason': 'bearing_outside_envelope'}
    m.fail('los_acquisition_timeout')
    packet = dict(feedback(0, 1000., 4., state='stopped'), reason='app_withdrawn')
    m.observe(packet)
    m.tick(now_s=1000.)
    assert m.phase == 'FAULT' and m.reason == 'los_acquisition_timeout'
    assert m.status(1000.)['diagnostics']['vision_reason'] == 'bearing_outside_envelope'
    m.observe(feedback(0, 1001., 4., state='stopped', episode='new'))
    assert m.diagnostics == {} and m.phase == 'READY'


def test_complementary_attitude_feedback_reaches_navigation():
    from bullet_drone.geometry import from_rpy
    config = NavigatorConfig.from_json(PROFILE)
    m = Mission(config, app_id='app')
    m.configure(dict(platform(config), attitude_model='imu_complementary'))
    packet = feedback(0, 1000., 4., state='stopped')
    q = from_rpy((.05, -.03, .2))
    packet['orientation_xyzw'] = list(q)
    m.observe(packet)
    assert m.last_input.attitude.orientation_xyzw == pytest.approx(q)
    assert m.last_input.barometer.altitude_m == 4.
    assert m.last_input.attitude.timestamp_s == 1000.


def test_status_records_explicit_takeoff_goal_and_barometer_datum():
    m=mission()
    reading=feedback(0,1000.,4.,state='stopped')
    reading.update(schema_version=2,takeoff_height_m=6.)
    m.observe(reading)
    d=m.status(1000.)['diagnostics']
    assert d['barometer_datum_m']==4.
    assert d['takeoff_goal_altitude_m']==10.
    assert m.goal==10.  # Recording the reference does not change control state.
