"""Deterministic measured search, qualification, braking and LOS handoff."""
from math import cos, sin, radians, degrees

import pytest
from bullet_app.search import YawSearch
from bullet_app.los_telemetry import snapshot
from test_mission import mission, feedback


def tick(m, step, *, yaw=0., bbox=None, metadata=None, altitude=20., episode='episode'):
    now = 1000 + step / 240
    f = feedback(step, now, altitude, armed=True, applied=m.sequence, episode=episode)
    f['orientation_xyzw'] = [0, 0, sin(radians(yaw) / 2), cos(radians(yaw) / 2)]
    m.observe(f)
    detection = (dict(sequence=step, source=dict(f['source'])) if metadata is None else metadata, bbox)
    return m.tick(now_s=now, detection=detection)


def after_hover():
    m = mission()
    for step in range(2000):
        tick(m, step, altitude=0. if step < 50 else 20.)
        if m.phase == 'WAIT_TARGET':
            return m, step
    pytest.fail((m.phase, m.reason))


def test_unwrap_net_circle_duplicates_reverse_and_simulation_clock():
    s = YawSearch()
    s.observe(radians(170), 100.)
    s.start(100.)
    s.observe(radians(-170), 102.)
    assert degrees(s.swept_rad) == pytest.approx(20.)
    assert degrees(s.yaw_rate_rad_s) == pytest.approx(10.)
    s.observe(radians(-160), 102.)
    assert degrees(s.swept_rad) == pytest.approx(20.)
    s.observe(radians(175), 103.)
    assert degrees(s.swept_rad) == pytest.approx(5.)
    s.start(103.)
    assert s.started_sim == 100. and degrees(s.swept_rad) == pytest.approx(5.)
    for i in range(1, 37):
        s.observe(radians(175 + i * 10), 103 + i)
    assert s.exhausted


def test_visible_target_skips_search_and_search_telemetry_is_inactive():
    m, step = after_hover()
    tick(m, step + 1, bbox=(300, 250, 25, 25))
    assert m.phase == 'YAW_SETTLE'
    assert m.diagnostics['target_centered']
    for step in range(step + 2, step + 140):
        tick(m, step, bbox=(300, 250, 25, 25))
        if m.phase == 'LOS_APPROACH':
            break
    assert m.phase == 'LOS_APPROACH'
    m, step = after_hover()
    rc = tick(m, step + 1)
    assert m.phase == 'YAW_SEARCH' and rc['channels'][3] > 1500
    assert rc['channels'][:2] == [1500, 1500]
    assert not snapshot(m, now_s=1000 + (step + 1) / 240, sequence=1)['active']


@pytest.mark.parametrize('bbox,source_change,reason', [
    ((0, 200, 20, 20), {}, 'clipped_target'),
    ((10, 230, 10, 10), {}, 'bearing_outside_envelope'),
    ((300, 250, 25, 25), {'episode_id': 'foreign'}, 'stale_or_foreign_image'),
    ((300, 250, 25, 25), {'measurement_us': 1}, 'stale_or_foreign_image'),
])
def test_rejected_candidates_keep_scanning(bbox, source_change, reason):
    m, step = after_hover()
    tick(m, step + 1)
    f = feedback(step + 2, 1000 + (step + 2) / 240, 20.)
    metadata = dict(sequence=step + 2, source=dict(f['source'], **source_change))
    rc = tick(m, step + 2, bbox=bbox, metadata=metadata)
    assert m.phase == 'YAW_SEARCH' and rc['channels'][3] > 1500
    assert m.diagnostics['vision_reason'] == reason


def test_unaligned_target_never_stops_scan():
    m, step = after_hover()
    tick(m, step + 1)
    now = 1000 + (step + 2) / 240
    f = feedback(step + 2, now, 20.)
    # Exposure is fresh but later than the newest attitude sample.
    metadata = dict(sequence=step + 2, source=dict(f['source'], measurement_us=int((now + .001) * 1e6)))
    m.observe(f)
    m.tick(now_s=now + .002, detection=(metadata, (300, 250, 25, 25)))
    assert m.phase == 'YAW_SEARCH' and m.diagnostics['vision_reason'] == 'attitude_unaligned'


def test_candidate_loss_resumes_same_sweep_and_heading_handoff_is_continuous():
    m, step = after_hover()
    tick(m, step + 1)
    start = m.search.started_sim
    # Physically plausible 15 deg/s scan until 45 degrees.
    for i in range(1, 721):
        step += 1
        tick(m, step + 1, yaw=i / 16)
    step += 2
    before = m.search.swept_rad
    rc = tick(m, step, yaw=45., bbox=(300, 250, 25, 25))
    assert m.phase == 'YAW_SETTLE' and rc['channels'][3] == 1500
    assert degrees(m.hold.heading_rad) == pytest.approx(0.)  # Save the new heading only after centering/settling.
    assert not snapshot(m, now_s=1000 + step / 240, sequence=1)['active']
    tick(m, step + 1, yaw=45.)
    assert m.phase == 'YAW_SEARCH' and m.search.started_sim == start
    assert m.search.swept_rad == pytest.approx(before)
    prior_throttle = None
    for step in range(step + 2, step + 140):
        rc = tick(m, step, yaw=45., bbox=(300, 250, 25, 25))
        if m.phase == 'LOS_ACQUIRE':
            assert prior_throttle is not None and abs(rc['channels'][2] - prior_throttle) <= 1
            break
        prior_throttle = rc['channels'][2]
    assert m.phase == 'LOS_ACQUIRE'
    assert m.search.started_sim == start
    assert degrees(m.hold.handoff(now_s=1000 + step / 240, target_altitude_m=m.goal).heading_rad) == pytest.approx(45.)
    # A saved pre-settle image cannot seed LOS or an artificial yaw-related LOS rate.
    old = dict(sequence=1, source=dict(m.feedback['source'], measurement_us=m.los_started_us - 1))
    assert m._target((old, (300, 250, 25, 25)), 1000 + step / 240) is None
    for step in range(step + 1, step + 80):
        rc = tick(m, step, yaw=45., bbox=(300, 250, 25, 25))
        assert abs(rc['channels'][3] - 1500) <= 1
        if m.phase == 'LOS_APPROACH':
            break
    assert m.phase == 'LOS_APPROACH'
    assert abs(m.nav_result.diagnostics.azimuth_rate_rad_s) < 1e-5


def test_settling_requires_continuous_yaw_and_altitude_dwell():
    m, step = after_hover()
    tick(m, step + 1)
    tick(m, step + 2, yaw=1., bbox=(300, 250, 25, 25))
    for step in range(step + 3, step + 35):
        tick(m, step, yaw=1., bbox=(300, 250, 25, 25))
    assert m.phase == 'YAW_SETTLE' and m.search.settled_sim is not None
    # A yaw-rate spike resets the dwell even though heading stays close.
    tick(m, step + 1, yaw=1.1, bbox=(300, 250, 25, 25))
    assert m.search.settled_sim is None
    for step in range(step + 2, step + 32):
        tick(m, step, yaw=1.1, bbox=(300, 250, 25, 25))
    assert m.phase == 'YAW_SETTLE'
    tick(m, step + 1, yaw=1.1, bbox=(300, 250, 25, 25), altitude=22.)
    assert m.search.settled_sim is None


def test_stalled_search_timeout_reset_and_stale_watchdog():
    m, step = after_hover()
    tick(m, step + 1)
    for step in range(step + 2, step + 8500):
        rc = tick(m, step)
        if m.phase == 'FAULT':
            break
    assert m.reason == 'search_timeout' and not rc['active']
    assert m.search.swept_rad == 0.
    tick(m, step + 1, episode='new', altitude=0.)
    assert m.search.started_sim is None and m.los_started_us is None
    m, step = after_hover()
    tick(m, step + 1)
    rc = m.tick(now_s=1001 + step / 240)
    assert not rc['active'] and m.reason == 'feedback_stale'


def test_search_refuses_profile_with_insufficient_yaw_authority():
    from dataclasses import replace
    from bullet_app.mission import Mission
    m = mission()
    config = replace(m.config, tuning=replace(m.config.tuning, yaw_rate_limit_rad_s=radians(10)))
    with pytest.raises(ValueError, match='at least 15 deg/s'):
        Mission(config)


def test_missing_camera_frame_gets_only_a_bounded_stationary_check():
    m, step = after_hover()
    for offset in range(1, 25):
        now = 1000 + (step + offset) / 240
        m.observe(feedback(step + offset, now, 20., armed=True, applied=m.sequence))
        m.tick(now_s=now, detection=None)
        if offset < 24:
            assert m.phase == 'WAIT_TARGET'
    assert m.phase == 'YAW_SEARCH'
    assert m.status(now)['target_remaining_s'] == 0.


@pytest.mark.parametrize('error_px', [-200., -40., -10., 0., 10., 40., 200.])
def test_center_rate_sign_deadband_and_saturation(error_px):
    from bullet_app.search import centering_command, SEARCH_RATE_RAD_S
    m = mission()
    error, rate = centering_command((310 + error_px, 230, 20, 20), m.config.camera)
    assert error == error_px
    assert abs(rate) <= SEARCH_RATE_RAD_S
    if abs(error_px) <= 10:
        assert rate == 0.
    else:
        assert rate * error_px < 0  # Positive image X must turn right (negative body yaw).
    if abs(error_px) == 200:
        assert abs(rate) == SEARCH_RATE_RAD_S


def test_off_center_visible_target_keeps_centering_and_corrects_overshoot():
    m, step = after_hover()
    rc = tick(m, step + 1, bbox=(420, 230, 20, 20))
    assert m.phase == 'YAW_CENTER' and rc['channels'][3] < 1500
    assert rc['channels'][:2] == [1500, 1500]
    assert m.diagnostics['target_error_x_px'] == 110
    assert m.diagnostics['search_commanded_yaw_rate_deg_s'] < 0
    assert not snapshot(m, now_s=1000 + (step + 1) / 240, sequence=1)['active']
    for step in range(step + 2, step + 140):
        tick(m, step, bbox=(340, 230, 20, 20))
        assert m.phase == 'YAW_CENTER' and m.nav_result is None
    # Crossing the center reverses yaw correction, rather than starting another circle.
    rc = tick(m, step + 1, bbox=(280, 230, 20, 20))
    assert m.phase == 'YAW_CENTER' and rc['channels'][3] > 1500
    rc = tick(m, step + 2, bbox=(310, 230, 20, 20))
    assert m.phase == 'YAW_SETTLE' and rc['channels'][3] == 1500
    assert m.diagnostics['target_error_x_px'] == 0 and m.diagnostics['target_centered']
    started = m.search.started_sim
    for step in range(step + 3, step + 30):
        tick(m, step, bbox=(310, 230, 20, 20))
    assert m.search.settled_sim is not None
    rc = tick(m, step + 1, bbox=(321, 230, 20, 20))
    assert m.phase == 'YAW_CENTER' and m.search.settled_sim is None
    assert rc['channels'][3] < 1500
    for step in range(step + 2, step + 160):
        tick(m, step, bbox=(310, 230, 20, 20))
        if m.phase == 'LOS_APPROACH':
            break
    assert m.phase == 'LOS_APPROACH' and m.search.started_sim == started


def test_target_loss_while_centering_resumes_original_scan():
    m, step = after_hover()
    tick(m, step + 1, bbox=(420, 230, 20, 20))
    started = m.search.started_sim
    assert m.phase == 'YAW_CENTER'
    rc = tick(m, step + 2)
    assert m.phase == 'YAW_SEARCH' and rc['channels'][3] > 1500
    assert m.search.started_sim == started
