"""Altitude-trend decisions tolerate noise without changing control feedback."""
from math import pi, sin, radians
from unittest.mock import Mock

import pytest

from bullet_drone.geometry import from_rpy
from test_mission import mission, feedback


def ready_hover(rate=0., *, hover_check=False):
    m = mission(hover_check=hover_check)
    m.observe(feedback(0, 1000., 0., state='stopped'))
    for step in range(1, 481):
        m.observe(feedback(step, 1000 + step / 240, 20 + rate * (step / 240 - 1.5), armed=True))
    m.running_seen = True
    m.phase = 'HOVER'
    m.started_sim, m.settled_sim = 0., 1.
    return m


def test_hover_check_restarts_stability_dwell_and_withdraws_at_completion():
    m = ready_hover(hover_check=True)
    first_hover = m.settled_sim
    disturbed = False
    rc = None
    for step in range(481, 8000):
        now = 1000 + step / 240
        altitude = 21. if 1200 <= step < 1201 else 20.
        m.observe(feedback(step, now, altitude, armed=True,
                          applied=0 if rc is None else rc['sequence']))
        rc = m.tick(now_s=now)
        if m.phase == 'TAKEOFF' and not disturbed:
            disturbed = True
            assert m.settled_sim is None
        if m.phase == 'COMPLETED':
            break
    assert disturbed and m.phase == 'COMPLETED' and m.reason == 'hover_complete'
    assert m.settled_sim > first_hover
    assert m.feedback['source']['simulation_time_s'] - m.settled_sim >= 20.
    assert not rc['active'] and rc['channels'][2] == 1000 and rc['channels'][4] == 1000
    assert m.status(now)['hover_remaining_s'] == 0.
    stopped = feedback(step + 1, now + .01, 20., state='stopped', applied=rc['sequence'])
    stopped['reason'] = 'app_withdrawn'
    m.observe(stopped)
    rc = m.tick(now_s=now + .01)
    assert m.phase == 'COMPLETED' and m.reason == 'hover_complete' and not rc['active']


def test_stationary_noise_completes_dwell_with_original_control_velocity():
    m = mission()
    m.observe(feedback(0, 1000., 0., state='stopped'))
    control = Mock(wraps=m.hold.control.step)
    m.hold.control.step = control
    spike_seen = False
    for step in range(1, 1500):
        now = 1000 + step / 240
        altitude = 0. if step < 50 else 20 + .03 * sin(2 * pi * 8 * step / 240)
        m.observe(feedback(step, now, altitude, armed=True, applied=m.sequence))
        m.tick(now_s=now)
        if m.phase in ('TAKEOFF', 'HOVER', 'WAIT_TARGET'):
            assert control.call_args.args[2] == m.vertical_speed
        if m.phase == 'HOVER' and abs(m.vertical_speed) > .2:
            spike_seen = True
            assert abs(m.diagnostics['settling_vertical_speed_m_s']) <= .2
        if m.phase == 'WAIT_TARGET':
            break
    assert spike_seen and m.phase == 'WAIT_TARGET', (m.phase, m.reason)
    assert m.diagnostics['settling_window_s'] >= 1. - 1e-9
    assert m.feedback['source']['simulation_time_s'] - m.settled_sim >= 3.


@pytest.mark.parametrize('rate', [-.3, .3])
def test_sustained_motion_resets_dwell_inside_altitude_envelope(rate):
    m = ready_hover(rate)
    assert abs(m.hold.estimate(1002.)[0] - m.goal) < .25
    m.tick(now_s=1002.)
    assert m.diagnostics['settling_vertical_speed_m_s'] == pytest.approx(rate)
    assert m.phase == 'TAKEOFF' and m.settled_sim is None


@pytest.mark.parametrize(('backend', 'altitude', 'settles'), [
    (None, 21.2, False),
    ('betaflight', 21.2, True),
])
def test_betaflight_allows_wider_takeoff_hover_altitude_envelope(backend, altitude, settles):
    m = ready_hover()
    samples = list(m._settling_altitudes)
    m._settling_altitudes.clear()
    step = 481
    now = 1000 + step / 240
    if backend:
        m.configure(dict(m.platform, schema_version=2, backend=backend, attitude_model='imu_complementary'))
    m._settling_altitudes.extend((time, altitude) for time, _ in samples)
    m.observe(feedback(step, now, altitude, armed=True))
    m.tick(now_s=now)
    assert (m.phase == 'HOVER') is settles


@pytest.mark.parametrize('axis', ['altitude', 'roll', 'pitch', 'heading'])
def test_altitude_and_attitude_excursions_still_reset_dwell(axis):
    m = ready_hover()
    packet = feedback(481, 1000 + 481 / 240, 21. if axis == 'altitude' else 20., armed=True)
    rpy = tuple(radians(6) if axis == name else 0. for name in ('roll', 'pitch', 'heading'))
    packet['orientation_xyzw'] = list(from_rpy(rpy))
    m.observe(packet)
    m.tick(now_s=1000 + 481 / 240)
    assert m.phase == 'TAKEOFF' and m.settled_sim is None


def test_irregular_measurements_fit_trend_and_require_full_window():
    m = mission()
    times = [0., .07, .2, .31, .5, .66, .8, .95, 1.1]
    for step, sim in enumerate(times):
        packet = feedback(step, 1000 + sim, 20 + .15 * sim, state='stopped')
        packet['source']['simulation_time_s'] = sim
        m.observe(packet)
        rate, span = m._settling_motion()
        if sim < 1.:
            assert rate is None
    assert span >= 1.
    assert rate == pytest.approx(.15)
    assert len(m._settling_altitudes) <= 241


def test_duplicates_do_not_advance_dwell_or_history():
    m = ready_hover()
    samples = list(m._settling_altitudes)
    for _ in range(10):
        m.observe(m.feedback)
        m.tick(now_s=1002.)
    assert list(m._settling_altitudes) == samples
    assert m.settled_sim == 1. and m.phase == 'HOVER'
    for step in range(481, 1441):
        m.observe(feedback(step, 1000 + step / 240, 20., armed=True))
    assert len(m._settling_altitudes) <= 241
    assert m._settling_motion()[0] == pytest.approx(0.)


@pytest.mark.parametrize('cause', ['missing', 'gap', 'clock'])
def test_missing_or_discontinuous_measurements_restart_confirmation(cause):
    m = ready_hover()
    step = 530 if cause == 'gap' else 481
    packet = feedback(step, 1000 + step / 240, None if cause == 'missing' else 20., armed=True)
    if cause == 'clock':
        packet['source']['simulation_time_s'] = 2.
    m.observe(packet)
    assert m._settling_motion() == (None, 0.)
    m.tick(now_s=1000 + step / 240)
    assert m.phase == 'TAKEOFF' and m.settled_sim is None


@pytest.mark.parametrize('change', ['episode', 'stream'])
def test_accepted_identity_changes_clear_settling_history(change):
    m = ready_hover()
    packet = feedback(0, 1002.01, 4., state='stopped', episode='new' if change == 'episode' else 'episode')
    if change == 'stream':
        m.configure(dict(m.platform, stream_id='new'))
        packet['source']['stream_id'] = 'new'
    m.observe(packet)
    assert m._settling_motion() == (None, 0.)
    assert len(m._settling_altitudes) == 1
    assert m.settled_sim is None
