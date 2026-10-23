from copy import deepcopy
from math import radians

from bullet_app.status import format_status, status_key


def status(phase='LOS_ACQUIRE', reason='acquiring'):
    return dict(episode_id='episode', phase=phase, reason=reason, diagnostics=dict(
        vision_reason='bearing_outside_envelope', rejected_azimuth_rad=radians(36.2),
        rejected_elevation_rad=radians(10), max_azimuth_rad=radians(35), max_elevation_rad=radians(60),
        acquisition_timeout_s=5., physics_state='running', physics_reason='started'))


def test_acquiring_and_timeout_show_rejected_degrees_and_limits():
    s = status()
    message = format_status(s)
    assert 'azimuth +36.20 deg exceeds +/-35.00 deg' in message
    assert 'elevation' not in message
    s.update(phase='FAULT', reason='los_acquisition_timeout')
    message = format_status(s)
    assert 'timed out after 5 s' in message
    assert 'azimuth +36.20 deg' in message
    s['diagnostics'].update(rejected_azimuth_rad=None, rejected_elevation_rad=radians(-61))
    assert 'elevation -61.00 deg exceeds +/-60.00 deg' in format_status(s)


def test_log_key_changes_for_cause_not_noisy_angle_values():
    s = status()
    noisy = deepcopy(s)
    noisy['diagnostics']['rejected_azimuth_rad'] = radians(36.3)
    assert status_key(s) == status_key(noisy)
    noisy['diagnostics']['vision_reason'] = 'no_target'
    assert status_key(s) != status_key(noisy)
    assert 'no target detected' in format_status(noisy)
    noisy.update(episode_id='new')
    assert status_key(s) != status_key(noisy)


def test_idle_message_preserves_physics_stop_and_watchdog_reason():
    s = status('READY', 'stop')
    s['diagnostics'].update(physics_state='stopped', physics_reason='stop')
    assert 'Stop command received [stop]' in format_status(s)
    s['diagnostics']['physics_reason'] = 'rc_expired'
    assert 'no valid RC command arrived before its deadline [rc_expired]' in format_status(s)


def test_search_progress_settling_and_terminal_reasons():
    s = status('YAW_SEARCH', 'searching_target')
    s['diagnostics'].update(search_swept_deg=123.4, search_measured_yaw_rate_deg_s=15.)
    assert 'swept 123.4/360 deg' in format_status(s)
    assert 'yaw +15.0 deg/s' in format_status(s)
    noisy = deepcopy(s)
    noisy['diagnostics']['search_swept_deg'] = 123.5
    assert status_key(s) == status_key(noisy)
    s.update(phase='YAW_SETTLE', reason='settling_yaw')
    assert 'braking and settling yaw' in format_status(s)
    s.update(phase='FAULT', reason='search_exhausted')
    assert 'one full yaw circle' in format_status(s)


def test_centering_message_includes_pixel_error_without_angle_noise_spam():
    s = status('YAW_CENTER', 'centering_target')
    s['diagnostics'].update(target_error_x_px=-23.5, target_center_tolerance_px=10.)
    assert 'centering target horizontally' in format_status(s)
    assert 'X error -23.5 px (limit +/-10)' in format_status(s)
    noisy = deepcopy(s)
    noisy['diagnostics']['target_error_x_px'] = -23.
    assert status_key(s) == status_key(noisy)


def test_image_deadline_explains_early_hold():
    s = status()
    s['diagnostics']['vision_reason'] = 'image_deadline'
    assert 'before the target image expires' in format_status(s)
