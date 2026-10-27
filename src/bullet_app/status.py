"""Readable mission messages; machine reason codes stay in telemetry."""
from math import degrees


REASON_MESSAGES = {
    'searching_target': 'rotating to find a target',
    'centering_target': 'centering target horizontally with yaw',
    'settling_centered_target': 'target centered; stopping and settling yaw',
    'settling_yaw': 'target found; braking and settling yaw',
    'search_exhausted': 'no usable target found in one full yaw circle',
    'search_timeout': 'yaw search exceeded its 34 s deadline',
    'stale_or_foreign_image': 'waiting for a fresh image from this episode',
    'no_target': 'no target detected',
    'not_observed': 'waiting for target observations',
    'stale_image': 'target image is too old',
    'image_deadline': 'holding position before the target image expires',
    'close_target_commit': 'target fills the image; continuing the last tracking command',
    'clipped_target': 'target box touches the image edge',
    'attitude_unaligned': 'waiting for attitude aligned to the image timestamp',
    'los_deviation': 'target moved outside the allowed LOS deviation',
    'association_jump': 'target bearing jumped too far between images',
    'vertical_warmup': 'waiting for altitude and vertical-speed estimates',
    'feedback_stale': 'physics feedback is too old',
    'attitude_stale': 'attitude sample is too old',
    'barometer_stale': 'barometer altitude sample is too old',
    'sensors_stale': 'attitude or altitude samples are too old',
    'rc_expired': 'no valid RC command arrived before its deadline',
    'app_withdrawn': 'the app withdrew RC control',
    'controller_timeout': 'renderer controller heartbeat timed out',
    'controller_changed': 'renderer controller changed',
    'stop': 'Stop command received',
    'reset': 'Reset command received',
    'spawn_applied': 'scenario applied; waiting for Start',
    'waiting_app': 'waiting for app readiness',
    'app_not_ready': 'Start rejected because the app was not ready',
}


def status_key(status):
    """Log cause changes, not every fluctuating bearing sample."""
    d = status.get('diagnostics', {})
    return (status['episode_id'], status['phase'], status['reason'],
            d.get('vision_reason'), d.get('input_rejection'),
            d.get('physics_state'), d.get('physics_reason'))


def rejection_message(d):
    reason = d.get('vision_reason')
    if reason == 'bearing_outside_envelope':
        exceeded = []
        for axis in ('azimuth', 'elevation'):
            measured, limit = d.get(f'rejected_{axis}_rad'), d.get(f'max_{axis}_rad')
            if measured is not None and limit is not None and abs(measured) > limit:
                exceeded.append(f'{axis} {degrees(measured):+.2f} deg exceeds +/-{degrees(limit):.2f} deg')
        return 'target rejected: ' + ('; '.join(exceeded) if exceeded else 'bearing outside allowed angle limits')
    if reason and reason != 'usable':
        return REASON_MESSAGES.get(reason, reason.replace('_', ' '))
    return ''


def format_status(status):
    phase, reason = status['phase'], status['reason']
    d = status.get('diagnostics', {})
    rejection = rejection_message(d)
    if phase in ('YAW_SEARCH', 'YAW_CENTER', 'YAW_SETTLE'):
        message = REASON_MESSAGES.get(reason, reason.replace('_', ' '))
        swept = d.get('search_swept_deg', 0.)
        rate = d.get('search_measured_yaw_rate_deg_s', 0.)
        error = d.get('target_error_x_px')
        centered = '' if error is None else f'; X error {error:+.1f} px (limit +/-{d.get("target_center_tolerance_px", 10.):g})'
        return f'{message}{centered}; swept {swept:.1f}/360 deg; yaw {rate:+.1f} deg/s' + ('; ' + rejection if rejection else '')
    if phase == 'LOS_ACQUIRE':
        return 'Acquiring target' + (': ' + rejection if rejection else '; collecting consistent observations')
    if phase == 'FAULT':
        if reason == 'los_acquisition_timeout':
            timeout = d.get('acquisition_timeout_s')
            duration = f' after {timeout:g} s' if timeout is not None else ''
            message = 'LOS acquisition timed out' + duration
        else:
            message = REASON_MESSAGES.get(reason, reason.replace('_', ' '))
        return message + ('; last visual issue: ' + rejection if rejection else '')
    if phase == 'READY' and d.get('physics_state') in ('stopped', 'error'):
        stop_reason = d.get('physics_reason', reason)
        return f"Physics {d['physics_state']}: {REASON_MESSAGES.get(stop_reason, stop_reason.replace('_', ' '))} [{stop_reason}]"
    message = REASON_MESSAGES.get(reason, reason.replace('_', ' '))
    return message + ('; ' + rejection if phase.startswith('LOS_') and rejection else '')
