"""Public-result mapping and 30 Hz publication on the mission-owned app socket."""
from copy import deepcopy
from math import degrees, floor, isfinite

import zmq
from bullet_msgs.flight import encode
from bullet_msgs.los import LOS_HZ, LOS_TOPIC, validate_los


MICROSECONDS_PER_SECOND = 1_000_000
PERIOD_S = 1 / LOS_HZ
TIME_EPSILON_S = 1e-9
LOS_PHASES = ('LOS_ACQUIRE', 'LOS_APPROACH')


def snapshot(mission, *, now_s, sequence, shutdown=False):
    """Map only public mission/result data; never alter controller state."""
    if mission.feedback is None:
        return None
    if not isfinite(now_s):
        raise ValueError('invalid LOS telemetry clock')
    result = mission.nav_result
    active = (not shutdown and mission.phase in LOS_PHASES and mission.feedback['state'] == 'running'
              and result is not None and result.usable and result.generated_at_s <= now_s < result.valid_until_s)
    reason = mission.reason
    if mission.phase == 'READY' and mission.feedback['state'] != 'running':
        reason = mission.feedback['reason']
    packet = dict(schema_version=1, app_id=mission.app_id, sequence=sequence,
                  source=deepcopy(mission.feedback['source']),
                  generated_us=round((result.generated_at_s if active else now_s) * MICROSECONDS_PER_SECOND),
                  navigation_sequence=result.sequence if active else None,
                  navigator_state=None if result is None else result.state.value,
                  stage=None if result is None else result.stage.value,
                  reason='app_shutdown' if shutdown else result.reason if active else reason,
                  mission_phase=mission.phase, active=active,
                  elevation_deg=None, azimuth_deg=None, bearing_valid=False,
                  elevation_rate_deg_s=None, azimuth_rate_deg_s=None, rate_observable=False,
                  optical_ttc_s=None, ttc_valid=False, ttc_reason='inactive', ttc_relative_residual=None)
    if active:
        d = result.diagnostics
        packet['bearing_valid'] = d.bearing_valid
        packet['rate_observable'] = d.rate_observable
        if d.bearing_valid:
            packet.update(elevation_deg=degrees(d.elevation_rad), azimuth_deg=degrees(d.azimuth_rad))
        if d.rate_observable:
            packet.update(elevation_rate_deg_s=degrees(d.elevation_rate_rad_s),
                          azimuth_rate_deg_s=degrees(d.azimuth_rate_rad_s))
        qualified = d.optical_ttc_s is not None and d.ttc_reason == 'qualified'
        packet.update(optical_ttc_s=d.optical_ttc_s if qualified else None, ttc_valid=qualified,
                      ttc_reason=d.ttc_reason, ttc_relative_residual=d.ttc_relative_residual)
    return validate_los(packet)


class LosTelemetryPublisher:
    """Borrow a socket owned by the mission thread; never bind, block or spawn."""
    def __init__(self, socket):
        self.socket = socket
        self.sequence = 0
        self.last_key = None
        self.next_sample_s = None
        self.last_now_s = None

    def publish(self, mission, *, now_s, shutdown=False):
        if self.last_now_s is not None and now_s < self.last_now_s:
            raise ValueError('LOS telemetry clock moved backwards')
        self.last_now_s = now_s
        packet = snapshot(mission, now_s=now_s, sequence=self.sequence + 1, shutdown=shutdown)
        if packet is None:
            return None
        source = packet['source']
        key = (source['physics_id'], source['episode_id'], source['stream_id'], packet['mission_phase'],
               packet['navigator_state'], packet['stage'], packet['reason'], packet['active'])
        changed = key != self.last_key
        due = packet['active'] and (self.next_sample_s is None or now_s + TIME_EPSILON_S >= self.next_sample_s)
        if not changed and not due:
            return None
        try:
            self.socket.send_multipart([LOS_TOPIC, encode(packet, validate_los)], flags=zmq.NOBLOCK)
        except zmq.Again:
            return None
        self.sequence += 1
        self.last_key = key
        if changed or self.next_sample_s is None:
            self.next_sample_s = now_s + PERIOD_S
        else:
            # Keep phase across 100 Hz ticks: adding a period to 'now' would yield 25 Hz.
            skipped = max(1, floor((now_s + TIME_EPSILON_S - self.next_sample_s) / PERIOD_S) + 1)
            self.next_sample_s += skipped * PERIOD_S
        return packet
