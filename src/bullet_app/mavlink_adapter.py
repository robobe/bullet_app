"""Publish latest mission feedback as standard MAVLink telemetry."""
import time

from los_tracking.geometry import euler
from pymavlink import mavutil
from bullet_msgs.status_messages import (MAV_SEVERITY_ERROR, STATUS_MESSAGE_BY_KEY,
                                         status_text_chunks)
from .status import format_status, status_key


HEARTBEAT_PERIOD_S = 1.
TELEMETRY_PERIOD_S = .05
FEEDBACK_STALE_S = .2
SCHEDULER_POLL_S = .01
SYSTEM_ID = 1
COMPONENT_ID = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
UNKNOWN_FLOAT = float('nan')


def mavlink_attitude(orientation_xyzw):
    """Convert Bullet FLU/Z-up XYZW orientation to MAVLink FRD/NED Euler angles."""
    x, y, z, w = orientation_xyzw
    return euler((x, -y, -z, w))


class MavlinkAdapter:
    """Own a publish-only MAVLink connection and read feedback from the blackboard."""
    def __init__(self, endpoint, blackboard, stop, *, connection_factory=mavutil.mavlink_connection,
                 clock=time.monotonic):
        self.endpoint = endpoint
        self.blackboard = blackboard
        self.stop = stop
        self.connection_factory = connection_factory
        self.clock = clock
        self.connection = None
        self.last_status_key = None

    def open(self):
        self.connection = self.connection_factory(self.endpoint, source_system=SYSTEM_ID,
                                                  source_component=COMPONENT_ID)

    def close(self):
        if self.connection is not None:
            self.connection.close()

    def run(self):
        if self.connection is None:
            raise RuntimeError('MAVLink adapter is not open')
        next_heartbeat = next_telemetry = self.clock()
        while not self.stop.is_set():
            now_s = self.clock()
            feedback = self._fresh_feedback(now_s)
            status = self.blackboard.latest_status()
            if now_s >= next_heartbeat:
                self._send_heartbeat(feedback)
                next_heartbeat = self._next_deadline(next_heartbeat, HEARTBEAT_PERIOD_S, now_s)
            if status is not None:
                current_status_key = status_key(status.value)
                if current_status_key != self.last_status_key:
                    self._send_status(status.value)
                    self.last_status_key = current_status_key
            if feedback is not None and now_s >= next_telemetry:
                self._send_telemetry(feedback.value)
                next_telemetry = self._next_deadline(next_telemetry, TELEMETRY_PERIOD_S, now_s)
            self.stop.wait(SCHEDULER_POLL_S)

    def _fresh_feedback(self, now_s):
        feedback = self.blackboard.latest_feedback()
        if feedback is None or now_s - feedback.received_at_s > FEEDBACK_STALE_S:
            return None
        return feedback

    def _send_heartbeat(self, feedback):
        armed = feedback is not None and feedback.value['armed']
        running = feedback is not None and feedback.value['state'] == 'running'
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
        system_status = (mavutil.mavlink.MAV_STATE_ACTIVE if running
                         else mavutil.mavlink.MAV_STATE_STANDBY)
        self.connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                           mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                           base_mode, 0, system_status, 3)

    def _send_telemetry(self, feedback):
        source = feedback['source']
        time_us = source['measurement_us']
        roll, pitch, yaw = mavlink_attitude(feedback['orientation_xyzw'])
        self.connection.mav.attitude_send(time_us // 1000 % 2**32, roll, pitch, yaw,
                                          UNKNOWN_FLOAT, UNKNOWN_FLOAT, UNKNOWN_FLOAT)
        altitude = feedback['barometer_altitude_m']
        if altitude is not None:
            self.connection.mav.altitude_send(time_us, UNKNOWN_FLOAT, UNKNOWN_FLOAT, altitude,
                                              UNKNOWN_FLOAT, UNKNOWN_FLOAT, UNKNOWN_FLOAT)

    def _send_status(self, status):
        reason = status.get('reason', '')
        key = reason if reason in STATUS_MESSAGE_BY_KEY else status.get('phase', '').lower()
        definition = STATUS_MESSAGE_BY_KEY.get(key, STATUS_MESSAGE_BY_KEY['fault'])
        severity = MAV_SEVERITY_ERROR if status.get('phase') == 'FAULT' else definition.severity
        text = f"{status.get('phase', 'UNKNOWN')}: {format_status(status)}"
        for chunk in status_text_chunks(definition.index, text):
            self.connection.mav.statustext_send(severity, chunk.encode('utf-8'))

    @staticmethod
    def _next_deadline(deadline, period_s, now_s):
        if now_s < deadline:
            return deadline
        return deadline + (int((now_s - deadline) / period_s) + 1) * period_s
