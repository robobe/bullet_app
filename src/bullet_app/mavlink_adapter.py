"""Publish latest mission feedback as standard MAVLink telemetry."""
import time
from threading import Event

from los_tracking.geometry import euler
from pymavlink import mavutil
from bullet_msgs.status_messages import (MAV_SEVERITY_ERROR, STATUS_MESSAGE_BY_KEY,
                                         status_text_chunks)
from bullet_msgs.mavlink_control import (RELOAD_PROFILE_ACK_ACCEPTED, RELOAD_PROFILE_ACK_DENIED,
                                         RELOAD_PROFILE_COMMAND, RELOAD_PROFILE_PARAMETER)
from .status import format_status, status_key


HEARTBEAT_PERIOD_S = 1.
TELEMETRY_PERIOD_S = .05
FEEDBACK_STALE_S = .2
SCHEDULER_POLL_S = .01
SYSTEM_ID = 1
COMPONENT_ID = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
UNKNOWN_FLOAT = float('nan')
BBOX_NAMED_VALUES = ('BBOX_VALID', 'BBOX_X', 'BBOX_Y', 'BBOX_W', 'BBOX_H')
RANGEFINDER_MIN_DISTANCE_CM = 1
RANGEFINDER_MAX_DISTANCE_CM = 5000
RANGEFINDER_INVALID_CM = 65535


def mavlink_attitude(orientation_xyzw):
    """Convert Bullet FLU/Z-up XYZW orientation to MAVLink FRD/NED Euler angles."""
    x, y, z, w = orientation_xyzw
    return euler((x, -y, -z, w))


class MavlinkAdapter:
    """Own a publish-only MAVLink connection and read feedback from the blackboard."""
    def __init__(self, endpoint, blackboard, stop, *, control_endpoint=None,
                 connection_factory=mavutil.mavlink_connection,
                 clock=time.monotonic):
        self.endpoint = endpoint
        self.control_endpoint = control_endpoint
        self.blackboard = blackboard
        self.stop = stop
        self.connection_factory = connection_factory
        self.clock = clock
        self.connection = None
        self.control_connection = None
        self.reload_requested = Event()
        self.last_status_key = None

    def open(self):
        self.connection = self.connection_factory(self.endpoint, source_system=SYSTEM_ID,
                                                  source_component=COMPONENT_ID)
        if self.control_endpoint is not None:
            self.control_connection = self.connection_factory(self.control_endpoint,
                                                               source_system=SYSTEM_ID,
                                                               source_component=COMPONENT_ID)

    def close(self):
        if self.connection is not None:
            self.connection.close()
        if self.control_connection is not None:
            self.control_connection.close()

    def take_reload_request(self):
        requested = self.reload_requested.is_set()
        self.reload_requested.clear()
        return requested

    def run(self):
        if self.connection is None:
            raise RuntimeError('MAVLink adapter is not open')
        next_heartbeat = next_telemetry = self.clock()
        while not self.stop.is_set():
            now_s = self.clock()
            feedback = self._fresh_feedback(now_s)
            self._poll_control()
            status = self.blackboard.latest_status()
            if now_s >= next_heartbeat:
                self._send_heartbeat(feedback)
                next_heartbeat = self._next_deadline(next_heartbeat, HEARTBEAT_PERIOD_S, now_s)
            if status is not None:
                current_status_key = status_key(status.value)
                if current_status_key != self.last_status_key:
                    self._send_status(status.value)
                    self.last_status_key = current_status_key
            if now_s >= next_telemetry:
                if feedback is not None:
                    self._send_telemetry(feedback.value)
                self._send_bbox(now_s)
                next_telemetry = self._next_deadline(next_telemetry, TELEMETRY_PERIOD_S, now_s)
            self.stop.wait(SCHEDULER_POLL_S)

    def _poll_control(self):
        if self.control_connection is None:
            return
        command = self.control_connection.recv_match(type='COMMAND_LONG', blocking=False)
        if command is None:
            return
        if (command.command != RELOAD_PROFILE_COMMAND
                or command.target_system not in (0, SYSTEM_ID)
                or command.target_component not in (0, COMPONENT_ID)
                or abs(command.param1 - RELOAD_PROFILE_PARAMETER) > 1e-6):
            self._send_reload_ack(RELOAD_PROFILE_ACK_DENIED, 'unsupported reload command')
            return
        self.reload_requested.set()

    def reply_reload(self, accepted, message):
        self._send_reload_ack(RELOAD_PROFILE_ACK_ACCEPTED if accepted else RELOAD_PROFILE_ACK_DENIED,
                              message)

    def _send_reload_ack(self, result, message):
        self.control_connection.mav.command_ack_send(RELOAD_PROFILE_COMMAND, result)
        self.connection.mav.statustext_send(6, message.encode('utf-8'))

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
        rangefinder = feedback.get('rangefinder')
        if rangefinder is not None:
            distance_cm = rangefinder['distance_cm'] if rangefinder['valid'] else None
            if distance_cm is None:
                distance_cm = RANGEFINDER_INVALID_CM
            else:
                distance_cm = min(RANGEFINDER_INVALID_CM - 1, max(0, round(distance_cm)))
            self.connection.mav.distance_sensor_send(
                time_us // 1000 % 2**32, RANGEFINDER_MIN_DISTANCE_CM,
                RANGEFINDER_MAX_DISTANCE_CM, distance_cm,
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, 0,
                mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, 0)

    def _send_bbox(self, now_s):
        vision = self.blackboard.latest_vision()
        bbox = None if vision is None else vision.value[1]
        values = {
            'BBOX_VALID': 1. if bbox is not None else 0.,
            'BBOX_X': UNKNOWN_FLOAT if bbox is None else float(bbox[0]),
            'BBOX_Y': UNKNOWN_FLOAT if bbox is None else float(bbox[1]),
            'BBOX_W': UNKNOWN_FLOAT if bbox is None else float(bbox[2]),
            'BBOX_H': UNKNOWN_FLOAT if bbox is None else float(bbox[3]),
        }
        timestamp_ms = int(now_s * 1000) % 2**32
        for name in BBOX_NAMED_VALUES:
            self.connection.mav.named_value_float_send(timestamp_ms, name.encode('ascii'), values[name])

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
