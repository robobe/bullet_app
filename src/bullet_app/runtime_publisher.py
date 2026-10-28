"""Own mission-runtime ZMQ publication independently of control."""
import json
import time

import zmq
from bullet_msgs.flight import RC_TOPIC, MISSION_TOPIC, encode, validate_rc
from bullet_msgs.los import LOS_TOPIC, validate_los

from .publisher import BboxPublisher


PUBLISH_PERIOD_S = .01
MSP_PERIOD_S = .05
CONFIG_PERIOD_S = 1.
MSP_TOPIC = b'analysis/app/msp'
MSP_RC_TOPIC = b'analysis/app/msp_rc'
CONFIG_TOPIC = b'analysis/app/config'


class RuntimePublisher:
    """Schedule latest state and ordered output events on publisher-owned sockets."""
    def __init__(self, transport, bbox_endpoint, blackboard, stop):
        self.transport = transport
        self.bbox_endpoint = bbox_endpoint
        self.blackboard = blackboard
        self.stop = stop

    def run(self):
        bbox_publisher = None
        try:
            with zmq.Context() as context, context.socket(zmq.PUB) as commands:
                commands.setsockopt(zmq.LINGER, 0)
                commands.setsockopt(zmq.SNDHWM, 4)
                commands.bind(self.transport['flight_command_endpoint'])
                bbox_publisher = BboxPublisher(self.bbox_endpoint)
                next_msp = next_config = time.monotonic()
                while not self.stop.is_set():
                    now = time.monotonic()
                    inputs = self.blackboard.take_publication_inputs()
                    self._publish_final_disarm(commands, inputs.final_disarm)
                    self._publish_events(bbox_publisher, inputs)
                    self._publish_latest(commands, bbox_publisher.socket, inputs, now, next_msp, next_config)
                    next_msp = self._next_deadline(next_msp, MSP_PERIOD_S, now)
                    next_config = self._next_deadline(next_config, CONFIG_PERIOD_S, now)
                    self.stop.wait(PUBLISH_PERIOD_S)
                inputs = self.blackboard.take_publication_inputs()
                self._publish_final_disarm(commands, inputs.final_disarm)
                self._publish_events(bbox_publisher, inputs)
        finally:
            if bbox_publisher is not None:
                bbox_publisher.close()

    def _publish_latest(self, commands, analysis, inputs, now_s, next_msp, next_config):
        if inputs.command is not None:
            self._send_command(commands, inputs.command.value)
        if inputs.status is not None:
            self._send_json(analysis, MISSION_TOPIC, inputs.status.value)
        if inputs.msp_snapshot is not None and now_s >= next_msp:
            self._send_json(analysis, MSP_TOPIC, inputs.msp_snapshot.value)
        if inputs.config_snapshot is not None and now_s >= next_config:
            self._send_json(analysis, CONFIG_TOPIC, inputs.config_snapshot.value)

    def _publish_events(self, bbox_publisher, inputs):
        for event in inputs.bbox_events:
            if not bbox_publisher.publish(*event.value):
                self.blackboard.record_output_drop('bbox')
        for event in inputs.los_events:
            try:
                bbox_publisher.socket.send_multipart([LOS_TOPIC, encode(event.value, validate_los)], flags=zmq.NOBLOCK)
            except zmq.Again:
                self.blackboard.record_output_drop('los')
        for event in inputs.msp_delivery_events:
            if not self._send_json(bbox_publisher.socket, MSP_RC_TOPIC, event.value):
                self.blackboard.record_output_drop('msp_delivery')

    def _publish_final_disarm(self, commands, final_disarm):
        if final_disarm is not None and self._send_command(commands, final_disarm.value, final=True):
            self.blackboard.acknowledge_final_disarm()

    @staticmethod
    def _send_command(commands, command, *, final=False):
        if not final and command['valid_until_us'] <= time.monotonic_ns() // 1000:
            return False
        try:
            commands.send_multipart([RC_TOPIC, encode(command, validate_rc)], flags=zmq.NOBLOCK)
        except zmq.Again:
            return False
        return True

    @staticmethod
    def _send_json(socket, topic, value):
        try:
            socket.send_multipart([topic, json.dumps(value, allow_nan=False).encode()], flags=zmq.NOBLOCK)
        except zmq.Again:
            return False
        return True

    @staticmethod
    def _next_deadline(deadline, period_s, now_s):
        if now_s < deadline:
            return deadline
        return deadline + (int((now_s - deadline) / period_s) + 1) * period_s
