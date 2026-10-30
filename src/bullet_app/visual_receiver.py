"""Receive schema-validated bbox detections from bullet_visual."""
import json

import zmq
from bullet_msgs import schema_bytes, validate_endpoint
from jsonschema import Draft202012Validator, ValidationError


VISUAL_BBOX_TOPIC = b'tracking/front/bbox'
MAX_VISUAL_BYTES = 64 * 1024


class VisualReceiver:
    def __init__(self, endpoint):
        validate_endpoint(endpoint)
        self.validator = Draft202012Validator(json.loads(schema_bytes('bbox.json')))
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.RCVHWM, 64)
        self.socket.setsockopt(zmq.MAXMSGSIZE, MAX_VISUAL_BYTES)
        self.socket.setsockopt(zmq.SUBSCRIBE, VISUAL_BBOX_TOPIC)
        self.socket.connect(endpoint)

    def poll(self):
        try:
            if not self.socket.poll(0):
                return None
            parts = self.socket.recv_multipart()
            if len(parts) != 2 or parts[0] != VISUAL_BBOX_TOPIC:
                raise ValueError('invalid visual bbox message framing')
            value = json.loads(parts[1].decode('utf-8'), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
            self.validator.validate(value)
            return value
        except (UnicodeError, json.JSONDecodeError, ValidationError, TypeError, ValueError) as error:
            raise ValueError(f'invalid visual bbox message: {error}') from error

    def close(self):
        self.socket.close(linger=0)
        self.context.term()
