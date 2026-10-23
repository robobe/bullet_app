"""Publish schema-validated bounding boxes as bounded, nonblocking ZMQ JSON."""

import json

import zmq
from bullet_msgs import schema_bytes, validate_endpoint
from jsonschema import Draft202012Validator, ValidationError


BBOX_TOPIC = "tracking/front/bbox"


class BboxPublisher:
    def __init__(self, endpoint):
        validate_endpoint(endpoint)
        self.validator = Draft202012Validator(json.loads(schema_bytes("bbox.json")))
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        try:
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.setsockopt(zmq.SNDHWM, 64)
            self.socket.bind(endpoint)
        except BaseException:
            self.close()
            raise

    def publish(self, metadata, bbox, image_topic):
        """Return whether ZMQ accepted the result, without guaranteeing delivery."""
        packet = dict(schema_version=1, image_topic=image_topic,
                      session=metadata["session"], sequence=metadata["sequence"],
                      capture_ns=metadata["capture_ns"],
                      bbox=None if bbox is None else dict(zip(("x", "y", "width", "height"), bbox, strict=True)))
        if metadata.get('version') == 2:
            from bullet_msgs.flight import validate_source
            source = validate_source(metadata.get('source'))
            if metadata['capture_ns'] != source['measurement_us'] * 1000:
                raise ValueError('bbox source/capture timestamp mismatch')
            packet.update(schema_version=2, source=source)
        try:
            self.validator.validate(packet)
        except ValidationError as error:
            raise ValueError(f"invalid bbox message: {error.message}") from error
        payload = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode()
        try:
            self.socket.send_multipart([BBOX_TOPIC.encode(), payload], flags=zmq.NOBLOCK)
        except zmq.Again:
            return False
        return True

    def close(self):
        self.socket.close(linger=0)
        self.context.term()
