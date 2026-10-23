from contextlib import contextmanager
import fcntl
import time

import numpy as np
import pytest
import zmq
from bullet_msgs.shm import FrameWriter, topic_prefix

from bullet_app.receiver import ImageReceiver
from bullet_app.detector import detect_red_box


TOPIC = "camera/front/image"


@contextmanager
def connection():
    with zmq.Context() as context, context.socket(zmq.XPUB) as publisher:
        publisher.setsockopt(zmq.LINGER, 0)
        publisher.bind_to_random_port("tcp://127.0.0.1")
        receiver = ImageReceiver(publisher.getsockopt_string(zmq.LAST_ENDPOINT), TOPIC)
        try:
            assert publisher.poll(5000)
            assert publisher.recv() == b"\x01" + topic_prefix(TOPIC)
            yield publisher, receiver
        finally:
            receiver.close()
            assert not receiver.reader.mappings
            assert receiver.socket.closed
            assert receiver.context.closed


def receive(receiver):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        result = receiver.poll()
        if result is not None:
            return result
        time.sleep(0.005)
    raise AssertionError("no frame received")


def assert_skipped(receiver):
    before = receiver.reader.skipped
    deadline = time.monotonic() + 5
    while receiver.reader.skipped == before and time.monotonic() < deadline:
        assert receiver.poll() is None
        time.sleep(0.005)
    assert receiver.reader.skipped == before + 1


def test_rgb_padding_owned_pixels_and_new_publisher_session():
    with connection() as (publisher, receiver):
        assert receiver.poll() is None
        with FrameWriter(2, 1, step=8) as writer:
            publisher.send(writer.write(TOPIC, bytes([1, 2, 3, 4, 5, 6, 0, 0])))
            metadata, image = receive(receiver)
            assert image.tolist() == [[[3, 2, 1], [6, 5, 4]]]
            assert image.flags.owndata and image.flags.writeable
            session = metadata["session"]
        with FrameWriter(2, 1, "bgr8", step=8) as writer:
            publisher.send(writer.write(TOPIC, bytes([7, 8, 9, 10, 11, 12, 0, 0])))
            metadata, second = receive(receiver)
            assert metadata["session"] != session
            assert metadata["sequence"] == 1
            assert second.tolist() == [[[7, 8, 9], [10, 11, 12]]]
            assert second.flags.owndata and second.flags.writeable
            assert image.tolist() == [[[3, 2, 1], [6, 5, 4]]]


def test_exact_topic_over_tcp():
    with connection() as (publisher, receiver), FrameWriter(8, 8) as writer:
        pixels = bytes([255, 0, 0]) * 64
        publisher.send(writer.write(TOPIC + "/other", pixels))
        publisher.send(writer.write(TOPIC + "2", pixels))
        deadline = time.monotonic() + 0.1
        while time.monotonic() < deadline:
            assert receiver.poll() is None
            time.sleep(0.005)
        publisher.send(writer.write(TOPIC, pixels))
        _, image = receive(receiver)
        assert detect_red_box(image) == (0, 0, 8, 8)


def test_duplicate_overwritten_missing_and_busy_frames():
    with connection() as (publisher, receiver), FrameWriter(2, 1) as writer:
        pixels = bytes(6)
        message = writer.write(TOPIC, pixels)
        publisher.send(message)
        receive(receiver)
        publisher.send(message)
        assert_skipped(receiver)
        stale = writer.write(TOPIC, pixels)
        for _ in range(3):
            writer.write(TOPIC, pixels)
        publisher.send(stale)
        assert_skipped(receiver)
        with FrameWriter(2, 1) as missing:
            missing_message = missing.write(TOPIC, pixels)
        publisher.send(missing_message)
        assert_skipped(receiver)
        busy_message = writer.write(TOPIC, pixels)
        fd = writer.slots[(writer.sequence - 1) % 3][1]
        fcntl.flock(fd, fcntl.LOCK_EX)
        try:
            publisher.send(busy_message)
            assert_skipped(receiver)
        finally:
            fcntl.flock(fd, fcntl.LOCK_UN)
        publisher.send(busy_message)
        receive(receiver)


def test_malformed_descriptor_reports_failure_and_cleans_up():
    with connection() as (publisher, receiver):
        publisher.send(topic_prefix(TOPIC) + b"invalid")
        with pytest.raises(ValueError, match="length"):
            receive(receiver)
