import json
import signal
import socket
import subprocess
import sys
import time

import numpy as np
import pytest
import yaml
import zmq
from bullet_msgs import schema_bytes
from bullet_msgs.shm import FrameWriter, topic_prefix
from mcap.reader import make_reader
from mcap_bagger.config import Config
from mcap_bagger.recorder import Recording

from bullet_app.publisher import BBOX_TOPIC, BboxPublisher


def free_endpoints():
    sockets = [socket.socket() for _ in range(4)]
    try:
        for item in sockets:
            item.bind(("127.0.0.1", 0))
        return [f"tcp://127.0.0.1:{item.getsockname()[1]}" for item in sockets]
    finally:
        for item in sockets:
            item.close()


def test_real_image_detection_and_no_target_recorded_to_mcap(tmp_path):
    telemetry, images, control, bbox_endpoint = free_endpoints()
    profile = tmp_path / "transport.json"
    profile.write_text(json.dumps(dict(telemetry_endpoint=telemetry, image_endpoint=images, control_endpoint=control)))
    config_path = tmp_path / "recorder.yaml"
    config_path.write_text(yaml.safe_dump(dict(project="test", output_dir="recordings",
        transport_config="transport.json", topics=[dict(key=BBOX_TOPIC, input="json",
            endpoint=bbox_endpoint, schema="bullet_msgs:bbox.json", timestamp_source="receive")])))
    recording = Recording(Config.load(config_path), on_error=lambda message: None)
    received = []
    original_receive = recording.receive

    def receive(topic, payload, received_ns=None):
        received.append(json.loads(bytes(payload)))
        original_receive(topic, payload, received_ns)

    recording.receive = receive
    recording.start()
    process = None
    try:
        with zmq.Context() as context, context.socket(zmq.XPUB) as publisher, FrameWriter(20, 16) as writer:
            publisher.setsockopt(zmq.LINGER, 0)
            publisher.bind(images)
            process = subprocess.Popen([sys.executable, "-m", "bullet_app.main",
                "--transport-config", str(profile), "--bbox-endpoint", bbox_endpoint],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            assert publisher.poll(5000), "app did not subscribe"
            assert publisher.recv() == b"\x01" + topic_prefix("camera/front/image")
            source_frames = {}
            for target in (True, False):
                rgb = np.zeros((16, 20, 3), dtype=np.uint8)
                if target:
                    rgb[4:10, 3:11] = (255, 0, 0)
                expected = dict(x=3, y=4, width=8, height=6) if target else None
                deadline = time.monotonic() + 5
                # PUB/SUB may lose results while subscriptions establish.
                while not any(packet["bbox"] == expected for packet in received):
                    assert process.poll() is None
                    assert time.monotonic() < deadline, recording.error
                    message = writer.write("camera/front/image", rgb.tobytes())
                    source_frames[writer.sequence] = target
                    publisher.send(message)
                    time.sleep(0.03)
            process.send_signal(signal.SIGINT)
            stdout, stderr = process.communicate(timeout=5)
            assert process.returncode == 0, (stdout, stderr)
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.communicate()
        recording.close()
    assert recording.error is None
    assert recording.path.suffix == ".mcap"
    with recording.path.open("rb") as stream:
        messages = list(make_reader(stream).iter_messages())
    results = []
    for schema, channel, message in messages:
        assert channel.topic == BBOX_TOPIC
        assert json.loads(schema.data) == json.loads(schema_bytes("bbox.json"))
        packet = json.loads(message.data)
        assert packet["session"] == writer.session
        assert packet["image_topic"] == "camera/front/image"
        assert packet["capture_ns"] > 0
        expected = dict(x=3, y=4, width=8, height=6) if source_frames[packet["sequence"]] else None
        assert packet["bbox"] == expected
        results.append(packet["bbox"])
    assert dict(x=3, y=4, width=8, height=6) in results
    assert None in results


def test_publisher_exact_wire_contract_validation_and_cleanup():
    endpoint = free_endpoints()[0]
    publisher = BboxPublisher(endpoint)
    try:
        with zmq.Context() as context, context.socket(zmq.SUB) as subscriber:
            subscriber.setsockopt(zmq.LINGER, 0)
            subscriber.setsockopt(zmq.SUBSCRIBE, BBOX_TOPIC.encode())
            subscriber.connect(endpoint)
            metadata = dict(session="a" * 32, sequence=1, capture_ns=99)
            deadline = time.monotonic() + 5
            while not subscriber.poll(20):
                assert time.monotonic() < deadline
                publisher.publish(metadata, (1, 2, 3, 4), "camera/front/image")
            parts = subscriber.recv_multipart()
            assert len(parts) == 2 and parts[0] == BBOX_TOPIC.encode()
            assert json.loads(parts[1])["bbox"] == dict(x=1, y=2, width=3, height=4)
            with pytest.raises(ValueError, match="invalid bbox"):
                publisher.publish(metadata, (-1, 0, 3, 4), "camera/front/image")
    finally:
        publisher.close()
    assert publisher.socket.closed and publisher.context.closed


def test_endpoint_validation_collision_and_send_backpressure(monkeypatch):
    with pytest.raises(ValueError, match="loopback"):
        BboxPublisher("tcp://0.0.0.0:5558")
    endpoint = free_endpoints()[0]
    first = BboxPublisher(endpoint)
    try:
        with pytest.raises(zmq.ZMQError):
            BboxPublisher(endpoint)

        def full(*args, **kwargs):
            raise zmq.Again()

        monkeypatch.setattr(zmq.Socket, "send_multipart", full)
        assert not first.publish(dict(session="b" * 32, sequence=1, capture_ns=0), None, "camera/front/image")
    finally:
        first.close()
