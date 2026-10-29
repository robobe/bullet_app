import numpy as np
import pytest

import bullet_app.main as live_los
import bullet_app.application as application
import bullet_app.runtime as runtime


@pytest.mark.parametrize("debug", [False, True])
def test_example_processes_frames_and_cleans_up(monkeypatch, debug):
    image = np.zeros((80, 100, 3), dtype=np.uint8)
    image[40:60, 30:50] = (0, 0, 255)
    frames = iter([None, ({"sequence": 1, "session": "a" * 32, "capture_ns": 1}, image),
                   ({"sequence": 2, "session": "a" * 32, "capture_ns": 2}, np.zeros_like(image))])
    class Receiver:
        closed = False

        def __init__(self, endpoint, topic):
            assert endpoint == "tcp://127.0.0.1:5556"
            assert topic == "camera/front/image"

        def poll(self):
            try:
                return next(frames)
            except StopIteration:
                raise KeyboardInterrupt

        def close(self):
            Receiver.closed = True

    class Publisher:
        closed = False
        results = []

        def __init__(self, endpoint):
            assert endpoint == "tcp://127.0.0.1:5558"

        def publish(self, metadata, bbox, topic):
            Publisher.results.append(bbox)

        def close(self):
            Publisher.closed = True

    monkeypatch.setattr(runtime, "BboxPublisher", Publisher)
    monkeypatch.setattr(runtime, "ImageReceiver", Receiver)
    monkeypatch.setattr(application, "load_transport", lambda path: {"image_endpoint": "tcp://127.0.0.1:5556"})
    monkeypatch.setattr(runtime.time, "sleep", lambda seconds: None)
    args = ["--transport-config", "unused.json"] + (["--debug"] if debug else [])
    assert live_los.main(args) == 0
    assert Receiver.closed and Publisher.closed
    assert Publisher.results == [(30, 40, 20, 20), None]


def test_example_reports_transport_error(monkeypatch, capsys):
    def fail(path):
        raise ValueError("invalid transport")
    monkeypatch.setattr(application, "load_transport", fail)
    assert live_los.main(["--transport-config", "unused.json"]) == 1
    assert "invalid transport" in capsys.readouterr().out


def test_hover_check_requires_mission_config(capsys):
    assert live_los.main(["--transport-config", "unused.json", "--hover-check"]) == 2
    assert "--hover-check requires --mission-config" in capsys.readouterr().err
