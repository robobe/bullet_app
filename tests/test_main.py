import numpy as np
import pytest

import bullet_app.main as live_los


def test_debug_drawing_preserves_input_and_clears_bbox():
    image = np.zeros((80, 100, 3), dtype=np.uint8)
    display = live_los.draw_debug(image, (30, 40, 20, 20), "Live")
    assert display[40, 30].tolist() == [0, 255, 255]
    assert not image.any()
    cleared = live_los.draw_debug(image, None, "Live")
    assert not cleared[35:].any()


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

    monkeypatch.setattr(live_los, "BboxPublisher", Publisher)
    monkeypatch.setattr(live_los, "ImageReceiver", Receiver)
    monkeypatch.setattr(live_los, "load_transport", lambda path: {"image_endpoint": "tcp://127.0.0.1:5556"})
    monkeypatch.setattr(live_los.time, "sleep", lambda seconds: None)
    calls = []

    def gui(name, result=None):
        def call(*args):
            assert debug, f"unexpected GUI call: {name}"
            calls.append(name)
            if name == "imshow":
                calls.append(args[1].copy())
            return result
        return call

    for name, result in [("namedWindow", None), ("imshow", None), ("waitKey", -1),
                         ("getWindowProperty", 1), ("destroyAllWindows", None)]:
        monkeypatch.setattr(live_los.cv2, name, gui(name, result))
    args = ["--transport-config", "unused.json"] + (["--debug"] if debug else [])
    assert live_los.main(args) == 0
    assert Receiver.closed and Publisher.closed
    assert Publisher.results == [(30, 40, 20, 20), None]
    if debug:
        displays = [item for item in calls if isinstance(item, np.ndarray)]
        assert len(displays) == 3
        assert displays[1][40, 30].tolist() == [0, 255, 255]
        assert not displays[2][35:].any()
        assert calls[-1] == "destroyAllWindows"
    else:
        assert calls == []


def test_example_reports_transport_error(monkeypatch, capsys):
    def fail(path):
        raise ValueError("invalid transport")
    monkeypatch.setattr(live_los, "load_transport", fail)
    assert live_los.main(["--transport-config", "unused.json"]) == 1
    assert "invalid transport" in capsys.readouterr().out
