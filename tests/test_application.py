from pathlib import Path

import bullet_app.application as application
from bullet_app.cli import AppOptions


def test_application_selects_detector_runtime(monkeypatch):
    options = AppOptions(Path("transport.json"), "camera/front/image", False,
                         "tcp://127.0.0.1:5558", None, False)
    calls = []

    class Detector:
        def __init__(self, received_options, transport):
            calls.append((received_options, transport))

        def run(self):
            return 7

    monkeypatch.setattr(application, "load_transport", lambda path: {"image_endpoint": "tcp://127.0.0.1:5556"})
    monkeypatch.setattr(application, "DetectorRuntime", Detector)

    assert application.Application(options).run() == 7
    assert calls == [(options, {"image_endpoint": "tcp://127.0.0.1:5556"})]


def test_application_selects_mission_runner(monkeypatch):
    options = AppOptions(Path("transport.json"), "camera/front/image", False,
                         "tcp://127.0.0.1:5558", Path("mission.yaml"), False)

    class Mission:
        def __init__(self, received_options, transport):
            assert received_options is options
            assert transport == {"image_endpoint": "tcp://127.0.0.1:5556"}

        def run(self):
            return 9

    monkeypatch.setattr(application, "load_transport", lambda path: {"image_endpoint": "tcp://127.0.0.1:5556"})
    monkeypatch.setattr(application, "MissionRunner", Mission)

    assert application.Application(options).run() == 9
