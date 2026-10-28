from pathlib import Path

import pytest

from bullet_app.cli import (AppOptions, CliParser, DEFAULT_BBOX_ENDPOINT, DEFAULT_IMAGE_TOPIC,
                            DEFAULT_MAVLINK_ENDPOINT)


def test_parser_returns_options_without_starting_application():
    options = CliParser().parse(["--transport-config", "transport.json"])

    assert options == AppOptions(Path("transport.json"), DEFAULT_IMAGE_TOPIC, False,
                                 DEFAULT_BBOX_ENDPOINT, None, False, DEFAULT_MAVLINK_ENDPOINT)


def test_parser_keeps_mission_options():
    options = CliParser().parse([
        "--transport-config", "transport.json", "--topic", "camera/rear/image", "--debug",
        "--bbox-endpoint", "tcp://127.0.0.1:6000", "--mission-config", "mission.yaml",
        "--hover-check",
    ])

    assert options == AppOptions(Path("transport.json"), "camera/rear/image", True,
                                 "tcp://127.0.0.1:6000", Path("mission.yaml"), True)


def test_parser_accepts_optional_mavlink_endpoint():
    options = CliParser().parse(["--transport-config", "transport.json",
                                 "--mavlink-endpoint", "udpout:127.0.0.1:14550"])

    assert options.mavlink_endpoint == "udpout:127.0.0.1:14550"


def test_parser_rejects_hover_check_without_mission_config():
    with pytest.raises(Exception, match="--hover-check requires --mission-config"):
        CliParser().parse(["--transport-config", "transport.json", "--hover-check"])


def test_parser_returns_success_without_starting_the_app_for_help(capsys):
    assert CliParser().parse(["--help"]) == 0
    assert "Receive Bullet camera images" in capsys.readouterr().out
