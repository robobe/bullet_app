from math import radians
from pathlib import Path

import pytest

from bullet_app.msp_flight import body_to_world
from bullet_app.rc import encode_demand
from los_tracking import ControlDemand, NavigatorConfig


PROFILE = Path(__file__).resolve().parents[2] / 'los_tracking/examples/bullet_profile.json'


def test_internal_and_betaflight_encoding_are_application_owned():
    config = NavigatorConfig.from_json(PROFILE)
    demand = ControlDemand(radians(10), radians(-5), radians(70), 9.81)
    internal = encode_demand(demand, config, backend='internal')
    betaflight = encode_demand(demand, config, backend='betaflight')
    assert internal.roll == betaflight.roll and internal.pitch == betaflight.pitch
    assert internal.throttle != betaflight.throttle
    assert betaflight.yaw == 1644  # Inverse of the configured Actual-rate curve.
    assert all(1000 <= value <= 2000 for value in betaflight.as_tuple())


def test_betaflight_adapter_clamps_at_tracker_physical_limits():
    config = NavigatorConfig.from_json(PROFILE)
    disarmed = encode_demand(ControlDemand(0, 0, 0, 0), config, backend='betaflight')
    assert disarmed.throttle == 1000
    channels = encode_demand(ControlDemand(10, -10, 100, 1e6), config, backend='betaflight')
    assert all(1000 <= value <= 2000 for value in channels.as_tuple())


def test_msp_frd_angles_convert_to_bullet_body_to_world_axes():
    assert body_to_world(0, 0, 0) == (0, 0, 0, 1)
    roll = body_to_world(radians(90), 0, 0)
    yaw = body_to_world(0, 0, radians(90))
    assert roll == pytest.approx((2**-.5, 0, 0, 2**-.5))
    assert yaw == pytest.approx((0, 0, 2**-.5, 2**-.5))
