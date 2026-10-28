from dataclasses import replace
from pathlib import Path

from bullet_app.rc import betaflight_throttle, calibrate_betaflight_hover
from los_tracking import NavigatorConfig


PROFILE = Path(__file__).parents[2] / 'los_tracking/examples/bullet_profile.yaml'


def test_calibrated_betaflight_hover_offsets_only_live_throttle():
    config = NavigatorConfig.from_yaml(PROFILE)
    config = replace(config, tuning=replace(config.tuning, betaflight_hover_pwm=1600))
    nominal = betaflight_throttle(config.vehicle.mass_kg * config.vehicle.gravity_m_s2,
                                  config.vehicle.max_thrust_n)

    assert calibrate_betaflight_hover(1000, config) == 1000
    assert calibrate_betaflight_hover(nominal, config) == 1600
