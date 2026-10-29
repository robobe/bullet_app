import pytest

from bullet_app.controller import TakeoffController


def test_aggressive_takeoff_reaches_four_meters_per_second():
    controller = TakeoffController(1.5, 0.25, 4.0, 8.0)
    controller.update(now_s=0.0, target_altitude_m=20.0, altitude_m=0.0)
    output = None
    for step in range(1, 6):
        output = controller.update(now_s=step / 10, target_altitude_m=20.0, altitude_m=0.0)
    assert output.reference_m_s == pytest.approx(4.0)
    assert output.saturated


def test_takeoff_integral_does_not_wind_up_at_climb_limit():
    controller = TakeoffController(1.5, 0.25, 4.0, 8.0)
    controller.update(now_s=0.0, target_altitude_m=20.0, altitude_m=0.0)
    first = controller.update(now_s=0.1, target_altitude_m=20.0, altitude_m=0.0)
    second = controller.update(now_s=0.2, target_altitude_m=20.0, altitude_m=0.0)
    assert second.integral_m_s == pytest.approx(first.integral_m_s)


def test_takeoff_reset_clears_reference_and_integral():
    controller = TakeoffController(1.5, 0.25, 4.0, 8.0)
    controller.update(now_s=0.0, target_altitude_m=20.0, altitude_m=0.0)
    controller.update(now_s=0.1, target_altitude_m=20.0, altitude_m=0.0)
    controller.reset()
    assert controller.reference_m_s == 0.0
    assert controller.integral_m_s == 0.0
    assert controller.last_time_s is None
