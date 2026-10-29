"""Aggressive altitude-reference control used only during takeoff."""
from dataclasses import dataclass
from math import isfinite

MAX_CONTROL_STEP_S = 0.1
TIME_EPSILON_S = 1e-9


@dataclass(frozen=True)
class TakeoffOutput:
    reference_m_s: float
    altitude_error_m: float
    integral_m_s: float
    saturated: bool


class TakeoffController:
    """Bounded PI controller that produces a vertical-speed reference."""

    def __init__(self, kp_s_inv: float, ki_s2_inv: float, max_climb_m_s: float,
                 slew_m_s2: float):
        values = (kp_s_inv, ki_s2_inv, max_climb_m_s, slew_m_s2)
        if any(not isfinite(value) or value < 0 for value in values) or max_climb_m_s <= 0 or slew_m_s2 <= 0:
            raise ValueError('invalid takeoff controller tuning')
        self.kp_s_inv = kp_s_inv
        self.ki_s2_inv = ki_s2_inv
        self.max_climb_m_s = max_climb_m_s
        self.slew_m_s2 = slew_m_s2
        self.reset()

    def reset(self):
        self.integral_m_s = 0.0
        self.reference_m_s = 0.0
        self.last_time_s = None

    def update(self, *, now_s: float, target_altitude_m: float, altitude_m: float) -> TakeoffOutput:
        if not all(isfinite(value) for value in (now_s, target_altitude_m, altitude_m)):
            raise ValueError('takeoff controller inputs must be finite')
        dt = 0.0 if self.last_time_s is None else now_s - self.last_time_s
        if dt < 0 or dt > MAX_CONTROL_STEP_S + TIME_EPSILON_S:
            raise ValueError('takeoff controller time step out of range')
        self.last_time_s = now_s
        error = target_altitude_m - altitude_m
        candidate_integral = self.integral_m_s + error * dt
        raw = self.kp_s_inv * error + self.ki_s2_inv * candidate_integral
        limited = max(-self.max_climb_m_s, min(self.max_climb_m_s, raw))
        saturated = limited != raw
        if saturated and ((raw > self.max_climb_m_s and error > 0) or (raw < -self.max_climb_m_s and error < 0)):
            candidate_integral = self.integral_m_s
            raw = self.kp_s_inv * error + self.ki_s2_inv * candidate_integral
            limited = max(-self.max_climb_m_s, min(self.max_climb_m_s, raw))
        self.integral_m_s = candidate_integral
        self.reference_m_s += max(-self.slew_m_s2 * dt, min(self.slew_m_s2 * dt, limited - self.reference_m_s))
        return TakeoffOutput(self.reference_m_s, error, self.integral_m_s, saturated)
