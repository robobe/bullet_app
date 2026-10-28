"""Application-owned conversion from physical demands to firmware RC sticks."""
from dataclasses import dataclass
from math import degrees, pi, sqrt
from los_tracking.geometry import clamp, combined_tilt
from los_tracking.types import finite

RC_CENTRE = 1500
RC_HALF_RANGE = 500
RC_MIN = 1000
RC_SPAN = 1000
NUMERICAL_TOLERANCE = 1e-12
ANGLE_SCALE_RAD = pi / 4
YAW_RATE_SCALE_RAD_S = 2.
MIN_CHECK = 1050
IDLE_FRACTION = .055

@dataclass(frozen=True)
class RcChannels:
    roll: int
    pitch: int
    throttle: int
    yaw: int
    def as_tuple(self):
        return self.roll, self.pitch, self.throttle, self.yaw

def encode_channels(roll, pitch, thrust, yaw_rate, config, *, backend="internal", angle_scale=ANGLE_SCALE_RAD, yaw_scale=YAW_RATE_SCALE_RAD_S):
    """Round like Bullet's RcCodec, then retract quantization across angle limits.

    Continuous references obey slew exactly. Integer PWM has a one-count
    resolution error; it cannot express arbitrary sub-count motion per tick.
    """
    vehicle, tuning = config.vehicle, config.tuning
    scale = angle_scale
    roll_channel = round(RC_CENTRE + RC_HALF_RANGE * roll / scale)
    pitch_channel = round(RC_CENTRE + RC_HALF_RANGE * pitch / scale)
    decode = lambda channel: (channel - RC_CENTRE) / RC_HALF_RANGE * scale
    while abs(decode(roll_channel)) > min(tuning.roll_limit_rad, tuning.combined_tilt_limit_rad) + NUMERICAL_TOLERANCE:
        roll_channel -= 1 if roll_channel > RC_CENTRE else -1
    while (abs(decode(pitch_channel)) > tuning.pitch_limit_rad + NUMERICAL_TOLERANCE
           or combined_tilt(decode(roll_channel), decode(pitch_channel)) > tuning.combined_tilt_limit_rad + NUMERICAL_TOLERANCE):
        pitch_channel -= 1 if pitch_channel > RC_CENTRE else -1
    yaw_channel = round(RC_CENTRE + RC_HALF_RANGE * yaw_rate / yaw_scale)
    while abs((yaw_channel - RC_CENTRE) / RC_HALF_RANGE * yaw_scale) > tuning.yaw_rate_limit_rad_s + NUMERICAL_TOLERANCE:
        yaw_channel -= 1 if yaw_channel > RC_CENTRE else -1
    throttle = round(RC_MIN + RC_SPAN * clamp(thrust / vehicle.max_thrust_n, 0.0, 1.0))
    channels = RcChannels(roll_channel, pitch_channel, throttle, yaw_channel)
    return channels, (decode(roll_channel), decode(pitch_channel))


def encode_demand(demand, config, *, backend='internal'):
    channels, _ = encode_channels(demand.roll_rad, demand.pitch_rad, demand.thrust_n,
                                 demand.yaw_rate_rad_s, config)
    if backend == 'internal':
        return channels
    if backend != 'betaflight':
        raise ValueError('Unsupported RC backend')
    # Actual rates: center sensitivity 70 deg/s, maximum 670 deg/s, zero expo.
    requested = degrees(clamp(abs(demand.yaw_rate_rad_s), 0., config.tuning.yaw_rate_limit_rad_s))
    stick = (-70 + sqrt(70**2 + 2400 * requested)) / 1200
    yaw = round(RC_CENTRE + RC_HALF_RANGE * (stick if demand.yaw_rate_rad_s >= 0 else -stick))
    throttle = betaflight_throttle(demand.thrust_n, config.vehicle.max_thrust_n)
    return RcChannels(channels.roll, channels.pitch, throttle, yaw)


def betaflight_throttle(thrust_n, max_thrust_n):
    fraction = sqrt(clamp(thrust_n / max_thrust_n, 0., 1.))
    stick_throttle = clamp((fraction - IDLE_FRACTION) / (1 - IDLE_FRACTION), 0., 1.)
    return RC_MIN if thrust_n == 0 else round(MIN_CHECK + (2000 - MIN_CHECK) * stick_throttle)


def calibrate_betaflight_hover(throttle, config):
    """Offset Betaflight throttle around its calibrated level-hover point."""
    if throttle == RC_MIN:
        return RC_MIN
    hover_pwm = config.tuning.betaflight_hover_pwm
    if hover_pwm == 0:
        return throttle
    nominal_hover_pwm = betaflight_throttle(config.vehicle.mass_kg * config.vehicle.gravity_m_s2,
                                             config.vehicle.max_thrust_n)
    return round(clamp(throttle + hover_pwm - nominal_hover_pwm, MIN_CHECK, 2000))
