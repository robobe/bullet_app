"""Application-owned MSP polling and firmware-frame conversion, not physics IO."""
from math import cos, sin
from threading import Event, Lock, Thread
import time

from msp_link import Client, FlightCommands, TcpTransport, MspError
from los_tracking import AttitudeSample, BarometerSample

ATTITUDE_PERIOD_S = .01
ALTITUDE_PERIOD_S = .02
STATUS_PERIOD_S = .1
STALE_S = .2
DISARMED = (1500, 1500, 1000, 1500, 1000, 2000) + (1000,) * 12


def body_to_world(roll, pitch, yaw):
    # The pinned sensor-only SITL reports the same body-to-world Euler signs as Bullet.
    cr, cp, cy = cos(roll / 2), cos(pitch / 2), cos(yaw / 2)
    sr, sp, sy = sin(roll / 2), sin(pitch / 2), sin(yaw / 2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


class MspFlight:
    def __init__(self, client=None):
        self.client = client if client is not None else Client(TcpTransport())
        self.commands = FlightCommands(self.client)
        self.stop, self.lock = Event(), Lock()
        self.attitude = self.altitude = self.status = None
        self.identity = None
        self.episode_generation = 0
        self.generation = 0
        self.firmware = None
        self.error = None
        self.thread = None

    def start(self):
        self.client.start()
        self.thread = Thread(target=self._poll, name='msp-feedback', daemon=True)
        self.thread.start()

    def set_episode(self, identity):
        with self.lock:
            if identity != self.identity:
                self.identity = identity
                self.episode_generation += 1
                self.attitude = self.altitude = self.status = None
                self.client.rc.clear()

    def _poll(self):
        next_altitude = next_status = 0.
        while not self.stop.is_set():
            if not self.client.connected.wait(.05):
                continue
            try:
                with self.lock:
                    episode_generation = self.episode_generation
                if self.generation != self.client.generation:
                    firmware = self.commands.identify()
                    if firmware['variant'] != b'BTFL' or firmware['version'][:3] != bytes((26, 6, 2)):
                        raise ValueError('Expected pinned Betaflight 2026.6.2 firmware')
                    with self.lock:
                        self.firmware = {key: list(value) for key, value in firmware.items()}
                        self.attitude = self.altitude = self.status = None
                        self.generation = self.client.generation
                started = time.monotonic()
                attitude = self.commands.read_attitude()
                altitude = self.commands.read_altitude() if started >= next_altitude else None
                status = self.commands.read_status() if started >= next_status else None
                with self.lock:
                    if episode_generation != self.episode_generation:
                        continue
                    self.attitude = attitude
                    if altitude is not None:
                        self.altitude = altitude
                        next_altitude = started + ALTITUDE_PERIOD_S
                    if status is not None:
                        self.status = status
                        next_status = started + STATUS_PERIOD_S
                    self.error = None
                self.stop.wait(max(0., ATTITUDE_PERIOD_S - (time.monotonic() - started)))
            except (MspError, TimeoutError, ValueError, OSError) as error:
                with self.lock:
                    self.error = str(error)
                self.stop.wait(.05)

    def snapshot(self):
        with self.lock:
            if not self.client.connected.is_set() or self.generation != self.client.generation:
                return None
            return self.attitude, self.altitude, self.status

    def update(self, channels):
        if self.client.connected.is_set() and self.generation == self.client.generation:
            self.client.rc.update(channels)

    def close(self):
        self.client.rc.clear()
        if self.client.connected.is_set():
            try:
                self.commands.send_rc(DISARMED)
            except (MspError, TimeoutError):
                pass
        self.client.rc.clear()
        self.stop.set()
        if self.thread is not None:
            self.thread.join(2)
        self.client.close()
