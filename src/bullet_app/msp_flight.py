"""Application-owned MSP polling and firmware-frame conversion, not physics IO."""
from threading import Event, Lock, Thread
import time

from msp_link import Client, FlightCommands, TcpTransport, MspError
from msp_link.commands import STATUS

STATUS_PERIOD_S = .1
STALE_S = .2
DISARMED = (1500, 1500, 1000, 1500, 1000, 2000) + (1000,) * 12


class MspFlight:
    def __init__(self, client=None):
        self.client = client if client is not None else Client(TcpTransport())
        self.commands = FlightCommands(self.client)
        self.stop, self.lock = Event(), Lock()
        self.status = None
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
                self.status = None
                self.client.rc.clear()

    def _poll(self):
        periods = dict(status=STATUS_PERIOD_S)
        due = dict.fromkeys(periods, 0.)
        schedule_identity = None
        while not self.stop.is_set():
            if not self.client.connected.wait(.05):
                continue
            try:
                with self.lock:
                    episode_generation = self.episode_generation
                if self.generation != self.client.generation:
                    connection_generation = self.client.generation
                    firmware = self.commands.identify()
                    if firmware['variant'] != b'BTFL' or firmware['version'][:3] != bytes((26, 6, 2)):
                        raise ValueError('Expected pinned Betaflight 2026.6.2 firmware')
                    with self.lock:
                        if connection_generation != self.client.generation:
                            continue
                        self.firmware = {key: list(value) for key, value in firmware.items()}
                        self.status = None
                        self.generation = self.client.generation
                connection_generation = self.generation
                identity = (episode_generation, connection_generation)
                if identity != schedule_identity:
                    due = dict.fromkeys(periods, 0.)
                    schedule_identity = identity
                started = time.monotonic()
                ready = [name for name in periods if due[name] <= started]
                if not ready:
                    self.stop.wait(max(0., min(due.values()) - started))
                    continue
                name = min(ready, key=due.get)
                sample = getattr(self.commands, 'read_' + name)()
                finished = time.monotonic()
                # Skip missed slots so slow responses cannot starve other telemetry.
                period = periods[name]
                due[name] = started + (int((finished - started) / period) + 1) * period
                with self.lock:
                    if (episode_generation != self.episode_generation
                            or connection_generation != self.client.generation
                            or not self.client.connected.is_set()):
                        continue
                    setattr(self, name, sample)
                    self.error = None
            except (MspError, TimeoutError, ValueError, OSError) as error:
                with self.lock:
                    self.error = str(error)
                self.stop.wait(.05)

    def snapshot(self):
        with self.lock:
            if not self.client.connected.is_set() or self.generation != self.client.generation:
                return None
            status = self.status
            if status is not None and time.monotonic() - status.received_at > STALE_S:
                status = None
            return None, None, status

    def diagnostics(self, now_s):
        return self.client.request_metrics(STATUS, now_s)

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
