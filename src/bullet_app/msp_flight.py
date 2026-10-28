"""Application-owned MSP polling and firmware-frame conversion, not physics IO."""
import heapq
from threading import Event, Lock, Thread
import time

from msp_link import Client, FlightCommands, TcpTransport, MspError
from msp_link.commands import STATUS
from msp_link.rc import pack_rc

STATUS_PERIOD_S = .1
ATTITUDE_PERIOD_S = .04
STALE_S = .2
CONNECTION_WAIT_S = .05
ERROR_RETRY_S = .05
THREAD_JOIN_TIMEOUT_S = 2
RC_MIN_PWM = 1000
RC_NEUTRAL_PWM = 1500
RC_MAX_PWM = 2000
RC_CHANNEL_COUNT = 18
RC_AETR_CHANNEL_COUNT = 4
RC_ARM_MODE_CHANNEL_COUNT = 2
RC_RATE_HZ = 50
RC_PERIOD_S = 1 / RC_RATE_HZ
SAFE_RC_COMMAND = (
    RC_NEUTRAL_PWM, RC_NEUTRAL_PWM, RC_MIN_PWM, RC_NEUTRAL_PWM,
    RC_MIN_PWM, RC_MAX_PWM,
) + (RC_MIN_PWM,) * (RC_CHANNEL_COUNT - RC_AETR_CHANNEL_COUNT - RC_ARM_MODE_CHANNEL_COUNT)


class RcScheduler:
    """Streams the latest accepted RC command without mission-loop timing."""
    def __init__(self, client):
        self.client = client
        self.stop = Event()
        self.lock = Lock()
        self.command = SAFE_RC_COMMAND
        self.generation = client.generation
        self.thread = None

    def start(self):
        self.thread = Thread(target=self._run, name='msp-rc', daemon=True)
        self.thread.start()

    def update(self, channels):
        command = tuple(channels)
        pack_rc(command)
        with self.lock:
            self.command = command

    def reset(self):
        with self.lock:
            self.command = SAFE_RC_COMMAND
            self.generation = self.client.generation
        self.client.rc.clear()

    def _deliver(self):
        with self.lock:
            if not self.client.connected.is_set() or self.generation != self.client.generation:
                self.command = SAFE_RC_COMMAND
                self.generation = self.client.generation
            command = self.command
        if self.client.connected.is_set():
            self.client.rc.update(command)

    def _run(self):
        due = [(time.monotonic(), 'rc')]
        while not self.stop.is_set():
            deadline, _ = heapq.heappop(due)
            if self.stop.wait(max(0., deadline - time.monotonic())):
                break
            self._deliver()
            now = time.monotonic()
            next_deadline = deadline + (int((now - deadline) / RC_PERIOD_S) + 1) * RC_PERIOD_S
            heapq.heappush(due, (next_deadline, 'rc'))

    def close(self):
        self.reset()
        self.stop.set()
        if self.thread is not None:
            self.thread.join(THREAD_JOIN_TIMEOUT_S)


class MspFlight:
    def __init__(self, client=None):
        self.client = client if client is not None else Client(TcpTransport())
        self.commands = FlightCommands(self.client)
        self.stop, self.lock = Event(), Lock()
        self.status = None
        self.attitude = None
        self.identity = None
        self.episode_generation = 0
        self.generation = 0
        self.firmware = None
        self.error = None
        self.thread = None
        self.rc_scheduler = RcScheduler(self.client)

    def start(self):
        self.client.start()
        self.thread = Thread(target=self._poll, name='msp-feedback', daemon=True)
        self.thread.start()
        self.rc_scheduler.start()

    def set_episode(self, identity):
        with self.lock:
            if identity != self.identity:
                self.identity = identity
                self.episode_generation += 1
                self.status = None
                self.attitude = None
                self.rc_scheduler.reset()

    def _poll(self):
        periods = dict(status=STATUS_PERIOD_S, attitude=ATTITUDE_PERIOD_S)
        due = dict.fromkeys(periods, 0.)
        schedule_identity = None
        while not self.stop.is_set():
            if not self.client.connected.wait(CONNECTION_WAIT_S):
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
                self.stop.wait(ERROR_RETRY_S)

    def snapshot(self):
        with self.lock:
            if not self.client.connected.is_set() or self.generation != self.client.generation:
                return None
            status = self.status
            attitude = self.attitude
            if attitude is not None and time.monotonic() - attitude.received_at > STALE_S:
                attitude = None
            if status is not None and time.monotonic() - status.received_at > STALE_S:
                status = None
            return attitude, None, status

    def diagnostics(self, now_s):
        return self.client.request_metrics(STATUS, now_s)

    def drain_rc_deliveries(self):
        return self.client.drain_rc_deliveries()

    def update(self, channels):
        if self.client.connected.is_set() and self.generation == self.client.generation:
            self.rc_scheduler.update(channels)

    def close(self):
        self.rc_scheduler.close()
        if self.client.connected.is_set():
            try:
                self.commands.send_rc(SAFE_RC_COMMAND)
            except (MspError, TimeoutError):
                pass
        self.client.rc.clear()
        self.stop.set()
        if self.thread is not None:
            self.thread.join(THREAD_JOIN_TIMEOUT_S)
        self.client.close()
