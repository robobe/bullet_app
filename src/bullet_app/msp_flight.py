"""Application-owned MSP polling and firmware-frame conversion, not physics IO."""
import heapq
from threading import Event, Lock, Thread
import time

from msp_link import Client, FlightCommands, TcpTransport, MspError
from msp_link.commands import ALTITUDE, ATTITUDE, STATUS
from msp_link.rc import pack_rc

STATUS_PERIOD_S = .1
ATTITUDE_PERIOD_S = .02
ALTITUDE_PERIOD_S = .05
STALE_S = .2
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


class MspScheduler:
    """Schedules MSP work and retains its latest telemetry state."""
    def __init__(self, client, commands):
        self.client = client
        self.commands = commands
        self.stop = Event()
        self.lock = Lock()
        self.command = SAFE_RC_COMMAND
        self.generation = 0
        self.rc_generation = client.generation
        self.identity = None
        self.episode_generation = 0
        self.status = None
        self.attitude = None
        self.altitude = None
        self.firmware = None
        self.error = None
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
            self.rc_generation = self.client.generation
        self.client.rc.clear()

    def set_episode(self, identity):
        with self.lock:
            if identity == self.identity:
                return
            self.identity = identity
            self.episode_generation += 1
            self.status = self.attitude = self.altitude = None
        self.reset()

    def _deliver(self):
        with self.lock:
            if not self.client.connected.is_set() or self.rc_generation != self.client.generation:
                self.command = SAFE_RC_COMMAND
                self.rc_generation = self.client.generation
            command = self.command
        if self.client.connected.is_set():
            self.client.rc.update(command)

    def _identify(self):
        with self.lock:
            known_generation = self.generation
        if known_generation == self.client.generation:
            return
        connection_generation = self.client.generation
        firmware = self.commands.identify()
        if firmware['variant'] != b'BTFL' or firmware['version'][:3] != bytes((26, 6, 2)):
            raise ValueError('Expected pinned Betaflight 2026.6.2 firmware')
        with self.lock:
            if connection_generation != self.client.generation:
                return
            self.firmware = {key: list(value) for key, value in firmware.items()}
            self.status = self.attitude = self.altitude = None
            self.generation = connection_generation

    def _read(self, name):
        self._identify()
        with self.lock:
            episode_generation = self.episode_generation
            connection_generation = self.generation
        sample = getattr(self.commands, 'read_' + name)()
        with self.lock:
            if (episode_generation != self.episode_generation
                    or connection_generation != self.client.generation
                    or not self.client.connected.is_set()):
                return
            setattr(self, name, sample)
            self.error = None

    def _run(self):
        tasks = ((RC_PERIOD_S, 'rc'), (STATUS_PERIOD_S, 'status'),
                 (ATTITUDE_PERIOD_S, 'attitude'), (ALTITUDE_PERIOD_S, 'altitude'))
        due = [(time.monotonic(), index, period, name)
               for index, (period, name) in enumerate(tasks)]
        heapq.heapify(due)
        while not self.stop.is_set():
            deadline, index, period, name = heapq.heappop(due)
            if self.stop.wait(max(0., deadline - time.monotonic())):
                break
            try:
                if name == 'rc':
                    self._deliver()
                elif self.client.connected.is_set():
                    self._read(name)
            except (MspError, TimeoutError, ValueError, OSError) as error:
                with self.lock:
                    self.error = str(error)
                self.stop.wait(ERROR_RETRY_S)
            now = time.monotonic()
            next_deadline = deadline + (int((now - deadline) / period) + 1) * period
            heapq.heappush(due, (next_deadline, index, period, name))

    def snapshot(self):
        with self.lock:
            if not self.client.connected.is_set() or self.generation != self.client.generation:
                return None
            samples = (self.attitude, self.altitude, self.status)
            return tuple(sample if sample is None or time.monotonic() - sample.received_at <= STALE_S else None
                         for sample in samples)

    def close(self):
        self.reset()
        self.stop.set()
        if self.thread is not None:
            self.thread.join(THREAD_JOIN_TIMEOUT_S)


class MspFlight:
    def __init__(self, client=None):
        self.client = client if client is not None else Client(TcpTransport())
        self.commands = FlightCommands(self.client)
        self.scheduler = MspScheduler(self.client, self.commands)

    def start(self):
        self.client.start()
        self.scheduler.start()

    def set_episode(self, identity):
        self.scheduler.set_episode(identity)

    @property
    def firmware(self):
        with self.scheduler.lock:
            return self.scheduler.firmware

    @property
    def error(self):
        with self.scheduler.lock:
            return self.scheduler.error

    def snapshot(self):
        return self.scheduler.snapshot()

    def diagnostics(self, now_s):
        metrics = self.client.request_metrics(STATUS, now_s)
        metrics['attitude'] = self.client.request_metrics(ATTITUDE, now_s)
        metrics['altitude'] = self.client.request_metrics(ALTITUDE, now_s)
        return metrics

    def drain_rc_deliveries(self):
        return self.client.drain_rc_deliveries()

    def update(self, channels):
        if self.client.connected.is_set() and self.scheduler.generation == self.client.generation:
            self.scheduler.update(channels)

    def close(self):
        self.scheduler.close()
        if self.client.connected.is_set():
            try:
                self.commands.send_rc(SAFE_RC_COMMAND)
            except (MspError, TimeoutError):
                pass
        self.client.rc.clear()
        self.client.close()
