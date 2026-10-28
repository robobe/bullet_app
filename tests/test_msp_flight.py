"""Betaflight MSP scheduler owns periodic telemetry and RC state."""
from threading import Event
from types import SimpleNamespace
import time

from bullet_app.msp_flight import (ALTITUDE_PERIOD_S, ATTITUDE_PERIOD_S,
                                   MspFlight, SAFE_RC_COMMAND, STATUS_PERIOD_S)
from msp_link.commands import STATUS


FIRMWARE = dict(variant=b'BTFL', version=bytes((26, 6, 2)))


def flight_with_client():
    connected = Event()
    connected.set()
    metrics_calls = []
    client = SimpleNamespace(
        connected=connected, generation=1,
        rc=SimpleNamespace(clear=lambda: None,
                           update=lambda channels: setattr(client, 'channels', channels)),
        request_metrics=lambda command, now: (metrics_calls.append(command) or {}))
    return MspFlight(client), client, metrics_calls


def test_scheduler_requests_all_telemetry_and_snapshot_includes_altitude():
    flight, _, metrics_calls = flight_with_client()
    status = SimpleNamespace(armed=True, arming_disabled_flags=0, received_at=time.monotonic())
    attitude = SimpleNamespace(roll_rad=.1, pitch_rad=.2, yaw_rad=.3, received_at=time.monotonic())
    altitude = SimpleNamespace(altitude_m=2., vertical_speed_m_s=.3, received_at=time.monotonic())
    altitude_received = Event()
    calls = []

    def read(name, sample):
        def reader():
            calls.append(name)
            if name == 'altitude':
                altitude_received.set()
            return sample
        return reader

    commands = SimpleNamespace(identify=lambda: FIRMWARE,
                               read_status=read('status', status),
                               read_attitude=read('attitude', attitude),
                               read_altitude=read('altitude', altitude))
    flight.commands = flight.scheduler.commands = commands
    flight.scheduler.start()
    assert altitude_received.wait(1)
    flight.scheduler.close()

    assert calls[:3] == ['status', 'attitude', 'altitude']
    assert flight.snapshot() == (attitude, altitude, status)
    flight.scheduler.attitude = SimpleNamespace(received_at=0.)
    assert flight.snapshot() == (None, altitude, status)
    flight.diagnostics(1.)
    assert metrics_calls == [STATUS]


def test_scheduler_uses_requested_telemetry_periods():
    assert STATUS_PERIOD_S == .1
    assert ATTITUDE_PERIOD_S == .02
    assert ALTITUDE_PERIOD_S == .05

    flight, _, _ = flight_with_client()
    calls = []

    def read(name):
        def reader():
            calls.append(name)
            return SimpleNamespace(received_at=time.monotonic())
        return reader

    commands = SimpleNamespace(identify=lambda: FIRMWARE, read_status=read('status'),
                               read_attitude=read('attitude'), read_altitude=read('altitude'))
    flight.commands = flight.scheduler.commands = commands
    flight.scheduler.start()
    time.sleep(.13)
    flight.scheduler.close()

    assert calls.count('status') >= 2
    assert calls.count('attitude') >= 6
    assert calls.count('altitude') >= 3


def test_scheduler_replaces_commands_and_resets_to_safe_after_disconnect():
    flight, client, _ = flight_with_client()
    flight.scheduler.generation = client.generation

    flight.scheduler._deliver()
    assert client.channels == SAFE_RC_COMMAND

    flight.update([1500, 1500, 1000, 1500, 2000, 2000])
    flight.update([1600, 1500, 1000, 1500, 2000, 2000])
    flight.scheduler._deliver()
    assert client.channels[0] == 1600

    client.connected.clear()
    client.generation += 1
    flight.scheduler._deliver()
    client.connected.set()
    flight.scheduler._deliver()
    assert client.channels == SAFE_RC_COMMAND


def test_snapshot_omits_disconnected_state():
    flight, client, _ = flight_with_client()
    flight.scheduler.generation = client.generation
    flight.scheduler.attitude = SimpleNamespace(received_at=time.monotonic())
    client.connected.clear()

    assert flight.snapshot() is None


def test_episode_change_clears_cached_telemetry():
    flight, client, _ = flight_with_client()
    flight.scheduler.generation = client.generation
    flight.scheduler.attitude = SimpleNamespace(received_at=time.monotonic())
    flight.scheduler.altitude = SimpleNamespace(received_at=time.monotonic())
    flight.scheduler.status = SimpleNamespace(received_at=time.monotonic())

    flight.set_episode('next')

    assert flight.snapshot() == (None, None, None)
