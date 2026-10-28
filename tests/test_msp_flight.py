"""Betaflight MSP stays responsible for status and RC, not navigation sensors."""
from threading import Event, Thread
from types import SimpleNamespace
import time

from bullet_app.msp_flight import MspFlight, SAFE_RC_COMMAND
from msp_link.commands import STATUS


def test_poller_requests_status_and_keeps_rc_available():
    connected = Event()
    connected.set()
    metrics_calls = []
    client = SimpleNamespace(connected=connected, generation=1,
                             rc=SimpleNamespace(clear=lambda: None, update=lambda channels: setattr(client, 'channels', channels)),
                             request_metrics=lambda command, now: (metrics_calls.append(command) or {}))
    flight = MspFlight(client)
    flight.generation = 1
    status = SimpleNamespace(armed=True, arming_disabled_flags=0, received_at=time.monotonic())
    calls = []

    def read_status():
        calls.append('status')
        flight.stop.set()
        return status

    flight.commands = SimpleNamespace(read_status=read_status,
                                      read_attitude=lambda: calls.append('attitude'),
                                      read_altitude=lambda: calls.append('altitude'))
    thread = Thread(target=flight._poll)
    thread.start()
    thread.join(1)

    assert not thread.is_alive()
    assert calls == ['status']
    assert flight.snapshot() == (None, None, status)
    flight.status = SimpleNamespace(armed=True, arming_disabled_flags=0, received_at=0.)
    assert flight.snapshot() == (None, None, None)
    flight.diagnostics(1.)
    assert metrics_calls == [STATUS]
    flight.update([1500, 1500, 1000, 1500, 2000, 2000])
    flight.rc_scheduler._deliver()
    assert client.channels[4] == 2000


def test_rc_scheduler_replaces_commands_and_resets_to_safe_after_disconnect():
    connected = Event()
    connected.set()
    client = SimpleNamespace(connected=connected, generation=1,
                             rc=SimpleNamespace(clear=lambda: None,
                                                update=lambda channels: setattr(client, 'channels', channels)))
    flight = MspFlight(client)
    flight.generation = 1

    flight.rc_scheduler._deliver()
    assert client.channels == SAFE_RC_COMMAND

    flight.update([1500, 1500, 1000, 1500, 2000, 2000])
    flight.update([1600, 1500, 1000, 1500, 2000, 2000])
    flight.rc_scheduler._deliver()
    assert client.channels[0] == 1600

    connected.clear()
    client.generation += 1
    flight.rc_scheduler._deliver()
    connected.set()
    flight.rc_scheduler._deliver()
    assert client.channels == SAFE_RC_COMMAND


def test_poller_requests_attitude_and_snapshot_includes_it():
    connected = Event()
    connected.set()
    client = SimpleNamespace(connected=connected, generation=1,
                             rc=SimpleNamespace(clear=lambda: None, update=lambda channels: None))
    flight = MspFlight(client)
    flight.generation = 1
    status = SimpleNamespace(armed=True, arming_disabled_flags=0, received_at=time.monotonic())
    attitude = SimpleNamespace(roll_rad=.1, pitch_rad=.2, yaw_rad=.3, received_at=time.monotonic())
    calls = []

    def read_status():
        calls.append('status')
        return status

    def read_attitude():
        calls.append('attitude')
        flight.stop.set()
        return attitude

    flight.commands = SimpleNamespace(read_status=read_status, read_attitude=read_attitude)
    thread = Thread(target=flight._poll)
    thread.start()
    thread.join(1)

    assert not thread.is_alive()
    assert calls == ['status', 'attitude']
    assert flight.snapshot() == (attitude, None, status)
