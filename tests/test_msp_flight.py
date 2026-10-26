"""Betaflight MSP stays responsible for status and RC, not navigation sensors."""
from threading import Event, Thread
from types import SimpleNamespace
import time

from bullet_app.msp_flight import MspFlight
from msp_link.commands import STATUS


def test_poller_requests_status_only_and_keeps_rc_available():
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
    assert client.channels[4] == 2000
