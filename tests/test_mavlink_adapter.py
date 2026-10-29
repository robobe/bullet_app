from math import isnan, pi, sin
from threading import Event
from types import SimpleNamespace

import pytest

from bullet_app.blackboard import RuntimeBlackboard
from bullet_app.mavlink_adapter import FEEDBACK_STALE_S, MavlinkAdapter, mavlink_attitude
from bullet_app.runtime import MissionRuntime


class Mav:
    def __init__(self):
        self.heartbeats = []
        self.attitudes = []
        self.altitudes = []
        self.statuses = []
        self.acks = []

    def heartbeat_send(self, *args):
        self.heartbeats.append(args)

    def attitude_send(self, *args):
        self.attitudes.append(args)

    def altitude_send(self, *args):
        self.altitudes.append(args)

    def statustext_send(self, *args):
        self.statuses.append(args)

    def command_ack_send(self, *args):
        self.acks.append(args)


class Connection:
    def __init__(self, messages=()):
        self.mav = Mav()
        self.closed = False
        self.messages = list(messages)

    def recv_match(self, **kwargs):
        return self.messages.pop(0) if self.messages else None

    def close(self):
        self.closed = True


def packet(*, time_us=1_000_000, altitude=3., orientation=(0., 0., 0., 1.), armed=True,
           state='running'):
    return dict(source=dict(measurement_us=time_us), barometer_altitude_m=altitude,
                orientation_xyzw=list(orientation), armed=armed, state=state)


def adapter(board):
    connection = Connection()
    result = MavlinkAdapter('udpout:127.0.0.1:14550', board, Event(),
                            connection_factory=lambda *args, **kwargs: connection)
    result.open()
    return result, connection


def test_adapter_uses_latest_blackboard_feedback_for_standard_messages():
    board = RuntimeBlackboard()
    value = packet(orientation=(0., 0., sin(pi / 4), sin(pi / 4)))
    value['orientation_xyzw'][3] = (1 - value['orientation_xyzw'][2] ** 2) ** .5
    board.write_feedback(value, 10.)
    publisher, connection = adapter(board)

    feedback = publisher._fresh_feedback(10.)
    publisher._send_heartbeat(feedback)
    publisher._send_telemetry(feedback.value)

    assert len(connection.mav.heartbeats) == 1
    assert connection.mav.heartbeats[0][2] != 0
    _, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed = connection.mav.attitudes[0]
    assert roll == pytest.approx(0.) and pitch == pytest.approx(0.)
    assert yaw == pytest.approx(-pi / 2)
    assert all(isnan(rate) for rate in (rollspeed, pitchspeed, yawspeed))
    assert connection.mav.altitudes[0][0] == 1_000_000
    assert connection.mav.altitudes[0][3] == 3.
    assert all(isnan(value) for index, value in enumerate(connection.mav.altitudes[0]) if index != 0 and index != 3)


def test_adapter_omits_stale_feedback_and_missing_altitude():
    board = RuntimeBlackboard()
    board.write_feedback(packet(altitude=None), 10.)
    publisher, connection = adapter(board)

    feedback = publisher._fresh_feedback(10.)
    publisher._send_telemetry(feedback.value)

    assert len(connection.mav.attitudes) == 1
    assert connection.mav.altitudes == []
    assert publisher._fresh_feedback(10. + FEEDBACK_STALE_S + .001) is None


def test_adapter_sends_indexed_status_with_fault_severity():
    board = RuntimeBlackboard()
    publisher, connection = adapter(board)
    status = {'phase': 'FAULT', 'reason': 'search_timeout', 'diagnostics': {}}
    publisher._send_status(status)
    assert connection.mav.statuses
    severity, text = connection.mav.statuses[0]
    assert severity == 3
    assert text.startswith(b'[')


def test_adapter_queues_valid_profile_reload_command():
    class Command:
        command = 31010
        target_system = 1
        target_component = 1
        param1 = 1.0
    board = RuntimeBlackboard()
    telemetry = Connection()
    control = Connection([Command()])
    publisher = MavlinkAdapter('telemetry', board, Event(), control_endpoint='control',
                               connection_factory=lambda endpoint, **kwargs: telemetry if endpoint == 'telemetry' else control)
    publisher.open()
    publisher._poll_control()
    assert publisher.take_reload_request()
    assert control.mav.acks == []


def test_mavlink_attitude_converts_flu_z_up_to_frd_ned():
    assert mavlink_attitude((0., 0., 0., 1.)) == pytest.approx((0., 0., 0.))
    assert mavlink_attitude((0., 0., sin(pi / 4), sin(pi / 4))) == pytest.approx((0., 0., -pi / 2))


def test_adapter_open_failure_is_reported_to_startup():
    def fail(*args, **kwargs):
        raise OSError('invalid endpoint')

    board = RuntimeBlackboard()
    publisher = MavlinkAdapter('invalid', board, Event(), connection_factory=fail)

    with pytest.raises(OSError, match='invalid endpoint'):
        publisher.open()


def test_adapter_closes_its_connection():
    publisher, connection = adapter(RuntimeBlackboard())

    publisher.close()

    assert connection.closed


def test_runtime_isolates_post_start_mavlink_failure():
    def fail():
        raise RuntimeError('disconnected')

    runtime = MissionRuntime({}, None, bbox_endpoint='tcp://127.0.0.1:5558', image_topic='camera/front/image')
    runtime.mavlink = SimpleNamespace(run=fail)

    runtime._run_mavlink()

    assert not runtime.stop.is_set()
