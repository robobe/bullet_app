from dataclasses import replace
import json
from math import pi
from types import SimpleNamespace

import pytest
import zmq
from bullet_app.los_telemetry import LosTelemetryPublisher, snapshot
from bullet_msgs.los import LOS_TOPIC, validate_los
from los_tracking import Diagnostics, NavigationResult, RcChannels, State, TrackStage


def mission(now=1000.):
    source = dict(physics_id='physics', episode_id='episode', stream_id='stream',
                  step_index=240, simulation_time_s=1., measurement_us=999_990_000)
    diagnostics = Diagnostics(elevation_rad=pi, azimuth_rad=-pi / 2, bearing_valid=True,
        elevation_rate_rad_s=pi / 4, azimuth_rate_rad_s=-pi / 3, rate_observable=True,
        optical_ttc_s=2., ttc_reason='qualified', ttc_relative_residual=.02)
    result = NavigationResult(1, now, now + .05, State.TRACK, TrackStage.TTC, 'tracking',
                              RcChannels(1500, 1500, 1300, 1500), diagnostics)
    return SimpleNamespace(app_id='app', phase='LOS_APPROACH', reason='tracking',
                           feedback=dict(source=source, state='running', reason='active'), nav_result=result)


class Socket:
    def __init__(self):
        self.messages = []
        self.fail = False

    def send_multipart(self, parts, *, flags):
        assert flags == zmq.NOBLOCK
        if self.fail:
            raise zmq.Again()
        self.messages.append(parts)


def test_degrees_timestamps_and_confidence_without_mutation():
    m = mission()
    original = m.nav_result
    value = snapshot(m, now_s=1000.01, sequence=1)
    assert value['elevation_deg'] == pytest.approx(180.)
    assert value['azimuth_deg'] == pytest.approx(-90.)
    assert value['elevation_rate_deg_s'] == pytest.approx(45.)
    assert value['azimuth_rate_deg_s'] == pytest.approx(-60.)
    assert value['optical_ttc_s'] == 2. and value['ttc_valid']
    assert value['ttc_relative_residual'] == .02
    assert value['generated_us'] == 1_000_000_000
    assert value['source']['measurement_us'] == 999_990_000
    assert value['source']['simulation_time_s'] == 1.
    assert value['navigation_sequence'] == original.sequence
    assert m.nav_result is original and original.diagnostics.elevation_rad == pi
    value['source']['step_index'] = 0
    assert m.feedback['source']['step_index'] == 240


@pytest.mark.parametrize('reason,residual', [('insufficient_history', None), ('not_closing', None), ('noisy_growth', .5), ('stale', .02)])
def test_unqualified_ttc_and_unavailable_bearing(reason, residual):
    m = mission()
    m.nav_result = replace(m.nav_result, diagnostics=Diagnostics(ttc_reason=reason, ttc_relative_residual=residual))
    value = snapshot(m, now_s=1000., sequence=1)
    assert not value['bearing_valid'] and not value['rate_observable']
    assert value['elevation_deg'] is None and value['azimuth_rate_deg_s'] is None
    assert not value['ttc_valid'] and value['optical_ttc_s'] is None
    assert value['ttc_reason'] == reason and value['ttc_relative_residual'] == residual


def test_30_hz_from_100_hz_ticks_and_immediate_state_changes():
    m = mission()
    socket = Socket()
    publisher = LosTelemetryPublisher(socket)
    emitted_at = []
    for index in range(100):
        now = 1000 + index / 100
        m.nav_result = replace(m.nav_result, sequence=index + 1, generated_at_s=now, valid_until_s=now + .05)
        if publisher.publish(m, now_s=now) is not None:
            emitted_at.append(index)
    assert len(emitted_at) == 30
    assert emitted_at[:5] == [0, 4, 7, 10, 14]
    now = 1000.991
    m.nav_result = replace(m.nav_result, state=State.RECOVER, generated_at_s=now, valid_until_s=now + .05)
    assert publisher.publish(m, now_s=now)['navigator_state'] == 'recover'
    assert publisher.publish(m, now_s=now + .001) is None
    for phase, feedback_state in [('COMPLETED', 'stopped'), ('FAULT', 'running'), ('READY', 'stopped')]:
        now += .002
        m.phase, m.feedback['state'] = phase, feedback_state
        value = publisher.publish(m, now_s=now)
        assert value is not None and not value['active']
        assert value['elevation_deg'] is None and value['optical_ttc_s'] is None
        assert value['navigator_state'] == 'recover' and value['navigation_sequence'] is None
        assert publisher.publish(m, now_s=now + .001) is None
    m.nav_result = None
    m.feedback['source'] = dict(m.feedback['source'], episode_id='fresh', step_index=0, simulation_time_s=0.)
    value = publisher.publish(m, now_s=now + .002)
    assert value['source']['episode_id'] == 'fresh' and value['navigator_state'] is None
    assert value['stage'] is None


def test_expiry_nonfinite_and_nonblocking_retry():
    m = mission()
    assert not snapshot(m, now_s=1001., sequence=1)['active']
    m.nav_result = replace(m.nav_result, diagnostics=replace(m.nav_result.diagnostics, elevation_rad=float('nan')))
    with pytest.raises(ValueError):
        snapshot(m, now_s=1000., sequence=1)
    m = mission()
    socket = Socket()
    publisher = LosTelemetryPublisher(socket)
    socket.fail = True
    assert publisher.publish(m, now_s=1000.) is None
    socket.fail = False
    assert publisher.publish(m, now_s=1000.001)['sequence'] == 1
    value = publisher.publish(m, now_s=1000.002, shutdown=True)
    assert not value['active'] and value['reason'] == 'app_shutdown'
    assert publisher.publish(m, now_s=1000.003, shutdown=True) is None
    with pytest.raises(ValueError, match='backwards'):
        publisher.publish(m, now_s=999.)


def test_exact_topic_tcp_and_mcap_readback(tmp_path):
    import socket as net
    import time
    import yaml
    from mcap.reader import make_reader
    from mcap_bagger.config import Config
    from mcap_bagger.recorder import Recording

    ports = []
    for _ in range(3):
        with net.socket() as reservation:
            reservation.bind(('127.0.0.1', 0))
            ports.append(reservation.getsockname()[1])
    transport = dict(zip(('telemetry_endpoint', 'image_endpoint', 'control_endpoint'),
                         [f'tcp://127.0.0.1:{port}' for port in ports]))
    (tmp_path / 'transport.json').write_text(json.dumps(transport))
    with zmq.Context() as context, context.socket(zmq.XPUB) as socket:
        socket.setsockopt(zmq.LINGER, 0)
        port = socket.bind_to_random_port('tcp://127.0.0.1')
        path = tmp_path / 'recorder.yaml'
        path.write_text(yaml.safe_dump(dict(project='los', output_dir='recordings', transport_config='transport.json',
            topics=[dict(key=LOS_TOPIC.decode(), input='json', endpoint=f'tcp://127.0.0.1:{port}',
                         schema='bullet_msgs:los_telemetry.json', timestamp_source='receive')])))
        recording = Recording(Config.load(path), on_error=lambda message: None)
        recording.start()
        expected = []
        try:
            assert socket.poll(3000)
            assert socket.recv() == b'\x01' + LOS_TOPIC
            publisher = LosTelemetryPublisher(socket)
            m = mission()
            socket.send_multipart([LOS_TOPIC + b'/extra', b'{"wrong":true}'])
            value = publisher.publish(m, now_s=1000.)
            expected.append(value)
            m.phase, m.reason, m.feedback['state'] = 'COMPLETED', 'target_hit', 'stopped'
            expected.append(publisher.publish(m, now_s=1000.001))
            deadline = time.monotonic() + 3
            while recording.counts[LOS_TOPIC.decode()]['written'] < 2:
                assert time.monotonic() < deadline, recording.error
                time.sleep(.005)
        finally:
            recording.close()
        assert recording.error is None
        with recording.path.open('rb') as stream:
            messages = list(make_reader(stream).iter_messages())
        assert [json.loads(message.data) for _, _, message in messages] == expected
        assert all(channel.topic == LOS_TOPIC.decode() for _, channel, _ in messages)
        assert expected[-1]['active'] is False
