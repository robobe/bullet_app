"""Dedicated mission and vision threads; OpenCV UI stays on the main thread."""
import json
from dataclasses import asdict
from pathlib import Path
import traceback
from bullet_msgs.provenance import revisions
from threading import Event, Lock, Thread
import time

import cv2
import zmq
from bullet_msgs.flight import (CONFIG_TOPIC, FEEDBACK_TOPIC, RC_TOPIC, MISSION_TOPIC,
                               MAX_FLIGHT_BYTES, decode, encode, validate_config, validate_feedback, validate_rc)
from bullet_msgs.control import STATE_TOPIC, validate_state
from bullet_msgs.scenario import END_TOPIC, validate_end
from los_tracking import NavigatorConfig
from .detector import detect_red_box
from .mission import Mission, COMMAND_PERIOD_S
from .los_telemetry import LosTelemetryPublisher
from .publisher import BboxPublisher
from .receiver import ImageReceiver


MAX_FEEDBACK_BATCH = 64
FEEDBACK_HWM = 64
THREAD_JOIN_TIMEOUT_S = 3
CONFIG_TIMEOUT_S = 3


def format_msp_status_before_arm(status):
    if status is None:
        return 'Betaflight MSP status before arm: unavailable'
    return ('Betaflight MSP status before arm: armed={}, mode_flags=0x{:08x}, '
            'arming_disabled_flags=0x{:08x}'.format(
                status.armed, status.mode_flags, status.arming_disabled_flags))


def msp_status_from_snapshot(snapshot):
    return None if snapshot is None else snapshot[2]


class MissionRuntime:
    def __init__(self, transport, config, *, bbox_endpoint, image_topic, hover_check=False):
        self.transport, self.config = transport, config
        self.bbox_endpoint, self.image_topic = bbox_endpoint, image_topic
        self.hover_check = hover_check
        self.stop = Event()
        self.lock = Lock()
        self.latest_vision = self.display = self.status = None
        self.error = None
        self.threads = []

    def start(self):
        for name, target in (('bullet-vision', self._vision), ('bullet-mission', self._control)):
            thread = Thread(name=name, target=self._guard, args=(target,), daemon=True)
            self.threads.append(thread)
            thread.start()

    def _guard(self, target):
        try:
            target()
        except Exception as error:
            traceback.print_exc()
            self.error = f'{target.__name__}: {error}'
            self.stop.set()

    def _vision(self):
        receiver = ImageReceiver(self.transport['image_endpoint'], self.image_topic)
        try:
            while not self.stop.is_set():
                result = receiver.poll()
                if result is None:
                    self.stop.wait(0.002)
                    continue
                metadata, image = result
                bbox = detect_red_box(image)
                metadata = dict(metadata, processing_completed_us=time.monotonic_ns() // 1000)
                with self.lock:
                    self.latest_vision = (metadata, bbox)
                    self.display = (image, bbox)
        finally:
            receiver.close()

    def _control(self):
        mission = Mission(self.config, hover_check=self.hover_check)
        publisher = None
        last_command = None
        los_telemetry = None
        msp = None
        msp_arm_status_episode = None
        msp_status = None
        next_msp_telemetry = 0.
        next_msp_summary = 0.
        try:
            with zmq.Context() as context, context.socket(zmq.SUB) as subscriber, context.socket(zmq.PUB) as commands:
                for socket in (subscriber, commands):
                    socket.setsockopt(zmq.LINGER, 0)
                subscriber.setsockopt(zmq.RCVHWM, FEEDBACK_HWM)
                subscriber.setsockopt(zmq.MAXMSGSIZE, MAX_FLIGHT_BYTES)
                for topic in (CONFIG_TOPIC, FEEDBACK_TOPIC, STATE_TOPIC, END_TOPIC):
                    subscriber.setsockopt(zmq.SUBSCRIBE, topic)
                subscriber.connect(self.transport['telemetry_endpoint'])
                commands.setsockopt(zmq.SNDHWM, 4)
                commands.bind(self.transport['flight_command_endpoint'])
                publisher = BboxPublisher(self.bbox_endpoint)
                los_telemetry = LosTelemetryPublisher(publisher.socket)
                source_revisions = revisions(Path(__file__).resolve().parents[3])
                next_snapshot = 0.
                last_config = float('-inf')
                last_bbox = None
                next_tick = time.monotonic()
                try:
                    while not self.stop.is_set():
                        for _ in range(MAX_FEEDBACK_BATCH):
                            try:
                                parts = subscriber.recv_multipart(zmq.NOBLOCK)
                            except zmq.Again:
                                break
                            if len(parts) != 2:
                                continue
                            try:
                                if parts[0] == CONFIG_TOPIC:
                                    platform = decode(parts[1], validate_config)
                                    mission.configure(platform)
                                    if platform.get('backend') == 'betaflight' and msp is None:
                                        from .msp_flight import MspFlight
                                        msp = MspFlight()
                                        msp.start()
                                    last_config = time.monotonic()
                                elif parts[0] == END_TOPIC:
                                    mission.complete(decode(parts[1], validate_end))
                                elif parts[0] == STATE_TOPIC:
                                    state = decode(parts[1], validate_state)
                                    if state.get('terminal_result') is not None:
                                        mission.complete(state['terminal_result'])
                                elif parts[0] == FEEDBACK_TOPIC:
                                    packet = decode(parts[1], validate_feedback)
                                    if packet['source']['measurement_us'] / 1e6 <= time.monotonic():
                                        mission.observe(packet)
                            except (ValueError, TypeError, UnicodeError) as error:
                                if parts[0] == CONFIG_TOPIC:
                                    raise ValueError(f'incompatible platform: {error}') from error
                                continue
                        now = time.monotonic()
                        with self.lock:
                            detection = self.latest_vision
                        if now - last_config > CONFIG_TIMEOUT_S and mission.running_seen:
                            mission.fail('platform_stale')
                        if msp is not None and mission.identity is not None:
                            msp.set_episode(mission.identity)
                            msp_snapshot = msp.snapshot()
                            msp_status = msp_status_from_snapshot(msp_snapshot)
                            mission.observe_estimates(msp_snapshot)
                            now = time.monotonic()
                        command = mission.tick(now_s=now, detection=detection)
                        if command is not None and (now - last_config <= CONFIG_TIMEOUT_S or mission.feedback['state'] == 'running'):
                            if now - last_config > CONFIG_TIMEOUT_S:
                                command['active'] = False
                                command['channels'][4] = 1000
                                command['channels'][2] = 1000
                            if (msp is not None and command['channels'][4] >= 1500
                                    and msp_arm_status_episode != mission.identity):
                                print(format_msp_status_before_arm(msp_status), flush=True)
                                msp_arm_status_episode = mission.identity
                            commands.send_multipart([RC_TOPIC, encode(command, validate_rc)], zmq.NOBLOCK)
                            last_command = command
                            if msp is not None:
                                msp.update(command['channels'])
                        if msp is not None and mission.feedback is not None and now >= next_msp_telemetry:
                            from dataclasses import asdict as snapshot_dict
                            snapshot = msp.snapshot()
                            timing_now = time.monotonic()
                            metrics = msp.diagnostics(timing_now)
                            attitude = mission.last_input.attitude
                            image_source = detection[0].get('source') if detection is not None else None
                            image_time = None if not image_source else image_source['measurement_us'] / 1e6
                            sensor_metrics = mission.sensor_diagnostics(timing_now)
                            sensor_metrics.update(
                                image_age_s=None if image_time is None else timing_now - image_time,
                                image_attitude_offset_s=(None if image_time is None or attitude is None
                                                         else image_time - attitude.timestamp_s),
                                vision_reason=mission.diagnostics.get('vision_reason'), mission_phase=mission.phase)
                            packet = dict(schema_version=1, source=mission.feedback['source'], backend='betaflight',
                                timestamp_kind='host_receipt', firmware=msp.firmware, error=msp.error,
                                msp_delivery=metrics, sensor_feedback=sensor_metrics,
                                rc_frames_sent=msp.client.rc.sent, last_rc_sent_at_s=msp.client.rc.last_sent_at,
                                attitude=None if not snapshot or snapshot[0] is None else snapshot_dict(snapshot[0]),
                                altitude=None if not snapshot or snapshot[1] is None else snapshot_dict(snapshot[1]),
                                status=None if not snapshot or snapshot[2] is None else snapshot_dict(snapshot[2]))
                            publisher.socket.send_multipart([b'analysis/app/msp', json.dumps(packet, allow_nan=False).encode()], zmq.NOBLOCK)
                            next_msp_telemetry = now + .05
                            if timing_now >= next_msp_summary:
                                print('Sim sensors: feedback={observed_hz:.1f}/{expected_hz:.1f} Hz, '
                                      'age={feedback_age_s}, attitude_age={attitude_age_s}, '
                                      'barometer_age={barometer_age_s}; MSP status requests={requests_hz}, '
                                      'responses={responses_hz}'.format(**sensor_metrics, **metrics), flush=True)
                                next_msp_summary = timing_now + 1.
                        los_telemetry.publish(mission, now_s=now)
                        status = mission.status(now)
                        if status is not None:
                            if detection is not None:
                                metadata, bbox = detection
                                status['diagnostics']['latest_image'] = {key: metadata[key] for key in ('session', 'sequence', 'capture_ns', 'source', 'processing_completed_us') if key in metadata}
                                status['diagnostics']['vision_processing_latency_s'] = (metadata.get('processing_completed_us', round(now * 1e6)) - metadata['capture_ns'] / 1000) / 1e6
                            if now >= next_snapshot:
                                snapshot = dict(schema_version=1, source=mission.feedback['source'], role='app',
                                                settings=asdict(self.config), revisions=source_revisions)
                                publisher.socket.send_multipart([b'analysis/app/config', json.dumps(snapshot, allow_nan=False).encode()], zmq.NOBLOCK)
                                next_snapshot = now + 1.
                            publisher.socket.send_multipart([MISSION_TOPIC, json.dumps(status, allow_nan=False).encode()], zmq.NOBLOCK)
                            with self.lock:
                                self.status = status
                        if detection is not None:
                            metadata, bbox = detection
                            key = (metadata['session'], metadata['sequence'])
                            if key != last_bbox:
                                publisher.publish(metadata, bbox, self.image_topic)
                                last_bbox = key
                        next_tick += COMMAND_PERIOD_S
                        if next_tick < now:
                            next_tick = now + COMMAND_PERIOD_S
                        self.stop.wait(max(0, next_tick - time.monotonic()))
                finally:
                    if los_telemetry is not None:
                        los_telemetry.publish(mission, now_s=time.monotonic(), shutdown=True)
                    if last_command is not None:
                        now_us = time.monotonic_ns() // 1000
                        last_command.update(sequence=last_command['sequence'] + 1, issued_us=now_us,
                                            valid_until_us=now_us + 100_000, active=False)
                        last_command['channels'][2] = last_command['channels'][4] = 1000
                        commands.send_multipart([RC_TOPIC, encode(last_command, validate_rc)], zmq.NOBLOCK)
        finally:
            if msp is not None:
                msp.close()
            if publisher is not None:
                publisher.close()

    def close(self):
        self.stop.set()
        for thread in self.threads:
            thread.join(THREAD_JOIN_TIMEOUT_S)
            if thread.is_alive():
                self.error = f'{thread.name} failed to stop'


def run_mission(args, transport):
    config = NavigatorConfig.from_yaml(args.mission_config)
    runtime = MissionRuntime(transport, config, bbox_endpoint=args.bbox_endpoint, image_topic=args.topic,
                             hover_check=args.hover_check)
    window = 'Bullet app: mission'
    opened = False
    try:
        runtime.start()
        if args.debug:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            opened = True
        from .status import format_status, status_key
        previous_status = None
        previous_phase = None
        while not runtime.stop.is_set():
            with runtime.lock:
                display, status = runtime.display, runtime.status
            if status is not None and status_key(status) != previous_status:
                previous_status = status_key(status)
                previous_phase = status['phase']
                print(f"Mission {previous_phase}: {format_status(status)}", flush=True)
            if args.debug:
                if display is not None:
                    from .main import draw_debug
                    cv2.imshow(window, draw_debug(*display, f"{status['phase']}: {format_status(status)}" if status else 'Waiting'))
                if cv2.waitKey(10) & 0xff in (ord('q'), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    break
            else:
                runtime.stop.wait(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        runtime.close()
        if opened:
            cv2.destroyAllWindows()
    if runtime.error:
        print(f'Bullet mission failed: {runtime.error}', flush=True)
        return 1
    return 0
