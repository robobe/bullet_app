"""Dedicated runtime workers; OpenCV UI stays on the main thread."""
from dataclasses import asdict
import os
from pathlib import Path
from bullet_msgs.provenance import revisions
from threading import Event, Thread
import time

import cv2
from loguru import logger
import numpy as np
import zmq
from bullet_msgs.flight import (CONFIG_TOPIC, FEEDBACK_TOPIC, MAX_FLIGHT_BYTES,
                               decode, validate_config, validate_feedback)
from bullet_msgs.control import STATE_TOPIC, validate_state
from bullet_msgs.scenario import END_TOPIC, validate_end
from los_tracking import NavigatorConfig
from .detector import detect_red_box
from .blackboard import RuntimeBlackboard
from .mission import Mission, COMMAND_PERIOD_S
from .los_telemetry import LosTelemetryScheduler
from .publisher import BboxPublisher
from .receiver import ImageReceiver
from .runtime_publisher import RuntimePublisher
from .ui import draw_debug


FEEDBACK_HWM = 64
FEEDBACKS_PER_CONTROL_TICK = 4
TELEMETRY_POLL_TIMEOUT_MS = 50
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


def input_diagnostics(health, now_s):
    return dict(input_feedback_queue_depth=health.feedback_queue_depth,
                input_dropped_feedback=health.dropped_feedback,
                input_malformed_messages=health.malformed_messages,
                input_age_s=None if health.last_received_at_s is None else now_s - health.last_received_at_s)


def publication_diagnostics(health):
    return {f'output_{name}_queue_depth': depth for name, depth in health.queue_depths.items()} | {
        f'output_dropped_{name}': count for name, count in health.dropped_events.items()}


class DetectorRuntime:
    """Own the detector-only image, publication, and optional display lifecycle."""

    def __init__(self, options, transport):
        self.options = options
        self.transport = transport

    def run(self):
        receiver = publisher = None
        window_open = False
        window = f"Bullet app: {self.options.topic} ({os.getpid()})"
        try:
            publisher = BboxPublisher(self.options.bbox_endpoint)
            receiver = ImageReceiver(self.transport["image_endpoint"], self.options.topic)
            image = np.zeros((480, 640, 3), dtype=np.uint8)
            bbox = last_frame = None
            last_report = time.monotonic()
            sequence = 0
            if self.options.debug:
                cv2.namedWindow(window, cv2.WINDOW_NORMAL)
                window_open = True
            logger.info("Subscribed to {} on {}, SHM pixels", self.options.topic, self.transport["image_endpoint"])
            logger.info("Publishing tracking/front/bbox on {}", self.options.bbox_endpoint)
            while True:
                result = receiver.poll()
                now = time.monotonic()
                if result is not None:
                    metadata, image = result
                    bbox = detect_red_box(image)
                    publisher.publish(metadata, bbox, self.options.topic)
                    sequence = metadata["sequence"]
                    last_frame = now
                status = "Waiting for frames" if last_frame is None else ("Live" if now - last_frame < 1 else "Stale")
                text = f"{status} | Frame {sequence} | bbox={bbox}"
                if now - last_report >= 1:
                    logger.info(text)
                    last_report = now
                if self.options.debug:
                    cv2.imshow(window, draw_debug(image, bbox, status))
                    if cv2.waitKey(10) & 0xff in (ord("q"), 27):
                        break
                    if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                        break
                elif result is None:
                    time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            if receiver is not None:
                receiver.close()
            if publisher is not None:
                publisher.close()
            if window_open:
                cv2.destroyAllWindows()
        return 0


class MissionRuntime:
    def __init__(self, transport, config, *, bbox_endpoint, image_topic, hover_check=False):
        self.transport, self.config = transport, config
        self.bbox_endpoint, self.image_topic = bbox_endpoint, image_topic
        self.hover_check = hover_check
        self.stop = Event()
        self.publisher_stop = Event()
        self.blackboard = RuntimeBlackboard()
        self.publisher = RuntimePublisher(transport, bbox_endpoint, self.blackboard, self.publisher_stop)
        self.error = None
        self.threads = []
        self.publisher_thread = None

    def start(self):
        self.publisher_thread = Thread(name='bullet-publisher', target=self._guard,
                                       args=(self.publisher.run,), daemon=True)
        self.publisher_thread.start()
        for name, target in (('bullet-telemetry', self._receive_telemetry),
                             ('bullet-vision', self._vision), ('bullet-mission', self._control)):
            thread = Thread(name=name, target=self._guard, args=(target,), daemon=True)
            self.threads.append(thread)
            thread.start()

    def _guard(self, target):
        try:
            target()
        except Exception as error:
            logger.exception("Mission thread {} failed", target.__name__)
            self.error = f'{target.__name__}: {error}'
            self.stop.set()

    def _receive_telemetry(self):
        with zmq.Context() as context, context.socket(zmq.SUB) as subscriber:
            subscriber.setsockopt(zmq.LINGER, 0)
            subscriber.setsockopt(zmq.RCVHWM, FEEDBACK_HWM)
            subscriber.setsockopt(zmq.MAXMSGSIZE, MAX_FLIGHT_BYTES)
            for topic in (CONFIG_TOPIC, FEEDBACK_TOPIC, STATE_TOPIC, END_TOPIC):
                subscriber.setsockopt(zmq.SUBSCRIBE, topic)
            subscriber.connect(self.transport['telemetry_endpoint'])
            while not self.stop.is_set():
                if not subscriber.poll(TELEMETRY_POLL_TIMEOUT_MS):
                    continue
                parts = subscriber.recv_multipart()
                if len(parts) != 2:
                    self.blackboard.record_malformed()
                    continue
                topic, payload = parts
                self._record_telemetry(topic, payload, time.monotonic())

    def _record_telemetry(self, topic, payload, received_at_s):
        try:
            if topic == CONFIG_TOPIC:
                self.blackboard.write_config(decode(payload, validate_config), received_at_s)
            elif topic == FEEDBACK_TOPIC:
                self.blackboard.write_feedback(decode(payload, validate_feedback), received_at_s)
            elif topic == STATE_TOPIC:
                self.blackboard.write_state(decode(payload, validate_state), received_at_s)
            elif topic == END_TOPIC:
                self.blackboard.write_end(decode(payload, validate_end), received_at_s)
        except (ValueError, TypeError, UnicodeError):
            if topic == CONFIG_TOPIC:
                raise
            self.blackboard.record_malformed()

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
                self.blackboard.write_vision((metadata, bbox), (image, bbox), time.monotonic())
                self.blackboard.enqueue_bbox((metadata, bbox, self.image_topic), time.monotonic())
        finally:
            receiver.close()

    def _control(self):
        mission = Mission(self.config, hover_check=self.hover_check)
        last_command = None
        los_telemetry = LosTelemetryScheduler()
        msp = None
        msp_arm_status_episode = None
        msp_status = None
        next_msp_telemetry = 0.
        next_msp_summary = 0.
        try:
            source_revisions = revisions(Path(__file__).resolve().parents[3])
            next_snapshot = 0.
            last_config = float('-inf')
            next_tick = time.monotonic()
            try:
                while not self.stop.is_set():
                    inputs = self.blackboard.take_control_inputs(FEEDBACKS_PER_CONTROL_TICK)
                    if inputs.config is not None and inputs.config.received_at_s > last_config:
                        mission.configure(inputs.config.value)
                        if inputs.config.value.get('backend') == 'betaflight' and msp is None:
                            from .msp_flight import MspFlight
                            msp = MspFlight()
                            msp.start()
                        last_config = inputs.config.received_at_s
                    for event in inputs.terminal_events:
                        mission.complete(event.value)
                    for sample in inputs.feedback:
                        if sample.value['source']['measurement_us'] / 1e6 <= time.monotonic():
                            mission.observe(sample.value)
                    now = time.monotonic()
                    detection = None if inputs.vision is None else inputs.vision.value
                    ingress = input_diagnostics(inputs.health, now)
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
                            logger.info(format_msp_status_before_arm(msp_status))
                            msp_arm_status_episode = mission.identity
                        last_command = command
                        self.blackboard.write_command(command, now)
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
                            vision_reason=mission.diagnostics.get('vision_reason'), mission_phase=mission.phase,
                                **input_diagnostics(inputs.health, timing_now),
                                **publication_diagnostics(self.blackboard.publication_health()))
                        packet = dict(schema_version=1, source=mission.feedback['source'], backend='betaflight',
                            timestamp_kind='host_receipt', firmware=msp.firmware, error=msp.error,
                            msp_delivery=metrics, sensor_feedback=sensor_metrics,
                            rc_frames_sent=msp.client.rc.sent, last_rc_sent_at_s=msp.client.rc.last_sent_at,
                            attitude=None if not snapshot or snapshot[0] is None else snapshot_dict(snapshot[0]),
                            altitude=None if not snapshot or snapshot[1] is None else snapshot_dict(snapshot[1]),
                            status=None if not snapshot or snapshot[2] is None else snapshot_dict(snapshot[2]))
                        self.blackboard.write_msp_snapshot(packet, timing_now)
                        next_msp_telemetry = now + .05
                        if timing_now >= next_msp_summary:
                            logger.info('Sim sensors: feedback={observed_hz:.1f}/{expected_hz:.1f} Hz, '
                                        'age={feedback_age_s}, attitude_age={attitude_age_s}, '
                                        'barometer_age={barometer_age_s}; MSP status requests={requests_hz}, '
                                        'responses={responses_hz}, attitude requests={attitude[requests_hz]}, '
                                        'responses={attitude[responses_hz]}, altitude requests={altitude[requests_hz]}, '
                                        'responses={altitude[responses_hz]}'.format(**sensor_metrics, **metrics))
                            next_msp_summary = timing_now + 1.
                    if msp is not None and mission.feedback is not None:
                        for delivery in msp.drain_rc_deliveries():
                            packet = dict(schema_version=1, source=mission.feedback['source'], backend='betaflight',
                                          timestamp_kind='host_monotonic', sent_at_us=delivery.sent_at_us,
                                          connection_generation=delivery.generation, channels=list(delivery.channels))
                            self.blackboard.enqueue_msp_delivery(packet, now)
                    los_packet = los_telemetry.sample(mission, now_s=now)
                    if los_packet is not None:
                        self.blackboard.enqueue_los(los_packet, now)
                    status = mission.status(now)
                    if status is not None:
                        status['diagnostics'].update(ingress, **publication_diagnostics(self.blackboard.publication_health()))
                        if detection is not None:
                            metadata, bbox = detection
                            status['diagnostics']['latest_image'] = {key: metadata[key] for key in ('session', 'sequence', 'capture_ns', 'source', 'processing_completed_us') if key in metadata}
                            status['diagnostics']['vision_processing_latency_s'] = (metadata.get('processing_completed_us', round(now * 1e6)) - metadata['capture_ns'] / 1000) / 1e6
                        if now >= next_snapshot:
                            snapshot = dict(schema_version=1, source=mission.feedback['source'], role='app',
                                            settings=asdict(self.config), revisions=source_revisions)
                            self.blackboard.write_config_snapshot(snapshot, now)
                            next_snapshot = now + 1.
                        self.blackboard.write_status(status, now)
                    next_tick += COMMAND_PERIOD_S
                    if next_tick < now:
                        next_tick = now + COMMAND_PERIOD_S
                    self.stop.wait(max(0, next_tick - time.monotonic()))
            finally:
                los_packet = los_telemetry.sample(mission, now_s=time.monotonic(), shutdown=True)
                if los_packet is not None:
                    self.blackboard.enqueue_los(los_packet, time.monotonic())
                if last_command is not None:
                    now_us = time.monotonic_ns() // 1000
                    last_command.update(sequence=last_command['sequence'] + 1, issued_us=now_us,
                                        valid_until_us=now_us + 100_000, active=False)
                    last_command['channels'][2] = last_command['channels'][4] = 1000
                    self.blackboard.write_final_disarm(last_command, now_us / 1e6)
        finally:
            if msp is not None:
                msp.close()

    def close(self):
        self.stop.set()
        for thread in self.threads:
            thread.join(THREAD_JOIN_TIMEOUT_S)
            if thread.is_alive():
                self.error = f'{thread.name} failed to stop'
        self.publisher_stop.set()
        if self.publisher_thread is not None:
            self.publisher_thread.join(THREAD_JOIN_TIMEOUT_S)
            if self.publisher_thread.is_alive():
                self.error = f'{self.publisher_thread.name} failed to stop'


class MissionRunner:
    """Own the mission runtime and optional OpenCV status display."""

    def __init__(self, options, transport):
        self.options = options
        self.transport = transport

    def run(self):
        config = NavigatorConfig.from_yaml(self.options.mission_config)
        runtime = MissionRuntime(self.transport, config, bbox_endpoint=self.options.bbox_endpoint,
                                 image_topic=self.options.topic, hover_check=self.options.hover_check)
        window = 'Bullet app: mission'
        opened = False
        try:
            runtime.start()
            if self.options.debug:
                cv2.namedWindow(window, cv2.WINDOW_NORMAL)
                opened = True
            from .status import format_status, status_key
            previous_status = None
            while not runtime.stop.is_set():
                display, status = runtime.blackboard.display_status()
                display = None if display is None else display.value
                status = None if status is None else status.value
                if status is not None and status_key(status) != previous_status:
                    previous_status = status_key(status)
                    logger.info("Mission {}: {}", status['phase'], format_status(status))
                if self.options.debug:
                    if display is not None:
                        text = f"{status['phase']}: {format_status(status)}" if status else 'Waiting'
                        cv2.imshow(window, draw_debug(*display, text))
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
            logger.error("Bullet mission failed: {}", runtime.error)
            return 1
        return 0


def run_mission(args, transport):
    """Compatibility wrapper for callers that imported the former helper."""
    return MissionRunner(args, transport).run()
