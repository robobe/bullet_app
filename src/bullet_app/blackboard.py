"""Stamped cross-thread runtime state, independent of transport and mission logic."""
from collections import deque
from dataclasses import dataclass
from threading import Lock
from typing import Any


FEEDBACK_QUEUE_CAPACITY = 8
OUTPUT_EVENT_QUEUE_CAPACITY = 64


@dataclass(frozen=True)
class StampedValue:
    value: Any
    received_at_s: float


@dataclass(frozen=True)
class ReceiverHealth:
    last_received_at_s: float | None
    feedback_queue_depth: int
    malformed_messages: int
    dropped_feedback: int


@dataclass(frozen=True)
class ControlInputs:
    config: StampedValue | None
    feedback: tuple[StampedValue, ...]
    terminal_events: tuple[StampedValue, ...]
    vision: StampedValue | None
    health: ReceiverHealth


@dataclass(frozen=True)
class PublicationHealth:
    queue_depths: dict[str, int]
    dropped_events: dict[str, int]


@dataclass(frozen=True)
class PublicationInputs:
    command: StampedValue | None
    status: StampedValue | None
    config_snapshot: StampedValue | None
    msp_snapshot: StampedValue | None
    bbox_events: tuple[StampedValue, ...]
    los_events: tuple[StampedValue, ...]
    msp_delivery_events: tuple[StampedValue, ...]
    final_disarm: StampedValue | None
    health: PublicationHealth


class RuntimeBlackboard:
    """Own snapshots shared by receiver, vision, control, and display workers."""
    def __init__(self):
        self._lock = Lock()
        self._config = None
        self._latest_feedback = None
        self._vision = self._display = self._status = None
        self._feedback = deque()
        self._terminal_events = deque()
        self._command = self._config_snapshot = self._msp_snapshot = self._final_disarm = None
        self._output_events = {name: deque() for name in ('bbox', 'los', 'msp_delivery')}
        self._dropped_events = {name: 0 for name in self._output_events}
        self._last_received_at_s = None
        self._malformed_messages = self._dropped_feedback = 0

    def write_config(self, value, received_at_s):
        with self._lock:
            self._config = StampedValue(value, received_at_s)
            self._last_received_at_s = received_at_s

    def write_feedback(self, value, received_at_s):
        with self._lock:
            sample = StampedValue(value, received_at_s)
            self._latest_feedback = sample
            if len(self._feedback) == FEEDBACK_QUEUE_CAPACITY:
                self._feedback.popleft()
                self._dropped_feedback += 1
            self._feedback.append(sample)
            self._last_received_at_s = received_at_s

    def write_state(self, value, received_at_s):
        with self._lock:
            if value.get('terminal_result') is not None:
                self._terminal_events.append(StampedValue(value['terminal_result'], received_at_s))
            self._last_received_at_s = received_at_s

    def write_end(self, value, received_at_s):
        with self._lock:
            self._terminal_events.append(StampedValue(value, received_at_s))
            self._last_received_at_s = received_at_s

    def record_malformed(self):
        with self._lock:
            self._malformed_messages += 1

    def write_vision(self, detection, display, received_at_s):
        with self._lock:
            stamped = StampedValue(detection, received_at_s)
            self._vision = stamped
            self._display = StampedValue(display, received_at_s)

    def write_status(self, value, received_at_s):
        with self._lock:
            self._status = StampedValue(value, received_at_s)

    def write_command(self, value, received_at_s):
        with self._lock:
            self._command = StampedValue(value, received_at_s)

    def write_config_snapshot(self, value, received_at_s):
        with self._lock:
            self._config_snapshot = StampedValue(value, received_at_s)

    def write_msp_snapshot(self, value, received_at_s):
        with self._lock:
            self._msp_snapshot = StampedValue(value, received_at_s)

    def write_final_disarm(self, value, received_at_s):
        with self._lock:
            self._final_disarm = StampedValue(value, received_at_s)

    def acknowledge_final_disarm(self):
        with self._lock:
            self._final_disarm = None

    def enqueue_bbox(self, value, received_at_s):
        self._enqueue('bbox', value, received_at_s)

    def enqueue_los(self, value, received_at_s):
        self._enqueue('los', value, received_at_s)

    def enqueue_msp_delivery(self, value, received_at_s):
        self._enqueue('msp_delivery', value, received_at_s)

    def _enqueue(self, name, value, received_at_s):
        with self._lock:
            events = self._output_events[name]
            if len(events) == OUTPUT_EVENT_QUEUE_CAPACITY:
                events.popleft()
                self._dropped_events[name] += 1
            events.append(StampedValue(value, received_at_s))

    def record_output_drop(self, name):
        with self._lock:
            self._dropped_events[name] += 1

    def take_control_inputs(self, feedback_limit):
        with self._lock:
            feedback = tuple(self._feedback.popleft()
                             for _ in range(min(feedback_limit, len(self._feedback))))
            terminal_events = tuple(self._terminal_events)
            self._terminal_events.clear()
            health = ReceiverHealth(self._last_received_at_s, len(self._feedback),
                                    self._malformed_messages, self._dropped_feedback)
            return ControlInputs(self._config, feedback, terminal_events, self._vision, health)

    def display_status(self):
        with self._lock:
            return self._display, self._status

    def latest_feedback(self):
        with self._lock:
            return self._latest_feedback

    def latest_status(self):
        with self._lock:
            return self._status

    def take_publication_inputs(self):
        with self._lock:
            queue_depths = {name: len(queue) for name, queue in self._output_events.items()}
            events = {name: tuple(queue) for name, queue in self._output_events.items()}
            for queue in self._output_events.values():
                queue.clear()
            health = PublicationHealth(queue_depths, dict(self._dropped_events))
            return PublicationInputs(self._command, self._status, self._config_snapshot, self._msp_snapshot,
                                     events['bbox'], events['los'], events['msp_delivery'], self._final_disarm,
                                     health)

    def publication_health(self):
        with self._lock:
            return PublicationHealth({name: len(queue) for name, queue in self._output_events.items()},
                                     dict(self._dropped_events))
