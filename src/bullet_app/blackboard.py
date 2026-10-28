"""Stamped cross-thread runtime state, independent of transport and mission logic."""
from collections import deque
from dataclasses import dataclass
from threading import Lock
from typing import Any


FEEDBACK_QUEUE_CAPACITY = 8


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


class RuntimeBlackboard:
    """Own snapshots shared by receiver, vision, control, and display workers."""
    def __init__(self):
        self._lock = Lock()
        self._config = None
        self._vision = self._display = self._status = None
        self._feedback = deque()
        self._terminal_events = deque()
        self._last_received_at_s = None
        self._malformed_messages = self._dropped_feedback = 0

    def write_config(self, value, received_at_s):
        with self._lock:
            self._config = StampedValue(value, received_at_s)
            self._last_received_at_s = received_at_s

    def write_feedback(self, value, received_at_s):
        with self._lock:
            sample = StampedValue(value, received_at_s)
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
