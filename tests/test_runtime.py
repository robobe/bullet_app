from types import SimpleNamespace

from bullet_app.blackboard import (FEEDBACK_QUEUE_CAPACITY, OUTPUT_EVENT_QUEUE_CAPACITY,
                                   RuntimeBlackboard)
from bullet_app.runtime import format_msp_status_before_arm, msp_status_from_snapshot


def test_msp_status_before_arm_formats_decoded_fields():
    status = SimpleNamespace(armed=False, mode_flags=3, arming_disabled_flags=16)

    assert format_msp_status_before_arm(status) == (
        'Betaflight MSP status before arm: armed=False, mode_flags=0x00000003, '
        'arming_disabled_flags=0x00000010')


def test_msp_status_before_arm_reports_unavailable_snapshot():
    assert format_msp_status_before_arm(None) == 'Betaflight MSP status before arm: unavailable'


def test_msp_status_snapshot_can_be_unavailable():
    assert msp_status_from_snapshot(None) is None


def test_blackboard_drops_oldest_feedback_and_preserves_terminal_events():
    board = RuntimeBlackboard()
    board.write_config({'backend': 'internal'}, 1.)
    for sequence in range(FEEDBACK_QUEUE_CAPACITY + 2):
        board.write_feedback({'sequence': sequence}, 2. + sequence)
    board.write_state({'terminal_result': {'event_id': 'state'}}, 20.)
    board.write_end({'event_id': 'end'}, 21.)

    inputs = board.take_control_inputs(FEEDBACK_QUEUE_CAPACITY)

    assert inputs.config.value == {'backend': 'internal'}
    assert [sample.value['sequence'] for sample in inputs.feedback] == list(range(2, FEEDBACK_QUEUE_CAPACITY + 2))
    assert [event.value['event_id'] for event in inputs.terminal_events] == ['state', 'end']
    assert inputs.health.feedback_queue_depth == 0
    assert inputs.health.dropped_feedback == 2
    assert board.latest_feedback().value == {'sequence': FEEDBACK_QUEUE_CAPACITY + 1}


def test_blackboard_returns_stamped_vision_display_and_status():
    board = RuntimeBlackboard()
    detection, display, status = ('metadata', 'bbox'), ('image', 'bbox'), {'phase': 'READY'}
    board.write_vision(detection, display, 1.)
    board.write_status(status, 2.)

    inputs = board.take_control_inputs(0)
    stamped_display, stamped_status = board.display_status()

    assert inputs.vision.value == detection and inputs.vision.received_at_s == 1.
    assert stamped_display.value == display and stamped_display.received_at_s == 1.
    assert stamped_status.value == status and stamped_status.received_at_s == 2.


def test_blackboard_keeps_latest_publication_state_and_bounded_events():
    board = RuntimeBlackboard()
    board.write_command({'sequence': 1}, 1.)
    board.write_command({'sequence': 2}, 2.)
    board.write_status({'sequence': 3}, 3.)
    board.write_config_snapshot({'sequence': 4}, 4.)
    board.write_msp_snapshot({'sequence': 5}, 5.)
    board.write_final_disarm({'sequence': 6}, 6.)
    for sequence in range(OUTPUT_EVENT_QUEUE_CAPACITY + 1):
        board.enqueue_los({'sequence': sequence}, 7. + sequence)

    inputs = board.take_publication_inputs()

    assert inputs.command.value == {'sequence': 2}
    assert inputs.status.value == {'sequence': 3}
    assert inputs.config_snapshot.value == {'sequence': 4}
    assert inputs.msp_snapshot.value == {'sequence': 5}
    assert inputs.final_disarm.value == {'sequence': 6}
    assert [event.value['sequence'] for event in inputs.los_events] == list(range(1, OUTPUT_EVENT_QUEUE_CAPACITY + 1))
    assert inputs.health.queue_depths['los'] == OUTPUT_EVENT_QUEUE_CAPACITY
    assert inputs.health.dropped_events['los'] == 1

    board.acknowledge_final_disarm()
    assert board.take_publication_inputs().final_disarm is None
