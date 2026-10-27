from types import SimpleNamespace

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
