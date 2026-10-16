"""Offline tests for left/right Limb API (no hardware)."""
from __future__ import annotations

from unittest.mock import MagicMock

from dm_openarm.limb import Limb


def test_limb_rejects_foreign_can() -> None:
    class Stub:
        pass

    arm = Stub()
    limb = Limb(arm, "left", [0x00, 0x01], ["a", "b"])  # type: ignore[arg-type]
    try:
        limb._require_can(0x05)
        assert False, "expected ValueError"
    except ValueError as e:
        assert "0x05" in str(e)


def test_limb_zero_all_submits_one_native_batch() -> None:
    arm = MagicMock()
    limb = Limb(arm, "left", [0, 1], ["a", "b"])
    limb.set_zero_all(persist=False)
    arm._set_zero_many.assert_called_once_with([0, 1], persist=False)
    arm.set_zero.assert_not_called()


if __name__ == "__main__":
    test_limb_rejects_foreign_can()
    arm = MagicMock()
    limb = Limb(arm, "left", [0, 1], ["a", "b"])
    update = {0: object(), 1: object()}
    limb.mit(update)
    arm.mit.assert_called_once_with(update)
    try:
        limb.mit({0: object(), 5: object()})
        assert False, "expected foreign CAN rejection"
    except ValueError:
        pass
    assert arm.mit.call_count == 1
    print("test_limb_api OK")
