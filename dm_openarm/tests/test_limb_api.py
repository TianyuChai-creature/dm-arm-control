"""Offline tests for left/right Limb API (no hardware)."""
from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import MagicMock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from dm_openarm.arm import Arm, MoveResult
from dm_openarm.limb import Limb


class FakeState:
    def __init__(self, can_id: int, position: float = 0.0):
        self.can_id = can_id
        self.position = position
        self.velocity = 0.0


def test_limb_rejects_foreign_can() -> None:
    class Stub:
        pass

    arm = Stub()
    limb = Limb(arm, "left", [0x01, 0x02], ["a", "b"])  # type: ignore[arg-type]
    try:
        limb._require_can(0x21)
        assert False, "expected ValueError"
    except ValueError as e:
        assert "0x21" in str(e)


def test_dual_move_joints_root_errors() -> None:
    class StubArm(Arm):
        def __init__(self) -> None:
            self.left = Limb(self, "left", [0x01], ["L"])  # type: ignore[misc]
            self.right = Limb(self, "right", [0x21], ["R"])  # type: ignore[misc]
            self._loop = MagicMock()

    arm = StubArm()
    try:
        arm.move_joints([0.0])
        assert False, "expected RuntimeError"
    except RuntimeError as e:
        assert "arm.left" in str(e) or "dual-arm" in str(e)


def test_limb_move_joints() -> None:
    class StubArm(Arm):
        def __init__(self) -> None:
            self._loop = MagicMock()
            self._loop.running.return_value = True
            self.left = Limb(self, "left", [0x01, 0x02], ["a", "b"])  # type: ignore[misc]
            self.right = Limb(self, "right", [0x21, 0x22], ["c", "d"])  # type: ignore[misc]
            self._done = False
            self.sent: list = []

        @property
        def mit_loop_running(self) -> bool:
            return True

        def states(self):  # type: ignore[override]
            if self._done:
                return [
                    FakeState(0x01, 0.25),
                    FakeState(0x02, 0.0),
                    FakeState(0x21, 0.0),
                    FakeState(0x22, 0.0),
                ]
            return [
                FakeState(0x01, 0.0),
                FakeState(0x02, 0.0),
                FakeState(0x21, 0.0),
                FakeState(0x22, 0.0),
            ]

        def mit(self, target, /, **kw):  # type: ignore[override]
            if isinstance(target, dict):
                self.sent.append(target)
                if all(getattr(v, "dq", 1.0) == 0.0 for v in target.values()):
                    self._done = True

    arm = StubArm()
    result = arm.left.move_joints([0.2, 0.0], duration=0.05, rate_hz=50.0, settle_s=0.0)
    assert isinstance(result, MoveResult)
    assert abs(result.q_cmd[0] - 0.2) < 1e-9


if __name__ == "__main__":
    test_limb_rejects_foreign_can()
    test_dual_move_joints_root_errors()
    test_limb_move_joints()
    print("test_limb_api OK")
