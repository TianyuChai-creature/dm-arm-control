"""Offline tests for high-level Arm helpers (no MIT worker / no USB required)."""
from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import MagicMock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from dm_openarm.arm import Arm, MoveResult
from dm_openarm.limb import Limb, _default_gains
from dm_openarm.trajectory import plan_joint_trajectory


class FakeState:
    def __init__(self, can_id: int, position: float, velocity: float = 0.0):
        self.can_id = can_id
        self.position = position
        self.velocity = velocity


class FakeCmd:
    def __init__(self, kp=0.0, kd=0.0, q=0.0, dq=0.0, tau=0.0):
        self.kp = kp
        self.kd = kd
        self.q = q
        self.dq = dq
        self.tau = tau


def assert_close(actual: float, expected: float, tol: float = 1e-9) -> None:
    assert abs(actual - expected) < tol, f"{actual} != {expected}"


def test_default_gains_five_dof() -> None:
    kp, kd = _default_gains(5)
    assert kp[:3] == [12.0, 12.0, 12.0]
    assert kp[3:] == [10.0, 10.0]
    assert kd[3] == 0.8


def test_move_to_and_hold_at_update_commands() -> None:
    """move_to/hold_at write MIT targets via mit() (mocked loop)."""
    arm = object.__new__(Arm)
    stored: dict[int, FakeCmd] = {}

    def mit(target, /, kp=0.0, kd=0.0, q=0.0, dq=0.0, tau=0.0):
        if isinstance(target, dict):
            for cid, cmd in target.items():
                stored[int(cid)] = FakeCmd(cmd.kp, cmd.kd, cmd.q, cmd.dq, cmd.tau)
        else:
            stored[int(target)] = FakeCmd(kp, kd, q, dq, tau)

    arm.mit = mit  # type: ignore[method-assign]
    arm.move_to(0x01, q=0.25, kp=12.0, kd=0.6)
    arm.hold_at(0x02, q=-0.10, kp=10.0, kd=0.8)

    assert_close(stored[0x01].q, 0.25)
    assert_close(stored[0x01].kp, 12.0)
    assert_close(stored[0x01].dq, 0.0)
    assert_close(stored[0x02].q, -0.10)
    assert_close(stored[0x02].kd, 0.8)


def test_move_joints_time_boxed_returns_result() -> None:
    """move_joints finishes by duration and reports lag without raising."""

    class StubArm(Arm):
        def __init__(self) -> None:
            # Bypass real DmArm / MitLoopController construction.
            self._loop = MagicMock()
            self.right = None
            self.left = Limb(self, "left", [0x01, 0x02], ["a", "b"])  # type: ignore[misc]

        @property
        def mit_loop_running(self) -> bool:
            return True

    arm = StubArm()
    can_ids = [0x01, 0x02]
    q0 = [0.0, 0.0]
    qf = [0.2, -0.1]
    arm._done = False  # type: ignore[attr-defined]

    def states():
        if arm._done:  # type: ignore[attr-defined]
            return [
                FakeState(can_ids[0], qf[0] + 0.05),
                FakeState(can_ids[1], qf[1]),
            ]
        return [FakeState(can_ids[0], q0[0]), FakeState(can_ids[1], q0[1])]

    arm.states = states  # type: ignore[method-assign]
    sent: list[dict] = []

    def mit(target, /, **kwargs):
        if isinstance(target, dict):
            sent.append({k: (v.q, v.dq) for k, v in target.items()})
            if all(abs(v.dq) < 1e-9 for v in target.values()):
                arm._done = True  # type: ignore[attr-defined]

    arm.mit = mit  # type: ignore[method-assign]

    result = arm.move_joints(
        qf,
        duration=0.05,
        rate_hz=50.0,
        settle_s=0.0,
        kp=5.0,
        kd=0.5,
    )
    assert isinstance(result, MoveResult)
    assert_close(result.duration, 0.05)
    assert_close(result.q_cmd[0], 0.2)
    assert_close(result.q_cmd[1], -0.1)
    assert result.max_abs_err >= 0.0
    assert len(sent) >= 1
    # Time-boxed: lag is reported, not an exception
    assert_close(result.err[0], 0.05, tol=1e-6)


def test_plan_used_by_move_is_rest_to_rest() -> None:
    traj = plan_joint_trajectory([0.0, 0.0], [0.3, -0.3], duration=1.0)
    qs, dqs = traj.sample(0.0)
    assert abs(dqs[0]) < 1e-12
    qs, dqs = traj.sample(1.0)
    assert abs(qs[0] - 0.3) < 1e-12
    assert abs(dqs[0]) < 1e-12


def main() -> None:
    test_default_gains_five_dof()
    test_move_to_and_hold_at_update_commands()
    test_move_joints_time_boxed_returns_result()
    test_plan_used_by_move_is_rest_to_rest()
    print("test_python_api OK")


if __name__ == "__main__":
    main()
