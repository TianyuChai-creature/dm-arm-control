"""Offline tests for high-level Arm helpers (no MIT worker / no USB required)."""
from __future__ import annotations

import inspect
from unittest.mock import MagicMock

from dm_openarm import MotorTimingStats, TimingStats, _core
from dm_openarm.arm import Arm


def test_mit_loop_does_not_home_by_default() -> None:
    params = inspect.signature(Arm.start_mit_loop).parameters
    assert params["hz"].default == _core.DEFAULT_CONTROL_HZ
    assert params["home"].default is False


def test_from_yaml_accepts_single_side() -> None:
    params = inspect.signature(Arm.from_yaml).parameters
    assert params["side"].default is None


def test_native_guard_helpers_are_exposed() -> None:
    assert callable(Arm.enable_seeded)
    assert callable(Arm.hold)
    assert callable(Arm.hold_command)


def test_timing_stats_is_exposed_and_forwarded() -> None:
    assert TimingStats is not None
    assert MotorTimingStats is not None
    arm = object.__new__(Arm)
    arm._loop = MagicMock()
    expected = object()
    arm._loop.timing_stats.return_value = expected
    assert arm.timing_stats() is expected
    arm._loop.timing_stats.assert_called_once_with()


def test_disable_disconnects_after_loop_failure() -> None:
    arm = object.__new__(Arm)
    arm._arm = MagicMock()
    arm.stop_mit_loop = MagicMock(side_effect=RuntimeError("loop failed"))
    try:
        arm.disable()
        assert False, "expected loop failure"
    except RuntimeError:
        pass
    arm._arm.disable.assert_called_once()
    arm._arm.disconnect.assert_called_once()


def test_partial_update_uses_native_merge() -> None:
    arm = object.__new__(Arm)
    arm._loop = MagicMock()
    arm._arm = MagicMock()
    arm._config = MagicMock()
    arm._config.motors = [MagicMock(can_id=1), MagicMock(can_id=2)]
    update = {1: _core.MitCommand(kp=10, kd=0.5, q=0.3)}
    arm.mit(update)
    arm._loop.set_commands.assert_called_once_with(update)
    arm._loop.commands.assert_not_called()
    arm._loop.set_all_commands.assert_not_called()


def test_native_partial_validation_is_atomic() -> None:
    config = _core.ArmConfig()
    motors = []
    for can_id in (1, 2):
        motor = _core.MotorConfig()
        motor.can_id = can_id
        motor.mst_id = can_id + 16
        motors.append(motor)
    config.motors = motors
    device = _core.DmArm(config)  # Construction opens no USB context.
    loop = _core.MitLoopController(device)
    loop.set_commands({1: _core.MitCommand(kp=10, kd=0.5, q=0.3)})
    for invalid_id, q in ((9, 0.4), (2, 99.0)):
        try:
            loop.set_commands({1: _core.MitCommand(q=0.7), invalid_id: _core.MitCommand(q=q)})
            assert False, "expected invalid partial batch"
        except (ValueError, IndexError):
            pass
        assert loop.accepted_sequence() == 1
        assert loop.commands()[0].q == 0.3
        assert loop.commands()[1].kp == 0


def test_disable_failure_does_not_disconnect() -> None:
    arm = object.__new__(Arm)
    arm._arm = MagicMock()
    arm.stop_mit_loop = MagicMock(side_effect=RuntimeError("loop failed"))
    arm._arm.disable.side_effect = RuntimeError("status unconfirmed")
    try:
        arm.disable()
        assert False, "expected unknown shutdown"
    except RuntimeError as exc:
        assert "shutdown unknown" in str(exc) and "loop failed" in str(exc)
    arm._arm.disconnect.assert_not_called()


def main() -> None:
    test_mit_loop_does_not_home_by_default()
    test_from_yaml_accepts_single_side()
    test_native_guard_helpers_are_exposed()
    test_timing_stats_is_exposed_and_forwarded()
    test_disable_disconnects_after_loop_failure()
    test_partial_update_uses_native_merge()
    test_native_partial_validation_is_atomic()
    test_disable_failure_does_not_disconnect()
    print("test_python_api OK")


if __name__ == "__main__":
    main()
