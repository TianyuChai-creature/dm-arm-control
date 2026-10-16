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
    arm._loop = MagicMock()
    arm.stop_mit_loop = MagicMock(side_effect=RuntimeError("loop failed"))
    arm._arm.disable.side_effect = RuntimeError("status unconfirmed")
    try:
        arm.disable()
        assert False, "expected unknown shutdown"
    except RuntimeError as exc:
        assert "shutdown unknown" in str(exc) and "loop failed" in str(exc)
    arm._arm.disconnect.assert_not_called()
    arm._loop._shutdown_failed.assert_called_once()
    assert "status unconfirmed" in arm._loop._shutdown_failed.call_args.args[0]


def test_empty_python_update_never_reaches_native_loop() -> None:
    arm = object.__new__(Arm)
    arm._arm = MagicMock()
    arm._loop = MagicMock()
    try:
        arm.mit({})
        assert False, "expected empty update rejection"
    except ValueError:
        pass
    arm._loop.set_commands.assert_not_called()


def test_limb_zero_batch_reaches_native_once() -> None:
    arm = object.__new__(Arm)
    arm._arm = MagicMock()
    arm._set_zero_many([1, 2], persist=False)
    arm._arm._set_zero_many.assert_called_once_with([1, 2], False)


def test_native_empty_update_preserves_sequence_and_commands() -> None:
    config = _core.ArmConfig()
    motor = _core.MotorConfig()
    motor.can_id, motor.mst_id = 1, 17
    config.motors = [motor]
    arm = _core.DmArm(config)
    loop = _core.MitLoopController(arm)
    loop.set_commands({1: _core.MitCommand(kp=10, q=0.3)})
    for submit in (lambda: loop.set_commands({}), lambda: loop.set_all_commands([])):
        try:
            submit()
            assert False, "expected native empty update rejection"
        except ValueError:
            pass
        assert loop.accepted_sequence() == 1
        assert loop.commands()[0].q == 0.3


def test_device_disable_failure_publishes_native_unknown_state() -> None:
    device = _core.DmArm(_core.ArmConfig())  # Never connects to USB.
    arm = object.__new__(Arm)
    arm._loop = _core.MitLoopController(device)
    arm._arm = MagicMock()
    arm._arm.disable.side_effect = RuntimeError("status unconfirmed")
    for _ in range(2):
        try:
            arm.disable()
            assert False, "expected unconfirmed shutdown"
        except RuntimeError:
            pass
        assert arm.safety_state == "SHUTDOWN_UNKNOWN"
        assert "status unconfirmed" in arm.fault
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


def test_recoverable_status_api_is_exposed_and_forwarded():
    from dm_openarm import ControlIssue, ControlStatus, HoldPendingError, FeedbackStaleError
    assert ControlIssue is not None and ControlStatus is not None
    assert issubclass(HoldPendingError, TimeoutError)
    assert issubclass(FeedbackStaleError, RuntimeError)
    arm = object.__new__(Arm)
    arm._loop = MagicMock()
    expected = object()
    arm._loop.control_status.return_value = expected
    assert arm.control_status() is expected
    arm._loop.control_status.assert_called_once_with()


def test_guarded_start_forwards_optional_timing_without_python_policy():
    arm = object.__new__(Arm)
    arm._loop = MagicMock()
    gains = [object()]
    for method in ('enable_seeded', 'start_seeded'):
        params = inspect.signature(getattr(Arm, method)).parameters
        assert params['feedback_hard_timeout'].default is None
        assert params['hold_timeout'].default == .2
        getattr(arm, method)(gains)
        getattr(arm._loop, method).assert_called_once_with(gains, 250., .1, .2, None, .2)
        getattr(arm._loop, method).reset_mock()
        getattr(arm, method)(gains, hz=100, command_timeout=.15, feedback_timeout=.3,
                             feedback_hard_timeout=.9, hold_timeout=.05)
        getattr(arm._loop, method).assert_called_once_with(gains, 100, .15, .3, .9, .05)


def test_control_status_is_an_independent_readonly_snapshot():
    import pytest
    device = _core.DmArm(_core.ArmConfig())
    loop = _core.MitLoopController(device)
    before = loop.control_status()
    assert before.state == 'DISABLED' and before.warnings == [] and before.faults == []
    loop._shutdown_failed('owner release unconfirmed')
    after = loop.control_status()
    assert before.state == 'DISABLED' and before.faults == []
    assert after.state == 'SHUTDOWN_UNKNOWN' and after.warnings == []
    issue = after.faults[0]
    assert issue.code == 'SHUTDOWN_UNCONFIRMED' and issue.message == 'owner release unconfirmed'
    assert issue.can_id is None and issue.age_s is None and issue.rx_sequence is None
    with pytest.raises(AttributeError):
        issue.code = 'changed'
    with pytest.raises(AttributeError):
        after.state = 'ACTIVE'
    copied = after.faults
    copied.clear()
    assert len(loop.control_status().faults) == 1
    assert after.hold_request_sequence == after.hold_submitted_sequence == 0


def test_invalid_new_deadlines_are_rejected_before_usb():
    import pytest
    config = _core.ArmConfig()
    motor = _core.MotorConfig()
    motor.can_id, motor.mst_id = 1, 17
    config.motors = [motor]
    device = _core.DmArm(config)
    loop = _core.MitLoopController(device)
    gains = [_core.MitCommand(kp=20, kd=1)]
    for hard in (0., .2, -.1, float('inf'), float('nan')):
        with pytest.raises(ValueError, match='timing'):
            loop.enable_seeded(gains, feedback_hard_timeout=hard)
        assert not device.connected() and loop.control_status().state == 'DISABLED'
    for wait in (0., -1., float('nan'), float('inf')):
        with pytest.raises(ValueError, match='timing'):
            loop.enable_seeded(gains, hold_timeout=wait)
        assert not device.connected()
