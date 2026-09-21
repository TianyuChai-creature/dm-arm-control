"""Offline tests for high-level Arm helpers (no MIT worker / no USB required)."""
from __future__ import annotations

import inspect
import sys
from pathlib import Path
from unittest.mock import MagicMock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

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


def main() -> None:
    test_mit_loop_does_not_home_by_default()
    test_from_yaml_accepts_single_side()
    test_native_guard_helpers_are_exposed()
    test_timing_stats_is_exposed_and_forwarded()
    test_disable_disconnects_after_loop_failure()
    print("test_python_api OK")


if __name__ == "__main__":
    main()
