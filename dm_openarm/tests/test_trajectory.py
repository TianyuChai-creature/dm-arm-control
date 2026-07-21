"""Offline unit tests for joint quintic trajectory (no hardware)."""
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from dm_openarm.trajectory import (
    estimate_duration,
    plan_joint_trajectory,
    sample_quintic,
    sample_quintic_ddq,
)


def test_endpoint_boundaries() -> None:
    q0, qf, T = 0.1, 0.9, 2.0
    q, dq = sample_quintic(q0, qf, T, 0.0)
    assert abs(q - q0) < 1e-12 and abs(dq) < 1e-12
    q, dq = sample_quintic(q0, qf, T, T)
    assert abs(q - qf) < 1e-12 and abs(dq) < 1e-12
    q, dq = sample_quintic(q0, qf, T, -1.0)
    assert abs(q - q0) < 1e-12
    q, dq = sample_quintic(q0, qf, T, T + 1.0)
    assert abs(q - qf) < 1e-12 and abs(dq) < 1e-12


def test_midpoint_and_accel_endpoints() -> None:
    q0, qf, T = 0.0, 1.0, 1.0
    q_mid, dq_mid = sample_quintic(q0, qf, T, 0.5)
    assert abs(q_mid - 0.5) < 1e-12
    # Peak velocity for unit step rest-to-rest: 15/8 / T at s=0.5
    assert abs(dq_mid - 1.875) < 1e-9
    assert abs(sample_quintic_ddq(q0, qf, T, 0.0)) < 1e-12
    assert abs(sample_quintic_ddq(q0, qf, T, T)) < 1e-12
    # Accel should change sign around mid (starts positive for positive step)
    a_early = sample_quintic_ddq(q0, qf, T, 0.2)
    a_late = sample_quintic_ddq(q0, qf, T, 0.8)
    assert a_early > 0.0
    assert a_late < 0.0


def test_multi_joint_sync() -> None:
    q0 = [0.0, 1.0, -0.5]
    qf = [0.5, 1.0, 0.5]
    traj = plan_joint_trajectory(q0, qf, duration=2.0)
    assert traj.duration == 2.0
    assert traj.n_joints == 3
    qs0, dqs0 = traj.sample(0.0)
    qsT, dqsT = traj.sample(2.0)
    for i in range(3):
        assert abs(qs0[i] - q0[i]) < 1e-12
        assert abs(dqs0[i]) < 1e-12
        assert abs(qsT[i] - qf[i]) < 1e-12
        assert abs(dqsT[i]) < 1e-12
    # Stationary joint stays put
    qs, dqs = traj.sample(1.0)
    assert abs(qs[1] - 1.0) < 1e-12
    assert abs(dqs[1]) < 1e-12


def test_estimate_duration() -> None:
    T = estimate_duration([0.0, 0.0], [0.2, 0.8], vmax=0.4, t_min=0.5)
    assert abs(T - 2.0) < 1e-12  # 0.8/0.4
    T2 = estimate_duration([0.0], [0.05], vmax=0.4, t_min=0.5)
    assert abs(T2 - 0.5) < 1e-12  # floored
    traj = plan_joint_trajectory([0.0, 0.0], [0.2, 0.8], duration=None, vmax=0.4)
    assert abs(traj.duration - 2.0) < 1e-12


def test_invalid_args() -> None:
    try:
        sample_quintic(0.0, 1.0, 0.0, 0.1)
        assert False, "expected ValueError"
    except ValueError:
        pass
    try:
        plan_joint_trajectory([0.0], [0.0, 1.0], duration=1.0)
        assert False, "expected ValueError"
    except ValueError:
        pass


if __name__ == "__main__":
    test_endpoint_boundaries()
    test_midpoint_and_accel_endpoints()
    test_multi_joint_sync()
    test_estimate_duration()
    test_invalid_args()
    print("test_trajectory OK")
