"""Joint-space quintic (rest-to-rest) trajectory planning.

Time-parameterized q(t), dq(t) for synchronous multi-joint motion.
No hardware dependency.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


def sample_quintic(q0: float, qf: float, T: float, t: float) -> tuple[float, float]:
    """Sample rest-to-rest quintic: zero vel/acc at endpoints.

    Returns (q, dq) at time t in [0, T] (clamped outside).
    """
    if T <= 0.0:
        raise ValueError("duration T must be > 0")
    if t <= 0.0:
        return float(q0), 0.0
    if t >= T:
        return float(qf), 0.0
    s = t / T
    s2 = s * s
    s3 = s2 * s
    s4 = s3 * s
    s5 = s4 * s
    dq = float(qf) - float(q0)
    q = float(q0) + dq * (10.0 * s3 - 15.0 * s4 + 6.0 * s5)
    qd = dq * (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / T
    return q, qd


def sample_quintic_ddq(q0: float, qf: float, T: float, t: float) -> float:
    """Second derivative of sample_quintic (for tests / optional FF)."""
    if T <= 0.0:
        raise ValueError("duration T must be > 0")
    if t <= 0.0 or t >= T:
        return 0.0
    s = t / T
    s2 = s * s
    s3 = s2 * s
    dq = float(qf) - float(q0)
    # d/dt of (30s^2 - 60s^3 + 30s^4)/T  => (60s - 180s^2 + 120s^3)/T^2
    return dq * (60.0 * s - 180.0 * s2 + 120.0 * s3) / (T * T)


def estimate_duration(
    q0: Sequence[float],
    qf: Sequence[float],
    *,
    vmax: float = 0.4,
    t_min: float = 0.5,
) -> float:
    """Duration from max |Δq|/vmax, floored at t_min."""
    if vmax <= 0.0:
        raise ValueError("vmax must be > 0")
    if t_min <= 0.0:
        raise ValueError("t_min must be > 0")
    if len(q0) != len(qf):
        raise ValueError("q0 and qf length mismatch")
    max_abs = 0.0
    for a, b in zip(q0, qf):
        max_abs = max(max_abs, abs(float(b) - float(a)))
    return max(t_min, max_abs / vmax)


@dataclass(frozen=True)
class Trajectory:
    """Synchronous multi-joint rest-to-rest trajectory."""

    q0: tuple[float, ...]
    qf: tuple[float, ...]
    duration: float

    def __post_init__(self) -> None:
        if self.duration <= 0.0:
            raise ValueError("duration must be > 0")
        if len(self.q0) != len(self.qf):
            raise ValueError("q0 and qf length mismatch")
        if len(self.q0) == 0:
            raise ValueError("trajectory needs at least one joint")

    @property
    def n_joints(self) -> int:
        return len(self.q0)

    def sample(self, t: float) -> tuple[list[float], list[float]]:
        qs: list[float] = []
        dqs: list[float] = []
        for a, b in zip(self.q0, self.qf):
            q, dq = sample_quintic(a, b, self.duration, t)
            qs.append(q)
            dqs.append(dq)
        return qs, dqs

    def q(self, t: float) -> list[float]:
        return self.sample(t)[0]

    def dq(self, t: float) -> list[float]:
        return self.sample(t)[1]


def plan_joint_trajectory(
    q0: Sequence[float],
    qf: Sequence[float],
    duration: float | None = None,
    *,
    vmax: float = 0.4,
    t_min: float = 0.5,
) -> Trajectory:
    """Build a multi-joint trajectory with shared duration T."""
    if len(q0) != len(qf):
        raise ValueError("q0 and qf length mismatch")
    if not q0:
        raise ValueError("empty joint vector")
    if duration is None:
        T = estimate_duration(q0, qf, vmax=vmax, t_min=t_min)
    else:
        if duration <= 0.0:
            raise ValueError("duration must be > 0")
        T = float(duration)
    return Trajectory(
        q0=tuple(float(x) for x in q0),
        qf=tuple(float(x) for x in qf),
        duration=T,
    )
