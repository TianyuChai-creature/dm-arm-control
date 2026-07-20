"""Fit per-joint gravity model: tau = amp * sin(q + phase) + bias.

Rewritten as linear LS:
  tau = A * sin(q) + B * cos(q) + C
  amp = hypot(A, B), phase = atan2(B, A), bias = C
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


@dataclass
class JointGravityFit:
    amp: float
    phase: float
    bias: float
    rmse: float
    n: int

    def as_yaml_map(self) -> str:
        return (
            f"{{ amp: {self.amp:.6f}, phase: {self.phase:.6f}, "
            f"bias: {self.bias:.6f} }}"
        )


def fit_joint_sin(q: Sequence[float], tau: Sequence[float]) -> JointGravityFit:
    """Least-squares fit for one joint. Needs >= 3 samples (more is better)."""
    n = len(q)
    if n != len(tau):
        raise ValueError("q and tau length mismatch")
    if n < 3:
        raise ValueError("need at least 3 samples to fit amp/phase/bias")

    # Normal equations for [A, B, C]
    s_ss = s_sc = s_s = s_cc = s_c = s_1 = 0.0
    s_ts = s_tc = s_t = 0.0
    for qi, ti in zip(q, tau):
        s = math.sin(qi)
        c = math.cos(qi)
        s_ss += s * s
        s_sc += s * c
        s_s += s
        s_cc += c * c
        s_c += c
        s_1 += 1.0
        s_ts += ti * s
        s_tc += ti * c
        s_t += ti

    # Solve 3x3 system M x = b
    # [[ss, sc, s], [sc, cc, c], [s, c, n]] [A,B,C]^T = [ts, tc, t]^T
    m = [
        [s_ss, s_sc, s_s],
        [s_sc, s_cc, s_c],
        [s_s, s_c, s_1],
    ]
    b = [s_ts, s_tc, s_t]
    a, bb, c = _solve_3x3(m, b)

    amp = math.hypot(a, bb)
    phase = math.atan2(bb, a) if amp > 1e-12 else 0.0
    bias = c

    sse = 0.0
    for qi, ti in zip(q, tau):
        pred = amp * math.sin(qi + phase) + bias
        err = ti - pred
        sse += err * err
    rmse = math.sqrt(sse / n)
    return JointGravityFit(amp=amp, phase=phase, bias=bias, rmse=rmse, n=n)


def fit_all_joints(
    samples_q: Sequence[Sequence[float]],
    samples_tau: Sequence[Sequence[float]],
) -> list[JointGravityFit]:
    """samples_q[k] = joint angles at sample k; samples_tau[k] same order."""
    if not samples_q or len(samples_q) != len(samples_tau):
        raise ValueError("samples_q/tau empty or length mismatch")
    n_joints = len(samples_q[0])
    for q, t in zip(samples_q, samples_tau):
        if len(q) != n_joints or len(t) != n_joints:
            raise ValueError("inconsistent joint count across samples")

    fits: list[JointGravityFit] = []
    for j in range(n_joints):
        qj = [row[j] for row in samples_q]
        tj = [row[j] for row in samples_tau]
        fits.append(fit_joint_sin(qj, tj))
    return fits


def format_gravity_yaml(
    fits: Sequence[JointGravityFit],
    *,
    enabled: bool = True,
    scale: float = 1.0,
    use_measured_q: bool = True,
    comments: Sequence[str] | None = None,
) -> str:
    lines = [
        "gravity:",
        f"  enabled: {'true' if enabled else 'false'}",
        f"  scale: {scale}",
        f"  use_measured_q: {'true' if use_measured_q else 'false'}",
        "  joints:",
    ]
    for i, fit in enumerate(fits):
        comment = ""
        if comments and i < len(comments):
            comment = f"  # {comments[i]}"
        lines.append(f"    - {fit.as_yaml_map()}{comment}")
    return "\n".join(lines) + "\n"


def _solve_3x3(m: list[list[float]], b: list[float]) -> tuple[float, float, float]:
    """Gaussian elimination with partial pivoting."""
    a = [row[:] + [b[i]] for i, row in enumerate(m)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(a[r][col]))
        if abs(a[pivot][col]) < 1e-12:
            raise ValueError("singular fit matrix; move joints through larger angle range")
        a[col], a[pivot] = a[pivot], a[col]
        div = a[col][col]
        for j in range(col, 4):
            a[col][j] /= div
        for row in range(3):
            if row == col:
                continue
            factor = a[row][col]
            for j in range(col, 4):
                a[row][j] -= factor * a[col][j]
    return a[0][3], a[1][3], a[2][3]
