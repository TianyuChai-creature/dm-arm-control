"""Coupled gravity model fitting: tau = W * phi(q).

Matches C++ GravityModel / gravity_model.cpp basis names.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

# Default basis (must match C++ gravity_model.cpp names)
DEFAULT_COUPLED_BASIS: list[str] = [
    "one",
    "sin_q0",
    "sin_q1",
    "sin_q2",
    "sin_q3",
    "sin_q4",
    "cos_q0",
    "cos_q1",
    "cos_q2",
    "cos_q3",
    "cos_q4",
    "sin_q3_q4",
    "cos_q3_q4",
]


@dataclass
class CoupledJointFit:
    weights: list[float]
    rmse: float
    n: int


@dataclass
class CoupledFitResult:
    basis: list[str]
    joints: list[CoupledJointFit] = field(default_factory=list)

    @property
    def weights_matrix(self) -> list[list[float]]:
        return [j.weights for j in self.joints]


def build_phi(q: Sequence[float], basis: Sequence[str]) -> list[float]:
    """Evaluate basis functions at full-arm joint vector q (len >= 5 for default)."""
    if len(q) < 5:
        # pad for arms with fewer joints in tests
        q = list(q) + [0.0] * (5 - len(q))
    q0, q1, q2, q3, q4 = float(q[0]), float(q[1]), float(q[2]), float(q[3]), float(q[4])
    table = {
        "one": 1.0,
        "sin_q0": math.sin(q0),
        "sin_q1": math.sin(q1),
        "sin_q2": math.sin(q2),
        "sin_q3": math.sin(q3),
        "sin_q4": math.sin(q4),
        "cos_q0": math.cos(q0),
        "cos_q1": math.cos(q1),
        "cos_q2": math.cos(q2),
        "cos_q3": math.cos(q3),
        "cos_q4": math.cos(q4),
        "sin_q3_q4": math.sin(q3 + q4),
        "cos_q3_q4": math.cos(q3 + q4),
        "sin_q2_q3_q4": math.sin(q2 + q3 + q4),
        "cos_q2_q3_q4": math.cos(q2 + q3 + q4),
    }
    out: list[float] = []
    for name in basis:
        if name not in table:
            raise ValueError(f"unknown gravity basis name: {name}")
        out.append(table[name])
    return out


def predict_coupled(
    q: Sequence[float],
    weights: Sequence[float],
    basis: Sequence[str],
) -> float:
    phi = build_phi(q, basis)
    if len(phi) != len(weights):
        raise ValueError("weights length must match basis")
    return sum(w * p for w, p in zip(weights, phi))


def fit_coupled_joint(
    samples_q: Sequence[Sequence[float]],
    samples_tau: Sequence[float],
    basis: Sequence[str] = DEFAULT_COUPLED_BASIS,
    ridge: float = 1e-6,
) -> CoupledJointFit:
    """Ridge LS: min ||Phi w - tau||^2 + ridge ||w||^2."""
    n = len(samples_q)
    if n != len(samples_tau):
        raise ValueError("samples_q/tau length mismatch")
    if n < 2:
        raise ValueError("need at least 2 samples for coupled fit")
    k = len(basis)
    # Normal equations (Phi^T Phi + ridge I) w = Phi^T tau
    ata = [[0.0] * k for _ in range(k)]
    atb = [0.0] * k
    for q, tau in zip(samples_q, samples_tau):
        phi = build_phi(q, basis)
        for i in range(k):
            atb[i] += phi[i] * float(tau)
            for j in range(k):
                ata[i][j] += phi[i] * phi[j]
    for i in range(k):
        ata[i][i] += ridge
    w = _solve_linear(ata, atb)
    sse = 0.0
    for q, tau in zip(samples_q, samples_tau):
        err = float(tau) - predict_coupled(q, w, basis)
        sse += err * err
    rmse = math.sqrt(sse / n)
    return CoupledJointFit(weights=w, rmse=rmse, n=n)


def fit_coupled_all(
    samples_q: Sequence[Sequence[float]],
    samples_tau: Sequence[Sequence[float]],
    basis: Sequence[str] = DEFAULT_COUPLED_BASIS,
    ridge: float = 1e-6,
    prune_eps: float = 0.0,
) -> CoupledFitResult:
    if not samples_q or len(samples_q) != len(samples_tau):
        raise ValueError("samples_q/tau empty or length mismatch")
    n_joints = len(samples_tau[0])
    for q, t in zip(samples_q, samples_tau):
        if len(t) != n_joints:
            raise ValueError("inconsistent joint count in tau samples")
    joints: list[CoupledJointFit] = []
    for j in range(n_joints):
        tj = [row[j] for row in samples_tau]
        fit = fit_coupled_joint(samples_q, tj, basis=basis, ridge=ridge)
        if prune_eps > 0.0:
            fit.weights = [0.0 if abs(w) < prune_eps else w for w in fit.weights]
            # recompute rmse after prune
            sse = 0.0
            for q, tau in zip(samples_q, tj):
                err = float(tau) - predict_coupled(q, fit.weights, basis)
                sse += err * err
            fit.rmse = math.sqrt(sse / len(tj))
        joints.append(fit)
    return CoupledFitResult(basis=list(basis), joints=joints)


def format_coupled_yaml(
    result: CoupledFitResult,
    *,
    enabled: bool = True,
    scale: float = 0.9,
    use_measured_q: bool = True,
    comments: Sequence[str] | None = None,
    n_joints: int | None = None,
) -> str:
    del n_joints  # kept for call-site compatibility; rows come from result.joints
    lines = [
        "gravity:",
        f"  enabled: {'true' if enabled else 'false'}",
        f"  scale: {scale}",
        f"  use_measured_q: {'true' if use_measured_q else 'false'}",
        "  coupled:",
        "    basis:",
    ]
    for name in result.basis:
        lines.append(f"      - {name}")
    lines.append("    weights:")
    for i, joint in enumerate(result.joints):
        c = f"  # {comments[i]}" if comments and i < len(comments) else ""
        wstr = ", ".join(f"{w:.8f}" for w in joint.weights)
        lines.append(f"      - [{wstr}]{c}")
    return "\n".join(lines) + "\n"


def _solve_linear(a: list[list[float]], b: list[float]) -> list[float]:
    """Gaussian elimination with partial pivoting for n x n."""
    n = len(b)
    m = [a[i][:] + [b[i]] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-14:
            raise ValueError("singular coupled fit matrix; need more diverse poses")
        m[col], m[pivot] = m[pivot], m[col]
        div = m[col][col]
        for j in range(col, n + 1):
            m[col][j] /= div
        for row in range(n):
            if row == col:
                continue
            factor = m[row][col]
            for j in range(col, n + 1):
                m[row][j] -= factor * m[col][j]
    return [m[i][n] for i in range(n)]
