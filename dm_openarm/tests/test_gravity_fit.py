"""Offline unit tests for gravity_fit (no hardware)."""
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from dm_openarm.gravity_fit import (
    DEFAULT_COUPLED_BASIS,
    build_phi,
    fit_all_joints,
    fit_coupled_all,
    fit_joint_sin,
    format_coupled_yaml,
    format_gravity_yaml,
    predict_coupled,
)


def test_exact_sin_recovery() -> None:
    amp, phase, bias = 2.5, 0.3, -0.1
    qs = [i * 0.4 - 1.2 for i in range(10)]
    taus = [amp * math.sin(q + phase) + bias for q in qs]
    fit = fit_joint_sin(qs, taus)
    assert abs(fit.amp - amp) < 1e-6
    assert abs(fit.phase - phase) < 1e-6
    assert abs(fit.bias - bias) < 1e-6
    assert fit.rmse < 1e-9


def test_multi_joint_decoupled() -> None:
    params = [(1.0, 0.0, 0.0), (0.5, 0.2, 0.1)]
    samples_q = []
    samples_tau = []
    for k in range(8):
        q = [0.3 * k - 1.0, -0.2 * k + 0.5]
        tau = [
            params[0][0] * math.sin(q[0] + params[0][1]) + params[0][2],
            params[1][0] * math.sin(q[1] + params[1][1]) + params[1][2],
        ]
        samples_q.append(q)
        samples_tau.append(tau)
    fits = fit_all_joints(samples_q, samples_tau)
    assert abs(fits[0].amp - 1.0) < 1e-6
    assert abs(fits[1].amp - 0.5) < 1e-6
    text = format_gravity_yaml(fits, enabled=True)
    assert "mode: decoupled" in text


def test_build_phi_known() -> None:
    q = [0.1, 0.2, 0.3, 0.4, 0.5]
    phi = build_phi(q, ["one", "sin_q3", "sin_q3_q4"])
    assert abs(phi[0] - 1.0) < 1e-15
    assert abs(phi[1] - math.sin(0.4)) < 1e-15
    assert abs(phi[2] - math.sin(0.4 + 0.5)) < 1e-15


def test_coupled_recovery() -> None:
    basis = DEFAULT_COUPLED_BASIS
    # true weights for joints 0 and 4
    w0 = [0.1] + [0.0] * (len(basis) - 1)
    w0[1] = 1.5  # sin_q0
    w0[6] = 0.3  # cos_q0
    w4 = [0.05] + [0.0] * (len(basis) - 1)
    w4[5] = 2.0  # sin_q4
    w4[11] = 0.8  # sin_q3_q4
    true_w = [w0, [0.0] * len(basis), [0.0] * len(basis), [0.0] * len(basis), w4]

    # Independent random-ish samples (avoid collinear joint schedules)
    samples_q = []
    samples_tau = []
    for i in range(40):
        q = [
            math.sin(i * 0.7) * 1.5,
            math.cos(i * 0.5) * 1.2,
            math.sin(i * 1.1 + 0.3) * 0.8,
            math.cos(i * 0.9 + 1.0) * 1.4,
            math.sin(i * 0.4 + 2.0) * 1.6,
        ]
        tau = [predict_coupled(q, true_w[j], basis) for j in range(5)]
        samples_q.append(q)
        samples_tau.append(tau)

    result = fit_coupled_all(samples_q, samples_tau, basis=basis, ridge=1e-10)
    # Prediction accuracy (primary); weights may not be unique if basis collinear
    for j in range(5):
        assert result.joints[j].rmse < 1e-5, (j, result.joints[j].rmse)
        for q, tau_row in zip(samples_q, samples_tau):
            pred = predict_coupled(q, result.joints[j].weights, basis)
            assert abs(pred - tau_row[j]) < 1e-4

    # Held-out poses
    for i in range(10):
        q = [0.1 * i, -0.2 * i, 0.05, 0.3 * math.sin(i), -0.4]
        for j in range(5):
            true = predict_coupled(q, true_w[j], basis)
            pred = predict_coupled(q, result.joints[j].weights, basis)
            assert abs(true - pred) < 1e-3, (i, j, true, pred)

    yaml_text = format_coupled_yaml(result, scale=0.9, comments=[f"j{i}" for i in range(5)])
    assert "mode: coupled" in yaml_text
    assert "sin_q3_q4" in yaml_text
    assert "weights:" in yaml_text


if __name__ == "__main__":
    test_exact_sin_recovery()
    test_multi_joint_decoupled()
    test_build_phi_known()
    test_coupled_recovery()
    print("test_gravity_fit OK")
