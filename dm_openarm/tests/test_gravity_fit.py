"""Offline unit test for gravity_fit (no hardware)."""
from __future__ import annotations

import math
import sys
from pathlib import Path

# Allow running without install: python tests/test_gravity_fit.py
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from dm_openarm.gravity_fit import fit_all_joints, fit_joint_sin, format_gravity_yaml


def test_exact_sin_recovery() -> None:
    amp, phase, bias = 2.5, 0.3, -0.1
    qs = [i * 0.4 - 1.2 for i in range(10)]
    taus = [amp * math.sin(q + phase) + bias for q in qs]
    fit = fit_joint_sin(qs, taus)
    assert abs(fit.amp - amp) < 1e-6
    assert abs(fit.phase - phase) < 1e-6
    assert abs(fit.bias - bias) < 1e-6
    assert fit.rmse < 1e-9


def test_multi_joint() -> None:
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
    assert "gravity:" in text
    assert "amp:" in text


if __name__ == "__main__":
    test_exact_sin_recovery()
    test_multi_joint()
    print("test_gravity_fit OK")
