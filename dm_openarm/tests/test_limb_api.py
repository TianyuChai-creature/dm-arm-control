"""Offline tests for left/right Limb API (no hardware)."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from dm_openarm.limb import Limb


def test_limb_rejects_foreign_can() -> None:
    class Stub:
        pass

    arm = Stub()
    limb = Limb(arm, "left", [0x00, 0x01], ["a", "b"])  # type: ignore[arg-type]
    try:
        limb._require_can(0x05)
        assert False, "expected ValueError"
    except ValueError as e:
        assert "0x05" in str(e)


if __name__ == "__main__":
    test_limb_rejects_foreign_can()
    print("test_limb_api OK")
