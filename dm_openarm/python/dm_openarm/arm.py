from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

from . import _core
from .limb import Limb


@dataclass(frozen=True)
class MoveResult:
    """Outcome of a time-boxed joint move (not a hard position gate)."""

    duration: float
    q_cmd: list[float]
    q_meas: list[float]
    err: list[float]
    max_abs_err: float


class Arm:
    """Device-level handle: one bus + one MIT loop; limbs via ``left`` / ``right``.

    Prefer::

        arm.left.move_joints(...)
        arm.right.mit(0x21, kp=12, kd=0.6, q=0.1)

    Root ``mit`` / ``states`` still address the full bus. Root ``move_joints``
    only works for single-arm (left-only) configs.
    """

    def __init__(self, config: _core.ArmConfig):
        self._config = config
        self._arm = _core.DmArm(config)
        self._loop = _core.MitLoopController(self._arm)

        motors = list(config.motors)
        lb = int(self._loop.left_begin())
        lc = int(self._loop.left_count())
        rb = int(self._loop.right_begin())
        rc = int(self._loop.right_count())

        def make(side: str, begin: int, count: int) -> Limb | None:
            if count <= 0:
                return None
            slice_m = motors[begin : begin + count]
            return Limb(
                self,
                side,
                [int(m.can_id) for m in slice_m],
                [str(m.name) for m in slice_m],
            )

        self.left: Limb | None = make("left", lb, lc)
        self.right: Limb | None = make("right", rb, rc)

    @classmethod
    def from_yaml(cls, path: str | Path) -> "Arm":
        return cls(_core.load_arm_config(str(path)))

    def is_dual(self) -> bool:
        return (
            self.left is not None
            and self.left.present
            and self.right is not None
            and self.right.present
        )

    def enable(self) -> None:
        self._arm.connect()

    def disable(self) -> None:
        self.stop_mit_loop()
        self._arm.disable()

    def states(self):
        """All motors on the bus (left then right). Prefer limb.states()."""
        return self._arm.states()

    def set_zero(self, can_id: int, persist: bool = True) -> None:
        self._arm.set_zero(can_id, persist)

    def set_zero_all(self, persist: bool = True) -> None:
        self._arm.set_zero_all(persist)

    def start_mit_loop(
        self,
        hz: float = 1000.0,
        zero_timeout: float = 5.0,
        *,
        home: bool = True,
    ) -> None:
        """Start the shared MIT loop for **all** motors on the bus.

        Prefer ``home=False`` on dual-arm stations (homing zeros every joint).
        """
        self._loop.start(hz)

        if not home:
            return

        states = self.states()
        self._loop.set_all_commands([
            _core.MitCommand(kp=12.0, kd=0.6, q=0.0, dq=0.0, tau=0.0)
            for _ in states
        ])

        pos_tol = 0.05
        deadline = time.monotonic() + zero_timeout
        while True:
            states = self.states()
            if all(abs(s.position) < pos_tol for s in states):
                break
            if time.monotonic() >= deadline:
                break
            time.sleep(0.01)

    def stop_mit_loop(self) -> None:
        if self._loop.running():
            self._loop.stop()

    @property
    def mit_loop_running(self) -> bool:
        return self._loop.running()

    def commands(self):
        return self._loop.commands()

    # ── Gravity (legacy = left limb) ─────────────────────────────────

    def set_gravity_enabled(self, enabled: bool) -> None:
        """Legacy: left gravity only. Prefer ``arm.left.set_gravity_enabled``."""
        self._loop.set_gravity_enabled(enabled)

    def gravity_enabled(self) -> bool:
        return self._loop.gravity_enabled()

    def set_gravity_scale(self, scale: float) -> None:
        self._loop.set_gravity_scale(scale)

    def gravity_scale(self) -> float:
        return self._loop.gravity_scale()

    def set_gravity_use_measured_q(self, use_measured: bool) -> None:
        self._loop.set_gravity_use_measured_q(use_measured)

    def gravity_torques(self, q: list[float] | None = None) -> list[float]:
        """Legacy left-limb gravity. Prefer ``arm.left.gravity_torques``."""
        if q is None:
            if self.left is None:
                raise RuntimeError("no left arm")
            q = [s.position for s in self.left.states()]
        return list(self._loop.gravity_torques(q))

    # ── Bus-level MIT ────────────────────────────────────────────────

    def mit(
        self,
        target: int | dict[int, _core.MitCommand],
        /,
        kp: float = 0.0,
        kd: float = 0.0,
        q: float = 0.0,
        dq: float = 0.0,
        tau: float = 0.0,
    ) -> None:
        """Bus-level MIT (any can_id). Prefer ``arm.left.mit`` / ``arm.right.mit``."""
        if isinstance(target, dict):
            for can_id, cmd in target.items():
                self._loop.set_command(can_id, cmd)
        else:
            self._loop.set_command(
                target,
                _core.MitCommand(kp=kp, kd=kd, q=q, dq=dq, tau=tau),
            )

    def move_to(
        self,
        can_id: int,
        *,
        q: float,
        kp: float = 12.0,
        kd: float = 0.6,
        dq: float = 0.0,
        tau: float = 0.0,
    ) -> None:
        self.mit(can_id, kp=kp, kd=kd, q=q, dq=dq, tau=tau)

    def hold_at(
        self,
        can_id: int,
        *,
        q: float,
        kp: float = 12.0,
        kd: float = 0.6,
    ) -> None:
        self.move_to(can_id, q=q, kp=kp, kd=kd, dq=0.0, tau=0.0)

    def move_joints(self, *args, **kwargs) -> MoveResult:
        """Not available on Arm — use arm.left / arm.right.move_joints."""
        raise RuntimeError(
            "use arm.left.move_joints(...) or arm.right.move_joints(...) "
            "(dual-arm only; root move_joints removed)"
        )

    def __enter__(self) -> "Arm":
        self.enable()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disable()

    def __del__(self) -> None:
        try:
            self.disable()
        except Exception:
            pass
