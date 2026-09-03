from __future__ import annotations

import time
from pathlib import Path

from . import _core
from .limb import Limb


class Arm:
    """Device-level handle: one bus + one MIT loop; limbs via ``left`` / ``right``.

    Prefer::

        arm.right.mit(0x05, kp=12, kd=0.6, q=0.1)

    Root ``mit`` / ``states`` still address the full bus.
    """

    def __init__(self, config: _core.ArmConfig):
        self._config = config
        self._arm = _core.DmArm(config)
        self._loop = _core.MitLoopController(self._arm)

        motors = list(config.motors)

        def make(side: str, spec: _core.LimbSpec) -> Limb:
            begin = int(spec.begin)
            count = int(spec.count)
            slice_m = motors[begin : begin + count]
            return Limb(
                self,
                side,
                [int(m.can_id) for m in slice_m],
                [str(m.name) for m in slice_m],
            )

        self.left = make("left", config.left)
        self.right = make("right", config.right)

    @classmethod
    def from_yaml(cls, path: str | Path) -> "Arm":
        return cls(_core.load_arm_config(str(path)))

    def enable(self) -> None:
        self._arm.enable()

    def disable(self) -> None:
        try:
            self.stop_mit_loop()
        finally:
            try:
                self._arm.disable()
            finally:
                self._arm.disconnect()

    def states(self):
        """All motors on the bus (left then right). Prefer limb.states()."""
        return self._arm.states()

    def set_zero(self, can_id: int, persist: bool = True) -> None:
        self._arm.set_zero(can_id, persist)

    def set_zero_all(self, persist: bool = True) -> None:
        self._arm.set_zero_all(persist)

    def start_mit_loop(
        self,
        hz: float = _core.DEFAULT_CONTROL_HZ,
        zero_timeout: float = 5.0,
        *,
        home: bool = False,
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

    @property
    def deadline_misses(self) -> int:
        return int(self._loop.deadline_misses())

    def commands(self):
        return self._loop.commands()

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
