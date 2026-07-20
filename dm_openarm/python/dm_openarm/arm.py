from __future__ import annotations

import time
from pathlib import Path

from . import _core


class Arm:
    def __init__(self, config: _core.ArmConfig):
        self._arm = _core.DmArm(config)
        self._loop = _core.MitLoopController(self._arm)

    @classmethod
    def from_yaml(cls, path: str | Path) -> "Arm":
        return cls(_core.load_arm_config(str(path)))

    def enable(self) -> None:
        self._arm.connect()

    def disable(self) -> None:
        self.stop_mit_loop()
        self._arm.disable()

    def states(self):
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
        """Start the MIT control loop.

        If *home* is True (default), all motors are first commanded to
        ``q=0`` and this method blocks until they are near zero or
        *zero_timeout* elapses. Set ``home=False`` to hold current
        commands (e.g. gravity-compensated hold at the present pose).
        """
        self._loop.start(hz)

        if not home:
            return

        states = self.states()
        self._loop.set_all_commands([
            _core.MitCommand(kp=12.0, kd=0.6, q=0.0, dq=0.0, tau=0.0)
            for _ in states
        ])

        pos_tol = 0.05  # radians
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

    # ── Gravity compensation ──────────────────────────────────────────

    def set_gravity_enabled(self, enabled: bool) -> None:
        """Enable/disable gravity feedforward inside the MIT loop.

        When enabled: ``tau_sent = cmd.tau + scale * g(q)``.
        """
        self._loop.set_gravity_enabled(enabled)

    def gravity_enabled(self) -> bool:
        return self._loop.gravity_enabled()

    def set_gravity_scale(self, scale: float) -> None:
        """Scale factor on gravity torques (1.0 = full model)."""
        self._loop.set_gravity_scale(scale)

    def gravity_scale(self) -> float:
        return self._loop.gravity_scale()

    def set_gravity_use_measured_q(self, use_measured: bool) -> None:
        """If True, g(q) uses measured positions; else MIT command q."""
        self._loop.set_gravity_use_measured_q(use_measured)

    def gravity_torques(self, q: list[float] | None = None) -> list[float]:
        """Return gravity torques for joint positions (motor order).

        If *q* is None, use current measured positions from :meth:`states`.
        """
        if q is None:
            q = [s.position for s in self.states()]
        return list(self._loop.gravity_torques(q))

    # ── The unified MIT API ───────────────────────────────────────────

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
        """Send MIT command(s) with full parameter control.

        **Single motor** — pass CAN ID + keyword arguments::

            arm.mit(0x01, kp=12.0, kd=0.6, q=0.8, dq=0.0, tau=0.5)

        **Multiple motors** — pass a dict of CAN ID → MitCommand::

            from dm_openarm import MitCommand
            arm.mit({
                0x01: MitCommand(kp=12.0, kd=0.6, q=0.8),
                0x04: MitCommand(kp=10.0, kd=0.8, q=0.5, tau=1.0),
            })

        Non-targeted motors keep whatever command was previously set.
        When gravity compensation is enabled, ``tau`` is *additional*
        feedforward on top of ``g(q)``.
        """
        if isinstance(target, dict):
            for can_id, cmd in target.items():
                self._loop.set_command(can_id, cmd)
        else:
            self._loop.set_command(
                target,
                _core.MitCommand(kp=kp, kd=kd, q=q, dq=dq, tau=tau),
            )

    # ── Context manager ───────────────────────────────────────────────

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
