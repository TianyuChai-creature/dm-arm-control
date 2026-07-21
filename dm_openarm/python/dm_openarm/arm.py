from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

from . import _core
from .trajectory import plan_joint_trajectory


@dataclass(frozen=True)
class MoveResult:
    """Outcome of a time-boxed joint move (not a hard position gate)."""

    duration: float
    q_cmd: list[float]
    q_meas: list[float]
    err: list[float]
    max_abs_err: float


def _default_gains(n: int) -> tuple[list[float], list[float]]:
    """Per-joint kp/kd: wrist-class softer large motors last if n==5."""
    kp = [12.0] * n
    kd = [0.6] * n
    if n >= 5:
        kp[3] = kp[4] = 10.0
        kd[3] = kd[4] = 0.8
    elif n >= 2:
        # treat last joint(s) as larger if unknown
        kp[-1] = 10.0
        kd[-1] = 0.8
    return kp, kd


def _as_gain_list(value: float | Sequence[float] | None, n: int, default: list[float]) -> list[float]:
    if value is None:
        return list(default)
    if isinstance(value, (int, float)):
        return [float(value)] * n
    out = [float(x) for x in value]
    if len(out) != n:
        raise ValueError(f"gain length {len(out)} != n_joints {n}")
    return out


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
        """Instantly set one motor's MIT target (no interpolation, non-blocking)."""
        self.mit(can_id, kp=kp, kd=kd, q=q, dq=dq, tau=tau)

    def hold_at(
        self,
        can_id: int,
        *,
        q: float,
        kp: float = 12.0,
        kd: float = 0.6,
    ) -> None:
        """Hold one motor at *q* with soft PD (alias of :meth:`move_to`, dq=0)."""
        self.move_to(can_id, q=q, kp=kp, kd=kd, dq=0.0, tau=0.0)

    def move_joints(
        self,
        q_goal: Sequence[float] | Mapping[int, float],
        duration: float | None = None,
        *,
        vmax: float = 0.4,
        rate_hz: float = 100.0,
        kp: float | Sequence[float] | None = None,
        kd: float | Sequence[float] | None = None,
        settle_s: float = 0.2,
        gravity: bool | None = None,
    ) -> MoveResult:
        """Time-boxed multi-joint rest-to-rest move with quintic (q, dq).

        Streams targets into the running MIT loop; gravity (if enabled) is still
        applied in C++ each cycle. Completion is **by planned duration**, not
        by position tolerance — soft MIT + residual gravity may leave a steady
        lag. Returns a :class:`MoveResult` with measured error for logging.
        """
        if not self.mit_loop_running:
            raise RuntimeError("MIT loop is not running; call start_mit_loop() first")
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be > 0")
        if settle_s < 0.0:
            raise ValueError("settle_s must be >= 0")

        states = self.states()
        can_ids = [int(s.can_id) for s in states]
        q0 = [float(s.position) for s in states]
        n = len(can_ids)
        if n == 0:
            raise RuntimeError("no motors in state snapshot")

        qf = list(q0)
        if isinstance(q_goal, Mapping):
            id_to_idx = {cid: i for i, cid in enumerate(can_ids)}
            for cid, val in q_goal.items():
                if int(cid) not in id_to_idx:
                    raise ValueError(f"unknown can_id in q_goal: {cid}")
                qf[id_to_idx[int(cid)]] = float(val)
        else:
            goal_list = [float(x) for x in q_goal]
            if len(goal_list) != n:
                raise ValueError(
                    f"q_goal length {len(goal_list)} must match motor count {n}"
                )
            qf = goal_list

        if gravity is not None:
            self.set_gravity_enabled(bool(gravity))

        traj = plan_joint_trajectory(q0, qf, duration=duration, vmax=vmax)
        kp_def, kd_def = _default_gains(n)
        kp_list = _as_gain_list(kp, n, kp_def)
        kd_list = _as_gain_list(kd, n, kd_def)

        dt = 1.0 / rate_hz
        t0 = time.monotonic()
        next_tick = t0
        T = traj.duration

        while True:
            now = time.monotonic()
            t = now - t0
            if t >= T:
                break
            qs, dqs = traj.sample(t)
            cmd_map = {
                can_ids[i]: _core.MitCommand(
                    kp=kp_list[i],
                    kd=kd_list[i],
                    q=qs[i],
                    dq=dqs[i],
                    tau=0.0,
                )
                for i in range(n)
            }
            self.mit(cmd_map)
            next_tick += dt
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)

        # Final hold at planned goal (dq = 0).
        hold_map = {
            can_ids[i]: _core.MitCommand(
                kp=kp_list[i],
                kd=kd_list[i],
                q=qf[i],
                dq=0.0,
                tau=0.0,
            )
            for i in range(n)
        }
        self.mit(hold_map)
        if settle_s > 0.0:
            time.sleep(settle_s)

        meas = [float(s.position) for s in self.states()]
        err = [m - c for m, c in zip(meas, qf)]
        max_abs = max((abs(e) for e in err), default=0.0)
        return MoveResult(
            duration=T,
            q_cmd=list(qf),
            q_meas=meas,
            err=err,
            max_abs_err=max_abs,
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
