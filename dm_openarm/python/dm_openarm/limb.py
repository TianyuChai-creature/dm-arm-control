"""Single-arm (left or right) view onto a shared dual-arm Arm / MIT loop."""
from __future__ import annotations

import time
from typing import TYPE_CHECKING, Mapping, Sequence

from . import _core
from .trajectory import plan_joint_trajectory

if TYPE_CHECKING:
    from .arm import Arm, MoveResult


def _default_gains(n: int) -> tuple[list[float], list[float]]:
    kp = [12.0] * n
    kd = [0.6] * n
    if n >= 5:
        kp[3] = kp[4] = 10.0
        kd[3] = kd[4] = 0.8
    elif n >= 2:
        kp[-1] = 10.0
        kd[-1] = 0.8
    return kp, kd


def _as_gain_list(
    value: float | Sequence[float] | None, n: int, default: list[float]
) -> list[float]:
    if value is None:
        return list(default)
    if isinstance(value, (int, float)):
        return [float(value)] * n
    out = [float(x) for x in value]
    if len(out) != n:
        raise ValueError(f"gain length {len(out)} != n_joints {n}")
    return out


class Limb:
    """Control API scoped to one arm side (same methods as the old single-arm surface)."""

    def __init__(self, arm: "Arm", side: str, can_ids: Sequence[int], names: Sequence[str]):
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self._arm = arm
        self.side = side
        self.can_ids = [int(c) for c in can_ids]
        self.names = list(names)
        self._id_set = set(self.can_ids)

    @property
    def present(self) -> bool:
        return len(self.can_ids) > 0

    @property
    def n_joints(self) -> int:
        return len(self.can_ids)

    def _require_present(self) -> None:
        if not self.present:
            raise RuntimeError(f"{self.side} arm is not configured")

    def _require_can(self, can_id: int) -> None:
        self._require_present()
        if int(can_id) not in self._id_set:
            raise ValueError(
                f"can_id 0x{int(can_id):02X} is not on {self.side} arm "
                f"(allowed: {[f'0x{c:02X}' for c in self.can_ids]})"
            )

    def states(self):
        self._require_present()
        return [s for s in self._arm.states() if int(s.can_id) in self._id_set]

    def commands(self):
        """MIT commands for this limb only (motor order)."""
        self._require_present()
        all_cmds = self._arm.commands()
        all_states = self._arm.states()
        # commands() aligns with full motor table order
        can_order = [int(s.can_id) for s in all_states]
        out = []
        for cid in self.can_ids:
            try:
                idx = can_order.index(cid)
            except ValueError:
                continue
            if idx < len(all_cmds):
                out.append(all_cmds[idx])
        return out

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
        self._require_present()
        if isinstance(target, dict):
            for can_id in target:
                self._require_can(can_id)
            self._arm.mit(target)
        else:
            self._require_can(target)
            self._arm.mit(target, kp=kp, kd=kd, q=q, dq=dq, tau=tau)

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

    def set_zero(self, can_id: int, persist: bool = True) -> None:
        self._require_can(can_id)
        self._arm.set_zero(can_id, persist=persist)

    def set_zero_all(self, persist: bool = True) -> None:
        self._require_present()
        for cid in self.can_ids:
            self._arm.set_zero(cid, persist=persist)

    def set_gravity_enabled(self, enabled: bool) -> None:
        self._require_present()
        self._arm._loop.set_limb_gravity_enabled(self.side, bool(enabled))

    def gravity_enabled(self) -> bool:
        self._require_present()
        return bool(self._arm._loop.limb_gravity_enabled(self.side))

    def set_gravity_scale(self, scale: float) -> None:
        self._require_present()
        self._arm._loop.set_limb_gravity_scale(self.side, float(scale))

    def gravity_scale(self) -> float:
        self._require_present()
        return float(self._arm._loop.limb_gravity_scale(self.side))

    def set_gravity_use_measured_q(self, use_measured: bool) -> None:
        self._require_present()
        self._arm._loop.set_limb_gravity_use_measured_q(self.side, bool(use_measured))

    def gravity_use_measured_q(self) -> bool:
        self._require_present()
        return bool(self._arm._loop.limb_gravity_use_measured_q(self.side))

    def gravity_torques(self, q: list[float] | None = None) -> list[float]:
        self._require_present()
        if q is None:
            q = [float(s.position) for s in self.states()]
        return list(self._arm._loop.limb_gravity_torques(self.side, list(q)))

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
    ) -> "MoveResult":
        """Time-boxed quintic for this limb only (does not touch the other side)."""
        from .arm import MoveResult

        self._require_present()
        if not self._arm.mit_loop_running:
            raise RuntimeError("MIT loop is not running; call arm.start_mit_loop() first")
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be > 0")
        if settle_s < 0.0:
            raise ValueError("settle_s must be >= 0")

        states = self.states()
        can_ids = [int(s.can_id) for s in states]
        # Ensure order matches self.can_ids
        by_id = {int(s.can_id): s for s in states}
        ordered = []
        for cid in self.can_ids:
            if cid not in by_id:
                raise RuntimeError(f"missing state for can_id 0x{cid:02X}")
            ordered.append(by_id[cid])
        can_ids = list(self.can_ids)
        q0 = [float(s.position) for s in ordered]
        n = len(can_ids)

        qf = list(q0)
        if isinstance(q_goal, Mapping):
            id_to_idx = {cid: i for i, cid in enumerate(can_ids)}
            for cid, val in q_goal.items():
                self._require_can(int(cid))
                qf[id_to_idx[int(cid)]] = float(val)
        else:
            goal_list = [float(x) for x in q_goal]
            if len(goal_list) != n:
                raise ValueError(
                    f"q_goal length {len(goal_list)} must match {self.side} joint count {n}"
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

        by_id = {int(s.can_id): s for s in self.states()}
        meas = [float(by_id[cid].position) for cid in can_ids]
        err = [m - c for m, c in zip(meas, qf)]
        max_abs = max((abs(e) for e in err), default=0.0)
        return MoveResult(
            duration=T,
            q_cmd=list(qf),
            q_meas=meas,
            err=err,
            max_abs_err=max_abs,
        )
