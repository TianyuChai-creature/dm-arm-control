"""Single-arm (left or right) view onto a shared dual-arm Arm / MIT loop."""
from __future__ import annotations

from typing import TYPE_CHECKING, Sequence

from . import _core

if TYPE_CHECKING:
    from .arm import Arm


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
    def n_joints(self) -> int:
        return len(self.can_ids)

    def _require_can(self, can_id: int) -> None:
        if int(can_id) not in self._id_set:
            raise ValueError(
                f"can_id 0x{int(can_id):02X} is not on {self.side} arm "
                f"(allowed: {[f'0x{c:02X}' for c in self.can_ids]})"
            )

    def states(self):
        return [s for s in self._arm.states() if int(s.can_id) in self._id_set]

    def commands(self):
        """MIT commands for this limb only (motor order)."""
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
        if isinstance(target, dict):
            for can_id in target:
                self._require_can(can_id)
            self._arm.mit(target)
        else:
            self._require_can(target)
            self._arm.mit(target, kp=kp, kd=kd, q=q, dq=dq, tau=tau)

    def set_zero(self, can_id: int, persist: bool = True) -> None:
        self._require_can(can_id)
        self._arm.set_zero(can_id, persist=persist)

    def set_zero_all(self, persist: bool = True) -> None:
        for cid in self.can_ids:
            self._arm.set_zero(cid, persist=persist)
