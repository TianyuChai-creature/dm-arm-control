from __future__ import annotations

import time
from pathlib import Path

from . import _core
from .limb import Limb
from .calibration import _Calibration


class Arm:
    """Device-level handle: one bus + one MIT loop; limbs via ``left`` / ``right``.

    Prefer::

        arm.right.mit(0x05, kp=12, kd=0.6, q=0.1)

    Root ``mit`` / ``states`` still address the full bus.
    """

    def __init__(self, config: _core.ArmConfig):
        self._calibration = _Calibration(self)
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
    def from_yaml(cls, path: str | Path, *, side: str | None = None) -> "Arm":
        config = _core.load_arm_config(str(path))
        if side is None:
            return cls(config)
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        config.bus_motors = list(config.motors)
        source = config.left if side == "left" else config.right
        config.motors = list(config.motors)[source.begin : source.begin + source.count]
        selected = _core.LimbSpec()
        selected.count = len(config.motors)
        empty = _core.LimbSpec()
        config.left = selected if side == "left" else empty
        config.right = selected if side == "right" else empty
        return cls(config)

    def enable(self) -> None:
        """Dispatch enable frames. Use enable_seeded for fresh status-1 confirmation."""
        self._arm.enable()

    def connect_passive(self) -> None:
        """Open the USB bus without enabling any motor."""
        self._arm.connect()

    def probe_status(self, timeout_s: float = 0.2) -> list[int]:
        """Return IDs with 0xCC-format replies observed after dispatch.

        The protocol has no request nonce, so a delayed older reply is
        indistinguishable from a reply to this call.
        """
        return self._arm.probe_status(timeout_s)

    def disable(self) -> None:
        loop_error = None
        disable_error = None
        try:
            self.stop_mit_loop()
        except Exception as exc:
            loop_error = exc
        try:
            self._arm.disable()
        except Exception as exc:
            disable_error = exc
        disconnect_error = None
        if disable_error is None:
            try:
                self._arm.disconnect()
            except Exception as exc:
                disconnect_error = exc
        if disable_error is not None or disconnect_error is not None:
            cause = disable_error if disable_error is not None else disconnect_error
            detail = f"shutdown unknown: {cause}"
            if loop_error is not None:
                detail += f"; loop stop failed: {loop_error}"
            raise RuntimeError(detail) from cause
        if loop_error is not None:
            raise loop_error

    def enable_seeded(self, gains, *, hz=250.0, command_timeout=0.1, feedback_timeout=0.2):
        """Enable and seed; return after the first complete hold batch is submitted."""
        self._loop.enable_seeded(gains, hz, command_timeout, feedback_timeout)

    def start_seeded(self, gains, *, hz=250.0, command_timeout=0.1, feedback_timeout=0.2):
        """Native feedback acquisition/seed; poll safety_state until HOLD."""
        self._loop.start_seeded(gains, hz, command_timeout, feedback_timeout)

    @property
    def safety_state(self):
        return self._loop.safety_state()

    @property
    def fault(self):
        return self._loop.fault()

    @property
    def accepted_sequence(self):
        return self._loop.accepted_sequence()

    @property
    def sent_sequence(self):
        return self._loop.sent_sequence()

    def hold(self, *, reset_fault=False):
        """Return after a fresh-position hold batch is submitted, or raise."""
        self._loop.hold(reset_fault)

    def hold_command(self):
        """Submit and confirm the last stationary command, including feedforward."""
        return self._loop.hold_command()

    def states(self):
        """All motors on the bus (left then right). Prefer limb.states()."""
        return self._arm.states()

    def set_zero(self, can_id: int, persist: bool = True) -> None:
        if self._loop.running():
            raise RuntimeError("set_zero requires a stopped MIT loop")
        self._arm.set_zero(can_id, persist)

    def set_zero_all(self, persist: bool = True) -> None:
        if self._loop.running():
            raise RuntimeError("set_zero_all requires a stopped MIT loop")
        self._arm.set_zero_all(persist)

    def calibrate_zero(self, *, kp=None, kd=None, gains=None):
        """Scan both arms, park at measured zeros, then persist all ten axes.

        Defaults: kp=120 on CAN 0/1/5/6, 60 on other axes, kd=5, dq=tau=0. Fast approach requires
        trustworthy existing zeros; 12s per joint and 120s total are timing
        targets reported as time_target_exceeded, never stop conditions.
        Requires clear paths, mechanical support during disabled Flash writes, and station commissioning.
        Failure with healthy feedback leaves a latched hold; inspect the report
        before explicitly stopping/disabling. Raises CalibrationError with .report.
        """
        return self._calibration.run(kp=kp, kd=kd, gains=gains)

    def calibration_status(self):
        """Return an independent snapshot, including partial Flash results."""
        return self._calibration.status()

    def cancel_zero_calibration(self):
        """Request cancellation; healthy feedback enters a latched hold."""
        self._loop._cancel_calibration()

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
        self._loop.stop()

    @property
    def mit_loop_running(self) -> bool:
        return self._loop.running()

    @property
    def deadline_misses(self) -> int:
        return int(self._loop.deadline_misses())

    def timing_stats(self):
        """Return one-second rolling TX and per-motor RX timing statistics."""
        return self._loop.timing_stats()

    def sent_commands(self):
        return self._loop.sent_commands()

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
            # Keep the Python ID check; merge with the current table under the native lock.
            ids = [int(m.can_id) for m in self._config.motors]
            if any(int(cid) not in ids for cid in target):
                raise ValueError("unknown motor CAN ID")
            self._loop.set_commands(target)
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
