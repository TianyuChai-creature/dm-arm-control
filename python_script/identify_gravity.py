#!/usr/bin/env python3
"""交互式重力参数辨识。

流程：
  u      — UNLOCK 跟手（双手托住重量再掰）
  l      — LOCK 钉住当前测量角（可松手）
  Enter  — 在【已有锁定目标】上采样；若当前是 UNLOCK 则先锁再采
  f      — 拟合
  q      — 放弃

【Enter 松弛的原因（已修）】
  旧逻辑在 Enter 时把 q_des 改成「当前测量角」。
  锁住后臂因重力略下沉时，测量角 ≠ 锁定目标，存在 kp*e 在扛重力；
  一改 q_des=测量角，误差 e→0，弹簧力消失 → 感觉突然松弛再塌。
  现：已 LOCK 时 Enter 保持原 q_des，只提高 kp 采样。
"""
from __future__ import annotations

import threading
import time
from pathlib import Path

from dm_openarm import Arm, MitCommand
from dm_openarm.gravity_fit import fit_all_joints, format_gravity_yaml

CONFIG = "dm_openarm/config/arm_5dof.yaml"
SETTLE_S = 1.0
SAMPLE_HZ = 50.0

# UNLOCK 跟手
KP_U_S, KD_U_S = 0.2, 0.25
KP_U_L, KD_U_L = 0.15, 0.3

# LOCK 保持（抗重力）
KP_L_S, KD_L_S = 22.0, 1.0
KP_L_L, KD_L_L = 18.0, 1.2

# 采样（在同一 q_des 上加硬，不改目标）
KP_S_S, KD_S_S = 30.0, 1.2
KP_S_L, KD_S_L = 24.0, 1.4


def g_unlock(cid: int) -> tuple[float, float]:
    return (KP_U_L, KD_U_L) if cid >= 0x04 else (KP_U_S, KD_U_S)


def g_lock(cid: int) -> tuple[float, float]:
    return (KP_L_L, KD_L_L) if cid >= 0x04 else (KP_L_S, KD_L_S)


def g_sample(cid: int) -> tuple[float, float]:
    return (KP_S_L, KD_S_L) if cid >= 0x04 else (KP_S_S, KD_S_S)


class HoldController:
    def __init__(self, arm: Arm):
        self.arm = arm
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._mu = threading.Lock()
        self.locked = True
        self._q_des: dict[int, float] = {}
        self._sample_mode = False

    def start(self) -> None:
        states = self.arm.states()
        with self._mu:
            self._q_des = {s.can_id: s.position for s in states}
            self.locked = True
            self._sample_mode = False
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="hold", daemon=True)
        self._thread.start()
        self._push_now()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def unlock(self) -> None:
        with self._mu:
            self.locked = False
            self._sample_mode = False
        self._push_now()
        print(
            "  → UNLOCK：双手托住再掰。到位后按 l 锁定，再 Enter 采样。",
            flush=True,
        )

    def lock_here(self) -> None:
        states = self.arm.states()
        with self._mu:
            self._q_des = {s.can_id: s.position for s in states}
            self.locked = True
            self._sample_mode = False
            q_des = dict(self._q_des)
        self._push_now()
        print(
            "  → LOCK 目标: "
            + " ".join(f"0x{cid:02X}={q:+.3f}" for cid, q in sorted(q_des.items())),
            flush=True,
        )

    def begin_sample(self) -> None:
        """Raise kp for sampling. Do NOT move q_des if already locked."""
        states = self.arm.states()
        with self._mu:
            if not self.locked or not self._q_des:
                # UNLOCK 时 Enter：先以当前角建锁
                self._q_des = {s.can_id: s.position for s in states}
            # 已 LOCK：保留原 _q_des，避免 e→0 导致突然松弛
            self.locked = True
            self._sample_mode = True
            q_des = dict(self._q_des)
        self._push_now()
        print(
            "  → SAMPLE 保持原锁定角并加硬: "
            + " ".join(f"0x{cid:02X}={q:+.3f}" for cid, q in sorted(q_des.items())),
            flush=True,
        )

    def end_sample(self) -> None:
        with self._mu:
            self._sample_mode = False
            # 仍 LOCK，q_des 不变
        self._push_now()

    def _push_now(self) -> None:
        """Synchronously send one MIT frame (don't wait for background tick)."""
        try:
            states = self.arm.states()
            with self._mu:
                locked = self.locked
                sample = self._sample_mode
                q_des = dict(self._q_des)
                if not locked:
                    q_des = {s.can_id: s.position for s in states}
                    self._q_des = q_des
            cmds = {}
            for s in states:
                cid = s.can_id
                if sample:
                    kp, kd = g_sample(cid)
                elif locked:
                    kp, kd = g_lock(cid)
                else:
                    kp, kd = g_unlock(cid)
                cmds[cid] = MitCommand(
                    kp=kp, kd=kd, q=q_des.get(cid, s.position), dq=0.0, tau=0.0
                )
            self.arm.mit(cmds)
        except Exception:
            pass

    def _loop(self) -> None:
        while not self._stop.is_set():
            self._push_now()
            time.sleep(0.02)


def average_state(arm: Arm, duration_s: float) -> tuple[list[float], list[float]]:
    n = 0
    q_sum: list[float] | None = None
    t_sum: list[float] | None = None
    dt = 1.0 / SAMPLE_HZ
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < duration_s:
        states = arm.states()
        q = [s.position for s in states]
        tau = [s.torque for s in states]
        if q_sum is None:
            q_sum = [0.0] * len(q)
            t_sum = [0.0] * len(tau)
        for i in range(len(q)):
            q_sum[i] += q[i]
            t_sum[i] += tau[i]
        n += 1
        time.sleep(dt)
    assert q_sum is not None and t_sum is not None and n > 0
    return [v / n for v in q_sum], [v / n for v in t_sum]


def main() -> int:
    print("=== 重力辨识 identify_gravity ===", flush=True)
    print(f"config: {CONFIG}", flush=True)
    print(
        f"""
操作（请严格按序）：

  1. 启动后默认 LOCK。
  2. u  — 解锁跟手（双手托住重量再掰）。
  3. l  — 锁定当前角（可松手检查）。
  4. Enter — 在锁定目标上加硬采样（不再改目标角，避免突然松弛）。
  5. 重复；肩/肘要有大角度。f 拟合 / q 放弃。

采样时长 {SETTLE_S:.1f}s。
""",
        flush=True,
    )

    arm = Arm.from_yaml(CONFIG)
    ctrl: HoldController | None = None
    samples_q: list[list[float]] = []
    samples_tau: list[list[float]] = []
    can_ids: list[int] = []
    names: list[str] = []

    try:
        arm.enable()
        arm.set_gravity_enabled(False)
        for _ in range(30):
            arm._arm.send_zero_mit_all()
            time.sleep(0.01)

        states = arm.states()
        can_ids = [s.can_id for s in states]
        names = [f"0x{cid:02X}" for cid in can_ids]

        arm.start_mit_loop(hz=1000.0, home=False)
        ctrl = HoldController(arm)
        ctrl.start()
        time.sleep(0.2)
        print("已 LOCK 启动姿态。移动请先 u。", flush=True)

        while True:
            st = "LOCK" if (ctrl and ctrl.locked) else "UNLOCK"
            try:
                line = input(f"[{len(samples_q)} samples | {st}] > ").strip().lower()
            except EOFError:
                line = "q"

            if line in ("q", "quit", "exit"):
                print("已放弃。", flush=True)
                return 1
            if line in ("f", "fit", "done"):
                break
            if line in ("u", "unlock", "move"):
                assert ctrl is not None
                ctrl.unlock()
                continue
            if line in ("l", "lock"):
                assert ctrl is not None
                ctrl.lock_here()
                continue
            if line not in ("", "s", "sample"):
                print("命令: u / l / Enter / f / q", flush=True)
                continue

            assert ctrl is not None
            if not ctrl.locked:
                print("  （当前 UNLOCK，先按当前角建锁再采样）", flush=True)
            ctrl.begin_sample()
            print(f"  采样 {SETTLE_S:.1f}s …", flush=True)
            time.sleep(0.1)
            q_mean, tau_mean = average_state(arm, SETTLE_S)
            samples_q.append(q_mean)
            samples_tau.append(tau_mean)
            preview = " ".join(
                f"0x{cid:02X}:q={qq:+.3f},τ={tt:+.3f}"
                for cid, qq, tt in zip(can_ids, q_mean, tau_mean)
            )
            print(f"  sample#{len(samples_q)}  {preview}", flush=True)
            ctrl.end_sample()
            print("  完成。下一姿态：u → 掰 → l → Enter。", flush=True)

        if len(samples_q) < 3:
            print(f"样本不足（{len(samples_q)} < 3）。", flush=True)
            return 2

        print(f"\n拟合 {len(samples_q)} 个姿态 …", flush=True)
        fits = fit_all_joints(samples_q, samples_tau)
        for cid, fit in zip(can_ids, fits):
            flag = "  << rmse 偏大" if fit.rmse > 0.1 else ""
            print(
                f"  0x{cid:02X}: amp={fit.amp:+.4f} phase={fit.phase:+.4f} "
                f"bias={fit.bias:+.4f}  rmse={fit.rmse:.4f}{flag}",
                flush=True,
            )

        yaml_text = format_gravity_yaml(
            fits, enabled=True, scale=1.0, use_measured_q=True, comments=names
        )
        print("\n--- 粘贴到 arm_5dof.yaml ---\n", flush=True)
        print(yaml_text, flush=True)
        out = Path("gravity_identified.yaml")
        out.write_text(yaml_text, encoding="utf-8")
        print(f"已写入 {out.resolve()}", flush=True)
        return 0

    except KeyboardInterrupt:
        print("\n中断。", flush=True)
        return 130
    finally:
        if ctrl is not None:
            ctrl.stop()
        try:
            arm.stop_mit_loop()
            arm.disable()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
