#!/usr/bin/env python3
"""交互式耦合重力辨识（按侧 left / right）。

流程：
  u      — UNLOCK 跟手（双手托住重量再掰）——仅操作所选侧
  l      — LOCK 钉住当前测量角（可松手）
  Enter  — 在【已有锁定目标】上采样；若当前是 UNLOCK 则先锁再采
  f      — 拟合
  q      — 放弃

示例：
  # 左臂（单臂配置默认）
  python python_script/identify_gravity.py --side left

  # 右臂（双臂配置）
  python python_script/identify_gravity.py \\
    --config dm_openarm/config/arm_dual_10dof.yaml --side right

输出：
  gravity_coupled_identified_left.yaml  /  _right.yaml
  （片段可直接粘到 arm_dual 的 left:/right: 或旧版根 gravity:）
"""
from __future__ import annotations

import threading
import time
from pathlib import Path

from dm_openarm import Arm, MitCommand
from dm_openarm.gravity_fit import (
    DEFAULT_COUPLED_BASIS,
    fit_coupled_all,
    format_coupled_yaml,
)
from dm_openarm.limb import Limb

DEFAULT_CONFIG = "dm_openarm/config/arm_5dof.yaml"
DUAL_CONFIG = "dm_openarm/config/arm_dual_10dof.yaml"
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

# 对侧仅软保持，避免误动
KP_OTHER, KD_OTHER = 8.0, 0.6

# 大电机（肘/肩）：左 0x04/0x05，右 0x24/0x25
_LARGE_CAN = frozenset({0x04, 0x05, 0x24, 0x25})


def is_large(cid: int) -> bool:
    return int(cid) in _LARGE_CAN


def g_unlock(cid: int) -> tuple[float, float]:
    return (KP_U_L, KD_U_L) if is_large(cid) else (KP_U_S, KD_U_S)


def g_lock(cid: int) -> tuple[float, float]:
    return (KP_L_L, KD_L_L) if is_large(cid) else (KP_L_S, KD_L_S)


def g_sample(cid: int) -> tuple[float, float]:
    return (KP_S_L, KD_S_L) if is_large(cid) else (KP_S_S, KD_S_S)


class HoldController:
    """Hold / teach assist scoped to one limb; soft-hold the rest of the bus."""

    def __init__(self, arm: Arm, limb: Limb):
        self.arm = arm
        self.limb = limb
        self._focus = set(limb.can_ids)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._mu = threading.Lock()
        self.locked = True
        self._q_des: dict[int, float] = {}
        self._sample_mode = False

    def start(self) -> None:
        states = self.limb.states()
        with self._mu:
            self._q_des = {int(s.can_id): float(s.position) for s in states}
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
            f"  → UNLOCK [{self.limb.side}]：双手托住再掰本侧。到位后按 l 锁定，再 Enter 采样。",
            flush=True,
        )

    def lock_here(self) -> None:
        states = self.limb.states()
        with self._mu:
            self._q_des = {int(s.can_id): float(s.position) for s in states}
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
        states = self.limb.states()
        with self._mu:
            if not self.locked or not self._q_des:
                self._q_des = {int(s.can_id): float(s.position) for s in states}
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
        self._push_now()

    def _push_now(self) -> None:
        try:
            all_states = self.arm.states()
            limb_states = [s for s in all_states if int(s.can_id) in self._focus]
            with self._mu:
                locked = self.locked
                sample = self._sample_mode
                q_des = dict(self._q_des)
                if not locked:
                    q_des = {int(s.can_id): float(s.position) for s in limb_states}
                    self._q_des = q_des
            cmds = {}
            for s in all_states:
                cid = int(s.can_id)
                if cid not in self._focus:
                    # Other limb: soft hold measured pose
                    cmds[cid] = MitCommand(
                        kp=KP_OTHER,
                        kd=KD_OTHER,
                        q=float(s.position),
                        dq=0.0,
                        tau=0.0,
                    )
                    continue
                if sample:
                    kp, kd = g_sample(cid)
                elif locked:
                    kp, kd = g_lock(cid)
                else:
                    kp, kd = g_unlock(cid)
                cmds[cid] = MitCommand(
                    kp=kp,
                    kd=kd,
                    q=q_des.get(cid, float(s.position)),
                    dq=0.0,
                    tau=0.0,
                )
            self.arm.mit(cmds)
        except Exception:
            pass

    def _loop(self) -> None:
        while not self._stop.is_set():
            self._push_now()
            time.sleep(0.02)


def average_limb_state(limb: Limb, duration_s: float) -> tuple[list[float], list[float]]:
    """Average q, tau for limb motors in limb.can_ids order."""
    n = 0
    q_sum: list[float] | None = None
    t_sum: list[float] | None = None
    dt = 1.0 / SAMPLE_HZ
    t0 = time.perf_counter()
    order = list(limb.can_ids)
    while time.perf_counter() - t0 < duration_s:
        by_id = {int(s.can_id): s for s in limb.states()}
        q = [float(by_id[c].position) for c in order]
        tau = [float(by_id[c].torque) for c in order]
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


def resolve_limb(arm: Arm, side: str) -> Limb:
    limb = arm.left if side == "left" else arm.right
    if limb is None or not limb.present:
        raise RuntimeError(
            f"{side} arm is not configured in this YAML "
            f"(left={arm.left is not None and arm.left.present}, "
            f"right={arm.right is not None and arm.right.present})"
        )
    return limb


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(
        description="Interactive coupled gravity identification (per side)"
    )
    parser.add_argument(
        "--side",
        choices=("left", "right"),
        default="left",
        help="which arm to identify (default left)",
    )
    parser.add_argument(
        "--config",
        default=None,
        help=f"YAML path (default: {DEFAULT_CONFIG}; "
        f"use {DUAL_CONFIG} for right arm)",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=0.9,
        help="scale written into output YAML (default 0.9)",
    )
    parser.add_argument(
        "--legacy-root-yaml",
        action="store_true",
        help="emit root gravity: (for arm_5dof) instead of left:/right: nested",
    )
    args = parser.parse_args()

    if args.config is None:
        args.config = DUAL_CONFIG if args.side == "right" else DEFAULT_CONFIG

    print("=== 重力辨识 identify_gravity（耦合 / 按侧）===", flush=True)
    print(f"config: {args.config}  side={args.side}", flush=True)
    print(
        f"""
操作（请严格按序）——仅移动 **{args.side}** 侧：

  1. 启动后默认 LOCK 本侧；对侧软保持不动。
  2. u  — 解锁跟手（双手托住重量再掰本侧）。
  3. l  — 锁定当前角（可松手检查）。
  4. Enter — 在锁定目标上加硬采样（不改目标角）。
  5. 重复 ≥20 点（耦合需要多轴组合：肩肘高低/伸屈一起变）。
  6. f 拟合 / q 放弃。

耦合提示：避免大量「只动一轴」；肩+肘组合姿态更重要。
采样时长 {SETTLE_S:.1f}s。
""",
        flush=True,
    )

    arm = Arm.from_yaml(args.config)
    ctrl: HoldController | None = None
    samples_q: list[list[float]] = []
    samples_tau: list[list[float]] = []
    can_ids: list[int] = []
    names: list[str] = []

    try:
        arm.enable()
        # Disable both sides' gravity feedforward during identification
        if arm.left and arm.left.present:
            arm.left.set_gravity_enabled(False)
        if arm.right and arm.right.present:
            arm.right.set_gravity_enabled(False)
        # Legacy single-arm path also clears left via root API
        arm.set_gravity_enabled(False)

        for _ in range(30):
            arm._arm.send_zero_mit_all()
            time.sleep(0.01)

        limb = resolve_limb(arm, args.side)
        can_ids = list(limb.can_ids)
        names = [f"0x{cid:02X}" for cid in can_ids]
        print(
            f"focus {args.side}: " + ", ".join(f"0x{c:02X}" for c in can_ids),
            flush=True,
        )

        # Seed all motors at current pose before loop
        for s in arm.states():
            arm.mit(
                int(s.can_id),
                kp=KP_OTHER,
                kd=KD_OTHER,
                q=float(s.position),
                dq=0.0,
                tau=0.0,
            )

        arm.start_mit_loop(hz=1000.0, home=False)
        ctrl = HoldController(arm, limb)
        ctrl.start()
        time.sleep(0.2)
        print(f"已 LOCK [{args.side}] 启动姿态。移动请先 u。", flush=True)

        while True:
            st = "LOCK" if (ctrl and ctrl.locked) else "UNLOCK"
            try:
                line = input(
                    f"[{args.side} | {len(samples_q)} samples | {st}] > "
                ).strip().lower()
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
            q_mean, tau_mean = average_limb_state(limb, SETTLE_S)
            samples_q.append(q_mean)
            samples_tau.append(tau_mean)
            preview = " ".join(
                f"0x{cid:02X}:q={qq:+.3f},τ={tt:+.3f}"
                for cid, qq, tt in zip(can_ids, q_mean, tau_mean)
            )
            print(f"  sample#{len(samples_q)}  {preview}", flush=True)
            ctrl.end_sample()
            print("  完成。下一姿态：u → 掰 → l → Enter。", flush=True)

        min_samples = 8
        if len(samples_q) < min_samples:
            print(
                f"样本不足（{len(samples_q)} < {min_samples}，耦合需多姿态）。",
                flush=True,
            )
            return 2

        print(
            f"\n拟合 {len(samples_q)} 个姿态（coupled, side={args.side}）…",
            flush=True,
        )

        result = fit_coupled_all(
            samples_q, samples_tau, basis=DEFAULT_COUPLED_BASIS, ridge=1e-6
        )
        for cid, joint in zip(can_ids, result.joints):
            flag = "  << rmse 偏大" if joint.rmse > 0.1 else ""
            print(
                f"  0x{cid:02X}: rmse={joint.rmse:.4f}  "
                f"|w|_max={max(abs(w) for w in joint.weights):.4f}{flag}",
                flush=True,
            )

        emit_side = None if args.legacy_root_yaml else args.side
        # Single-arm left + default config: allow root gravity for arm_5dof convenience
        if (
            args.side == "left"
            and not arm.is_dual()
            and not args.legacy_root_yaml
            and Path(args.config).name == "arm_5dof.yaml"
        ):
            emit_side = None

        yaml_text = format_coupled_yaml(
            result,
            enabled=True,
            scale=args.scale,
            use_measured_q=True,
            comments=names,
            side=emit_side,
        )
        out = Path(f"gravity_coupled_identified_{args.side}.yaml")

        if emit_side:
            paste_hint = (
                f"将下面片段合并进 arm_dual_10dof.yaml 的 **{args.side}:** 段 "
                f"（替换该侧 gravity:），并设 enabled: true"
            )
        else:
            paste_hint = "将下面片段合并进 arm_5dof.yaml 的 gravity: 段"

        print(f"\n--- {paste_hint} ---\n", flush=True)
        print(yaml_text, flush=True)
        out.write_text(yaml_text, encoding="utf-8")
        print(f"已写入 {out.resolve()}", flush=True)
        print(
            f"启用后: hold / move 用 arm.{args.side}.set_gravity_enabled(True) 验证。",
            flush=True,
        )
        return 0

    except KeyboardInterrupt:
        print("\n中断。", flush=True)
        return 130
    except Exception as exc:
        print(f"FAIL: {exc}", flush=True)
        return 1
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
