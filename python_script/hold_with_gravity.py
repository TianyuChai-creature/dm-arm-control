#!/usr/bin/env python3
"""重力补偿保持（可托起后停稳再松手）。

【为何以前一松手就掉】
  旧脚本只在启动瞬间把 q_des 设成当时姿态。
  你把臂拉高后，目标仍在下面 → 松手被弹簧拉回起点（像掉下去）。

【现在】
  • 始终叠加重力前馈 tau_g = amp*sin(q+phase)+bias
  • 你托着移动时：跟手（目标跟着走）+ 重力前馈托重量
  • 停稳约 0.4s：自动锁定当前角 + 中等 kp，此时再松手应能站住
  • 若仍慢沉：会打印提示；可改 YAML gravity.scale 或下面 KP_HOLD

Ctrl+C 退出。
"""
from __future__ import annotations

import threading
import time

from dm_openarm import Arm, MitCommand

CONFIG = "dm_openarm/config/arm_5dof.yaml"

# 跟手（有速度时）
KP_D_S, KD_D_S = 1.0, 0.45
KP_D_L, KD_D_L = 0.8, 0.5

# 锁定保持：略硬 + 更大 kd，抑制过补偿引起的过冲
KP_H_S, KD_H_S = 12.0, 1.2
KP_H_L, KD_H_L = 10.0, 1.4

VEL_MOVE = 0.12  # rad/s
STILL_S = 0.4

# 默认欠补偿（与 YAML scale 一致时可再乘）
DEFAULT_SCALE = 0.9


def g_drag(cid: int) -> tuple[float, float]:
    return (KP_D_L, KD_D_L) if cid >= 0x04 else (KP_D_S, KD_D_S)


def g_hold(cid: int) -> tuple[float, float]:
    return (KP_H_L, KD_H_L) if cid >= 0x04 else (KP_H_S, KD_H_S)


class GravityHoldAssist:
    """
    locked: 钉死 q_des（不因下沉改目标）
    运动中: 跟手 + 仍由 MIT loop 叠加重力
    停稳: 自动锁定
    锁定后慢沉: 不解锁（避免跟丢塌掉）
    """

    def __init__(self, arm: Arm):
        self.arm = arm
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._mu = threading.Lock()
        self.locked = True
        self._q_des: dict[int, float] = {}
        self._still_t0: float | None = None
        self.mode = "hold"

    def start(self) -> None:
        states = self.arm.states()
        with self._mu:
            self._q_des = {s.can_id: s.position for s in states}
            self.locked = True
            self._still_t0 = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="ghold", daemon=True)
        self._thread.start()
        self._push()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _max_vel(self, states) -> float:
        return max((abs(s.velocity) for s in states), default=0.0)

    def _push(self) -> None:
        try:
            states = self.arm.states()
            with self._mu:
                locked = self.locked
                q_des = dict(self._q_des)

            cmds = {}
            for s in states:
                cid = s.can_id
                if locked:
                    kp, kd = g_hold(cid)
                    q = q_des.get(cid, s.position)
                else:
                    kp, kd = g_drag(cid)
                    q = s.position
                cmds[cid] = MitCommand(kp=kp, kd=kd, q=q, dq=0.0, tau=0.0)
            self.arm.mit(cmds)
        except Exception:
            pass

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                states = self.arm.states()
                vmax = self._max_vel(states)
                now = time.perf_counter()

                with self._mu:
                    if vmax > VEL_MOVE:
                        # 明确在被托着动 → 跟手；下沉很慢通常不进这里
                        self.locked = False
                        self._still_t0 = None
                        self.mode = "drag"
                        self._q_des = {s.can_id: s.position for s in states}
                        # 运动中用测量角算 g(q)，托得更顺
                        try:
                            self.arm.set_gravity_use_measured_q(True)
                        except Exception:
                            pass
                    else:
                        if not self.locked:
                            if self._still_t0 is None:
                                self._still_t0 = now
                            elif (now - self._still_t0) >= STILL_S:
                                self._q_des = {s.can_id: s.position for s in states}
                                self.locked = True
                                self.mode = "hold"
                                # 锁定后用目标角算 g(q_des)：前馈恒定，避免随下沉正反馈过冲
                                try:
                                    self.arm.set_gravity_use_measured_q(False)
                                except Exception:
                                    pass
                                print(
                                    "  [锁定] 已停稳，可松手。目标: "
                                    + " ".join(
                                        f"0x{c:02X}={q:+.2f}"
                                        for c, q in sorted(self._q_des.items())
                                    ),
                                    flush=True,
                                )
                        # 已 locked：即使慢沉也绝不改 q_des、不解锁
            except Exception:
                pass
            self._push()
            time.sleep(0.02)


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Gravity-compensated soft hold")
    parser.add_argument(
        "--scale",
        type=float,
        default=DEFAULT_SCALE,
        help="gravity scale (default 0.9 under-compensate; try 0.8~1.0)",
    )
    args = parser.parse_args()

    print("=== hold_with_gravity ===", flush=True)
    arm = Arm.from_yaml(CONFIG)
    assist: GravityHoldAssist | None = None
    try:
        arm.enable()
        for _ in range(30):
            arm._arm.send_zero_mit_all()
            time.sleep(0.01)

        states = arm.states()
        qs = [s.position for s in states]
        arm.set_gravity_scale(1.0)
        print(f"重力前馈预览（模型×{args.scale}）:", flush=True)
        for s, tg in zip(states, arm.gravity_torques(qs)):
            print(
                f"  0x{s.can_id:02X}  q={s.position:+.4f}  tau_g={tg * args.scale:+.4f} N·m",
                flush=True,
            )

        # 默认欠补偿：过补偿在软刚度下会顶过头
        arm.set_gravity_enabled(True)
        arm.set_gravity_scale(args.scale)
        arm.set_gravity_use_measured_q(False)  # 锁定时用 q_des，避免正反馈
        print(
            f"gravity_enabled={arm.gravity_enabled()} scale={arm.gravity_scale()} "
            f"(欠补偿更稳；仍下沉 --scale 1.0；过冲 --scale 0.8)",
            flush=True,
        )

        arm.start_mit_loop(hz=1000.0, home=False)
        assist = GravityHoldAssist(arm)
        assist.start()

        print(
            f"""
使用方法：
  1. 启动后锁定当前姿态（重力前馈 scale={args.scale} + 中等 kp + 较大 kd）。
  2. 托着移到新姿态 → 停 0.4s 见「[锁定]」→ 再松手。
  3. 个别点过冲：--scale 0.8；仍下沉：--scale 1.0 或 1.05。
  4. Ctrl+C 退出。
""",
            flush=True,
        )

        while True:
            time.sleep(1.0)
            states = arm.states()
            mode = assist.mode if assist else "?"
            line = f"[{mode}] " + " | ".join(
                f"0x{s.can_id:02X} q={s.position:+.3f}" for s in states
            )
            print(line, flush=True)

    except KeyboardInterrupt:
        print("\n已退出", flush=True)
        return 0
    finally:
        if assist is not None:
            assist.stop()
        try:
            arm.set_gravity_enabled(False)
            arm.stop_mit_loop()
            arm.disable()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
