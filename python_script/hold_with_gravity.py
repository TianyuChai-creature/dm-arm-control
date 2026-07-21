#!/usr/bin/env python3
"""双臂站位：按侧重力软保持（托起后停稳再松手）。

  • 只操作 --side（left|right）；对侧软钉当前角
  • 运动中：本侧跟手 + 该侧重力前馈
  • 停稳约 0.4s：锁定本侧 q_des
  • Ctrl+C 退出

示例：
  python python_script/hold_with_gravity.py --side left
  python python_script/hold_with_gravity.py --side right --scale 0.9
"""
from __future__ import annotations

import argparse
import threading
import time

from dm_openarm import Arm, MitCommand
from dm_openarm.limb import Limb

CONFIG = "dm_openarm/config/arm.yaml"

KP_D_S, KD_D_S = 1.0, 0.45
KP_D_L, KD_D_L = 0.8, 0.5
KP_H_S, KD_H_S = 12.0, 1.2
KP_H_L, KD_H_L = 10.0, 1.4
KP_OTHER, KD_OTHER = 8.0, 0.6

VEL_MOVE = 0.12
STILL_S = 0.4
DEFAULT_SCALE = 0.9
_LARGE = frozenset({0x04, 0x05, 0x24, 0x25})


def is_large(cid: int) -> bool:
    return int(cid) in _LARGE


def g_drag(cid: int) -> tuple[float, float]:
    return (KP_D_L, KD_D_L) if is_large(cid) else (KP_D_S, KD_D_S)


def g_hold(cid: int) -> tuple[float, float]:
    return (KP_H_L, KD_H_L) if is_large(cid) else (KP_H_S, KD_H_S)


class GravityHoldAssist:
    def __init__(self, arm: Arm, limb: Limb):
        self.arm = arm
        self.limb = limb
        self._focus = set(limb.can_ids)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._mu = threading.Lock()
        self.locked = True
        self._q_des: dict[int, float] = {}
        self._still_t0: float | None = None
        self.mode = "hold"

    def start(self) -> None:
        states = self.limb.states()
        with self._mu:
            self._q_des = {int(s.can_id): float(s.position) for s in states}
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

    def _max_vel_focus(self, states) -> float:
        return max(
            (abs(s.velocity) for s in states if int(s.can_id) in self._focus),
            default=0.0,
        )

    def _push(self) -> None:
        try:
            all_states = self.arm.states()
            with self._mu:
                locked = self.locked
                q_des = dict(self._q_des)

            cmds = {}
            for s in all_states:
                cid = int(s.can_id)
                if cid not in self._focus:
                    cmds[cid] = MitCommand(
                        kp=KP_OTHER,
                        kd=KD_OTHER,
                        q=float(s.position),
                        dq=0.0,
                        tau=0.0,
                    )
                    continue
                if locked:
                    kp, kd = g_hold(cid)
                    q = q_des.get(cid, float(s.position))
                else:
                    kp, kd = g_drag(cid)
                    q = float(s.position)
                cmds[cid] = MitCommand(kp=kp, kd=kd, q=q, dq=0.0, tau=0.0)
            self.arm.mit(cmds)
        except Exception:
            pass

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                all_states = self.arm.states()
                focus = [s for s in all_states if int(s.can_id) in self._focus]
                vmax = self._max_vel_focus(focus)
                now = time.perf_counter()

                with self._mu:
                    if vmax > VEL_MOVE:
                        self.locked = False
                        self._still_t0 = None
                        self.mode = "drag"
                        self._q_des = {
                            int(s.can_id): float(s.position) for s in focus
                        }
                        try:
                            self.limb.set_gravity_use_measured_q(True)
                        except Exception:
                            pass
                    else:
                        if not self.locked:
                            if self._still_t0 is None:
                                self._still_t0 = now
                            elif (now - self._still_t0) >= STILL_S:
                                self._q_des = {
                                    int(s.can_id): float(s.position) for s in focus
                                }
                                self.locked = True
                                self.mode = "hold"
                                try:
                                    self.limb.set_gravity_use_measured_q(False)
                                except Exception:
                                    pass
                                print(
                                    f"  [锁定 {self.limb.side}] 已停稳，可松手。目标: "
                                    + " ".join(
                                        f"0x{c:02X}={q:+.2f}"
                                        for c, q in sorted(self._q_des.items())
                                    ),
                                    flush=True,
                                )
            except Exception:
                pass
            self._push()
            time.sleep(0.02)


def main() -> int:
    parser = argparse.ArgumentParser(description="Per-side gravity soft hold (dual-arm)")
    parser.add_argument("--config", default=CONFIG)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument(
        "--scale",
        type=float,
        default=DEFAULT_SCALE,
        help="gravity scale for this side (default 0.9)",
    )
    args = parser.parse_args()

    print(f"=== hold_with_gravity side={args.side} ===", flush=True)
    arm = Arm.from_yaml(args.config)
    assist: GravityHoldAssist | None = None
    limb: Limb | None = None
    try:
        arm.enable()
        for _ in range(30):
            arm._arm.send_zero_mit_all()
            time.sleep(0.01)

        limb = arm.left if args.side == "left" else arm.right
        if limb is None or not limb.present:
            print(f"side {args.side} not configured")
            return 1

        # Seed all motors
        for s in arm.states():
            arm.mit(
                int(s.can_id),
                kp=KP_OTHER,
                kd=KD_OTHER,
                q=float(s.position),
                dq=0.0,
                tau=0.0,
            )

        qs = [float(s.position) for s in limb.states()]
        print(f"重力前馈预览 [{args.side}]（模型×{args.scale}）:", flush=True)
        for s, tg in zip(limb.states(), limb.gravity_torques(qs)):
            print(
                f"  0x{s.can_id:02X}  q={s.position:+.4f}  tau_g={tg * args.scale:+.4f} N·m",
                flush=True,
            )

        # Other side gravity off; focus side on
        if arm.left and arm.left.present and args.side != "left":
            arm.left.set_gravity_enabled(False)
        if arm.right and arm.right.present and args.side != "right":
            arm.right.set_gravity_enabled(False)
        limb.set_gravity_enabled(True)
        limb.set_gravity_scale(args.scale)
        limb.set_gravity_use_measured_q(False)
        print(
            f"gravity[{args.side}] enabled={limb.gravity_enabled()} "
            f"scale={limb.gravity_scale()}",
            flush=True,
        )

        arm.start_mit_loop(hz=1000.0, home=False)
        assist = GravityHoldAssist(arm, limb)
        assist.start()

        print(
            f"""
使用方法（仅 {args.side}）：
  1. 启动后锁定本侧姿态。
  2. 托着移到新姿态 → 停 0.4s 见「[锁定]」→ 再松手。
  3. 过冲 --scale 0.8；下沉 --scale 1.0。
  4. Ctrl+C 退出。对侧保持软钉不动。
""",
            flush=True,
        )

        while True:
            time.sleep(1.0)
            states = limb.states()
            mode = assist.mode if assist else "?"
            line = f"[{args.side}|{mode}] " + " | ".join(
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
            if limb is not None:
                limb.set_gravity_enabled(False)
            arm.stop_mit_loop()
            arm.disable()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
