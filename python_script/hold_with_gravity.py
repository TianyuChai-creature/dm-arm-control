#!/usr/bin/env python3
"""重力补偿 + 软刚度保持当前姿态。

默认从 YAML 读取 gravity 参数（amp/phase/bias，初值为 0）。
系数全 0 时等价于纯软刚度保持，便于先联调再辨识。

用法（仓库根目录）::

    source .venv/bin/activate
    python python_script/hold_with_gravity.py
    # Ctrl+C 退出
"""
from __future__ import annotations

import time

from dm_openarm import Arm, MitCommand


# 软保持增益（前馈准了以后可以再降）
KP_SMALL = 8.0
KD_SMALL = 0.4
KP_LARGE = 6.0
KD_LARGE = 0.5


def gains_for_can_id(can_id: int) -> tuple[float, float]:
    # 0x04/0x05 为大电机 DM8009
    if can_id >= 0x04:
        return KP_LARGE, KD_LARGE
    return KP_SMALL, KD_SMALL


def main() -> int:
    arm = Arm.from_yaml("dm_openarm/config/arm_5dof.yaml")
    try:
        arm.enable()
        # 短时 MIT 全 0，刷新反馈
        for _ in range(20):
            arm._arm.send_zero_mit_all()
            time.sleep(0.01)

        states = arm.states()
        print("当前姿态 / 重力前馈预览（scale 已含）:")
        tau_g = arm.gravity_torques([s.position for s in states])
        for s, tg in zip(states, tau_g):
            print(
                f"  0x{s.can_id:02X}  q={s.position:+.4f} rad  "
                f"tau_g={tg:+.4f} N·m",
                flush=True,
            )

        # 以当前位置为保持目标；tau 命令保持 0，重力由 loop 叠加
        cmds = {}
        for s in states:
            kp, kd = gains_for_can_id(s.can_id)
            cmds[s.can_id] = MitCommand(
                kp=kp, kd=kd, q=s.position, dq=0.0, tau=0.0
            )

        arm.set_gravity_enabled(True)
        arm.start_mit_loop(hz=1000.0, home=False)
        arm.mit(cmds)

        print(
            "重力补偿保持中（软 kp）。Ctrl+C 退出。\n"
            "提示：若 amp/bias 仍为 0，仅软刚度保持，请在 YAML gravity.joints 填入辨识系数。",
            flush=True,
        )
        while True:
            time.sleep(0.5)
            states = arm.states()
            line = " | ".join(
                f"0x{s.can_id:02X} q={s.position:+.3f}" for s in states
            )
            print(line, flush=True)

    except KeyboardInterrupt:
        print("\n已退出", flush=True)
        return 0
    finally:
        try:
            arm.set_gravity_enabled(False)
            arm.stop_mit_loop()
            arm.disable()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
