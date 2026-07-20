#!/usr/bin/env python3
"""持续读取各轴反馈状态，Ctrl+C 退出（不回零、不施加力矩）。"""
from __future__ import annotations

import time

from dm_openarm import Arm


def main() -> int:
    arm = Arm.from_yaml("dm_openarm/config/arm_5dof.yaml")
    try:
        arm.enable()
        print(
            f"{'can_id':>8} {'pos(rad)':>12} {'vel(rad/s)':>12} {'tau(N·m)':>12} {'dt(s)':>12}",
            flush=True,
        )
        print("持续读取中，Ctrl+C 退出…", flush=True)

        while True:
            arm._arm.send_zero_mit_all()
            states = arm.states()
            # 清屏式：每轮打印一块，行首用 \r 不方便多轴，改为时间戳 + 多行
            print(f"--- {time.strftime('%H:%M:%S')} ---", flush=True)
            for s in states:
                print(
                    f"0x{s.can_id:02X}"
                    f"{s.position:12.4f}"
                    f"{s.velocity:12.4f}"
                    f"{s.torque:12.4f}"
                    f"{s.feedback_interval_s:12.6f}",
                    flush=True,
                )
            time.sleep(0.05)  # ~20 Hz 打印
    except KeyboardInterrupt:
        print("\n已退出", flush=True)
        return 0
    finally:
        try:
            arm.disable()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
