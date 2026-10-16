#!/usr/bin/env python3
"""主动反馈诊断：使能电机并发送零 MIT，Ctrl+C 请求停止。"""
from __future__ import annotations

import time

from dm_openarm import Arm


def main() -> int:
    arm = Arm.from_yaml("dm_openarm/config/arm.yaml")
    result = 0
    try:
        arm.enable()
        print(
            f"{'can_id':>8} {'pos(rad)':>12} {'vel(rad/s)':>12} {'tau(N·m)':>12} "
            f"{'age(s)':>12} {'err':>5}",
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
                    f"{s.last_rx_age_s:12.6f}"
                    f"{s.error_code:5d}",
                    flush=True,
                )
            time.sleep(0.05)  # ~20 Hz 打印
    except KeyboardInterrupt:
        print("\n停止请求已收到", flush=True)
    except Exception as exc:
        print(f"读取失败：{exc}", flush=True)
        result = 1
    finally:
        try:
            arm.disable()
        except Exception as exc:
            print(f"关闭失败：{exc}", flush=True)
            result = 2
    return result


if __name__ == "__main__":
    raise SystemExit(main())
