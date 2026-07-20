#!/usr/bin/env python3
"""联通测试：通过 dm_openarm API 检查 5 轴是否有反馈（不对机械臂做回零/大运动）。"""
from __future__ import annotations

import time

from dm_openarm import Arm


def main() -> int:
    arm = Arm.from_yaml("dm_openarm/config/arm_5dof.yaml")
    print("=== dm_openarm link_test ===", flush=True)
    try:
        arm.enable()
        hits = 0
        last = None
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 2.0:
            # 全 0 MIT：维持通信，不施加力矩（不启动回零 loop）
            arm._arm.send_zero_mit_all()
            states = arm.states()
            for s in states:
                if (
                    s.feedback_interval_s > 0
                    or abs(s.position) > 1e-9
                    or abs(s.velocity) > 1e-9
                    or abs(s.torque) > 1e-9
                ):
                    hits += 1
                    last = s
                    break
            time.sleep(0.01)

        if last is None:
            print("FAIL: no motor feedback", flush=True)
            return 3

        print("PASS: motor feedback received — link OK", flush=True)
        print(
            f"  sample can_id=0x{last.can_id:02X} pos={last.position:.4f} "
            f"vel={last.velocity:.4f} tau={last.torque:.4f} dt={last.feedback_interval_s:.4f}s "
            f"hits={hits}",
            flush=True,
        )
        return 0
    finally:
        try:
            arm.disable()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
