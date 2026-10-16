#!/usr/bin/env python3
"""主动反馈诊断：使能电机并发送零 MIT；仅查在线请用 scan_motors_online.py。"""
from __future__ import annotations

import time

from dm_openarm import Arm


def main() -> int:
    arm = Arm.from_yaml("dm_openarm/config/arm.yaml")
    print("=== dm_openarm link_test ===", flush=True)
    result = 0
    try:
        arm.enable()
        hits = 0
        last = None
        last_sequence = 0
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 2.0:
            # 全 0 MIT：维持通信，不施加力矩（不启动回零 loop）
            arm._arm.send_zero_mit_all()
            states = arm.states()
            for s in states:
                if s.feedback_fresh(0.1) and int(s.rx_sequence) != last_sequence:
                    hits += 1
                    last = s
                    last_sequence = int(s.rx_sequence)
                    break
            time.sleep(0.01)

        if last is None:
            print("FAIL: no motor feedback", flush=True)
            result = 3
        elif int(last.error_code) != 0:
            print(f"FAIL: motor error_code={last.error_code}", flush=True)
            result = 4
        else:
            print(
                f"  sample can_id=0x{last.can_id:02X} pos={last.position:.4f} "
                f"vel={last.velocity:.4f} tau={last.torque:.4f} "
                f"age={last.last_rx_age_s:.4f}s err={last.error_code} "
                f"hits={hits}",
                flush=True,
            )
    except KeyboardInterrupt:
        print("FAIL: diagnostic interrupted", flush=True)
        result = 130
    except Exception as exc:
        print(f"FAIL: diagnostic failed: {exc}", flush=True)
        result = 1
    finally:
        try:
            arm.disable()
        except Exception as exc:
            print(f"FAIL: shutdown failed: {exc}", flush=True)
            result = 2
    if result == 0:
        print("PASS: motor feedback received; connection closed", flush=True)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
