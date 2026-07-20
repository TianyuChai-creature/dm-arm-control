#!/usr/bin/env python3
"""只测 USB2CANFD 与电机是否联通：不持续推力矩、不做回零。"""
from __future__ import annotations

import os
import sys
import time

from dmcan import dmcan_device_type

from damiao import (
    Control_Mode,
    DM_Motor_Type,
    DmActData,
    Motor_Control,
)

# 与 dev_sn.py / damiao.py 当前设备一致
SN = "52A871B1AA5EF4E239371A5083463F26"
CAN_ID = 0x01
MST_ID = 0x11
NOM_BAUD = 1_000_000
DAT_BAUD = 1_000_000  # classic CAN 下数据域不用
DURATION_S = 2.0


def main() -> int:
    init = [
        DmActData(
            motorType=DM_Motor_Type.DM4310,
            mode=Control_Mode.MIT_MODE,
            can_id=CAN_ID,
            mst_id=MST_ID,
        )
    ]

    print("=== u2canfd link test ===", flush=True)
    print(f"SN={SN}", flush=True)
    print(f"canid=0x{CAN_ID:02X} mstid=0x{MST_ID:02X}", flush=True)
    print("mode=classic CAN 1M (canfd=False, brs=False)", flush=True)
    print(f"duration={DURATION_S}s  command=MIT all-zero (no torque)", flush=True)
    print(flush=True)

    rx_hits = 0
    last = None
    code = 1

    try:
        control = Motor_Control(
            NOM_BAUD,
            DAT_BAUD,
            SN,
            init,
            device_type=dmcan_device_type.USB2CANFD,
            canfd=False,
            brs=False,
            auto_enable=True,
        )
    except Exception as exc:
        print(f"FAIL: open/init: {exc}", flush=True)
        return 1

    try:
        motor = control.getMotor(CAN_ID)
        if motor is None:
            print("FAIL: motor not registered", flush=True)
            code = 2
        else:
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < DURATION_S:
                # 全 0：维持通信，不施加力矩
                control.control_mit(motor, 0.0, 0.0, 0.0, 0.0, 0.0)
                pos = motor.Get_Position()
                vel = motor.Get_Velocity()
                tau = motor.Get_tau()
                err = motor.Get_err()
                dt = motor.getTimeInterval()
                if dt > 0 or abs(pos) > 1e-9 or abs(vel) > 1e-9 or abs(tau) > 1e-9 or err != 0:
                    rx_hits += 1
                    last = (pos, vel, tau, err, dt)
                time.sleep(0.01)

            if last is None:
                print(
                    "FAIL: no motor feedback "
                    "(check power, wiring, baud, canid/mstid)",
                    flush=True,
                )
                code = 3
            else:
                pos, vel, tau, err, dt = last
                print("PASS: motor feedback received — link OK", flush=True)
                print(
                    f"  last: pos={pos:.4f} vel={vel:.4f} tau={tau:.4f} "
                    f"err={err} dt={dt:.4f}s  hits={rx_hits}",
                    flush=True,
                )
                print(
                    "  note: MIT tau=0; this test does not command motion/homing",
                    flush=True,
                )
                code = 0
    except Exception as exc:
        print(f"FAIL: {exc}", flush=True)
        code = 1
    finally:
        # 尽量发失能帧；完整 close() 在本机 libusb 上会断言崩溃，跳过以免盖住测试结果
        try:
            control.disable_all()
            time.sleep(0.05)
        except Exception:
            pass
        os._exit(code)


if __name__ == "__main__":
    main()
