#!/usr/bin/env python3
"""扫描 canid 0x01~0x05：检测是否在线；在线则下发 MIT 全 0 并回报状态。"""
from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from dmcan import dmcan_device_type

from damiao import (
    Control_Mode,
    DM_Motor_Type,
    DmActData,
    Motor_Control,
)

SN = "52A871B1AA5EF4E239371A5083463F26"
# 达妙常见约定：mstid = canid + 0x10
CAN_IDS = [0x01, 0x02, 0x03, 0x04, 0x05]
MST_OF = {cid: cid + 0x10 for cid in CAN_IDS}

NOM_BAUD = 1_000_000
DAT_BAUD = 1_000_000
# 每轮控制周期；总观察时间
PERIOD_S = 0.01
DURATION_S = 2.5
# 判定在线：观察窗口内至少收到这么多次有效状态更新
MIN_HITS = 5


@dataclass
class MotorStat:
    can_id: int
    mst_id: int
    hits: int = 0
    pos: float = 0.0
    vel: float = 0.0
    tau: float = 0.0
    err: int = 0
    dt: float = 0.0
    send_ok: int = 0
    send_fail: int = 0

    @property
    def online(self) -> bool:
        return self.hits >= MIN_HITS


def _has_feedback(pos: float, vel: float, tau: float, err: int, dt: float) -> bool:
    return dt > 0 or abs(pos) > 1e-9 or abs(vel) > 1e-9 or abs(tau) > 1e-9 or err != 0


def main() -> int:
    init: List[DmActData] = [
        DmActData(
            motorType=DM_Motor_Type.DM4310,
            mode=Control_Mode.MIT_MODE,
            can_id=cid,
            mst_id=MST_OF[cid],
        )
        for cid in CAN_IDS
    ]

    print("=== multi-motor scan 0x01~0x05 ===", flush=True)
    print(f"SN={SN}", flush=True)
    print("mode=classic CAN 1M  canfd=False brs=False", flush=True)
    print(
        "cmd=MIT kp=kd=q=dq=tau=0 (status only, no torque)",
        flush=True,
    )
    print(
        "map: "
        + ", ".join(f"0x{c:02X}->mst 0x{MST_OF[c]:02X}" for c in CAN_IDS),
        flush=True,
    )
    print(f"duration={DURATION_S}s  min_hits={MIN_HITS}", flush=True)
    print(flush=True)

    stats: Dict[int, MotorStat] = {
        cid: MotorStat(can_id=cid, mst_id=MST_OF[cid]) for cid in CAN_IDS
    }

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
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < DURATION_S:
            for cid in CAN_IDS:
                motor = control.getMotor(cid)
                st = stats[cid]
                if motor is None:
                    st.send_fail += 1
                    continue
                ok = control.control_mit(motor, 0.0, 0.0, 0.0, 0.0, 0.0)
                if ok:
                    st.send_ok += 1
                else:
                    st.send_fail += 1
                pos = motor.Get_Position()
                vel = motor.Get_Velocity()
                tau = motor.Get_tau()
                err = motor.Get_err()
                dt = motor.getTimeInterval()
                if _has_feedback(pos, vel, tau, err, dt):
                    st.hits += 1
                    st.pos, st.vel, st.tau, st.err, st.dt = pos, vel, tau, err, dt
            time.sleep(PERIOD_S)
    except Exception as exc:
        print(f"FAIL during scan: {exc}", flush=True)
        try:
            control.disable_all()
        except Exception:
            pass
        os._exit(1)

    # 结果表
    print(f"{'canid':<8}{'mstid':<8}{'online':<8}{'hits':>6}  "
          f"{'pos':>10}{'vel':>10}{'tau':>10}{'err':>6}  send_ok/fail",
          flush=True)
    print("-" * 88, flush=True)

    online_ids: List[int] = []
    for cid in CAN_IDS:
        st = stats[cid]
        flag = "YES" if st.online else "NO"
        if st.online:
            online_ids.append(cid)
        print(
            f"0x{cid:02X}    0x{st.mst_id:02X}    {flag:<8}{st.hits:6d}  "
            f"{st.pos:10.4f}{st.vel:10.4f}{st.tau:10.4f}{st.err:6d}  "
            f"{st.send_ok}/{st.send_fail}",
            flush=True,
        )

    print(flush=True)
    print(
        f"summary: online {len(online_ids)}/{len(CAN_IDS)}  "
        f"ids={[f'0x{i:02X}' for i in online_ids]}",
        flush=True,
    )

    if online_ids:
        print(flush=True)
        print("--- status of online motors (after MIT zero cmd) ---", flush=True)
        for cid in online_ids:
            st = stats[cid]
            print(
                f"  canid=0x{cid:02X} mstid=0x{st.mst_id:02X} "
                f"pos={st.pos:.4f} vel={st.vel:.4f} tau={st.tau:.4f} "
                f"err={st.err} dt={st.dt:.4f}s hits={st.hits}",
                flush=True,
            )
        code = 0
    else:
        print("FAIL: no motor online in 0x01~0x05", flush=True)
        code = 3

    try:
        control.disable_all()
        time.sleep(0.05)
    except Exception:
        pass
    # 避免本机 libusb close 断言盖住结果
    os._exit(code)


if __name__ == "__main__":
    main()
