#!/usr/bin/env python3
"""交互式重力参数辨识。

流程：
  1. 使能并以较高 kp 保持当前姿态（重力补偿关闭）
  2. 手动缓慢摆到若干静态姿态，在每个姿态按 Enter 采样
  3. 对每轴拟合  tau = amp * sin(q + phase) + bias
  4. 打印可粘贴进 arm_5dof.yaml 的 gravity 段

注意：
  - 采样时请尽量静止 0.5s+；姿态尽量覆盖各轴较大角度范围
  - 至少 5 个姿态，推荐 8–15 个
  - 输入 q 退出并不保存；拟合完成后可选写文件

用法（仓库根目录）::

    source .venv/bin/activate
    pip install -e ./dm_openarm
    python python_script/identify_gravity.py
"""
from __future__ import annotations

import time
from pathlib import Path

from dm_openarm import Arm, MitCommand
from dm_openarm.gravity_fit import fit_all_joints, format_gravity_yaml

CONFIG = "dm_openarm/config/arm_5dof.yaml"
SETTLE_S = 0.6
SAMPLE_HZ = 50.0
# 辨识用较高刚度，减小静差，让反馈力矩更接近重力负载
KP_ID_SMALL = 25.0
KD_ID_SMALL = 1.0
KP_ID_LARGE = 18.0
KD_ID_LARGE = 1.2


def gains(can_id: int) -> tuple[float, float]:
    if can_id >= 0x04:
        return KP_ID_LARGE, KD_ID_LARGE
    return KP_ID_SMALL, KD_ID_SMALL


def average_state(arm: Arm, duration_s: float) -> tuple[list[float], list[float]]:
    """Return mean q and mean measured torque over duration."""
    n = 0
    q_sum: list[float] | None = None
    t_sum: list[float] | None = None
    dt = 1.0 / SAMPLE_HZ
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < duration_s:
        states = arm.states()
        q = [s.position for s in states]
        tau = [s.torque for s in states]
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


def hold_current(arm: Arm) -> None:
    states = arm.states()
    cmds = {}
    for s in states:
        kp, kd = gains(s.can_id)
        cmds[s.can_id] = MitCommand(kp=kp, kd=kd, q=s.position, dq=0.0, tau=0.0)
    arm.mit(cmds)


def main() -> int:
    print("=== 重力辨识 identify_gravity ===", flush=True)
    print(f"config: {CONFIG}", flush=True)
    print(
        "操作：\n"
        "  Enter     — 在当前静态姿态采样（会 settle {:.1f}s）\n"
        "  f + Enter — 结束采样并拟合\n"
        "  q + Enter — 放弃退出\n".format(SETTLE_S),
        flush=True,
    )

    arm = Arm.from_yaml(CONFIG)
    samples_q: list[list[float]] = []
    samples_tau: list[list[float]] = []
    can_ids: list[int] = []
    names: list[str] = []

    try:
        arm.enable()
        arm.set_gravity_enabled(False)  # 辨识时不要叠旧重力模型
        for _ in range(30):
            arm._arm.send_zero_mit_all()
            time.sleep(0.01)

        states = arm.states()
        can_ids = [s.can_id for s in states]
        names = [f"0x{cid:02X}" for cid in can_ids]

        arm.start_mit_loop(hz=1000.0, home=False)
        hold_current(arm)

        print("已使能并保持当前姿态。请手动摆到静态点后按 Enter 采样。", flush=True)

        while True:
            try:
                line = input(f"[{len(samples_q)} samples] > ").strip().lower()
            except EOFError:
                line = "q"

            if line in ("q", "quit", "exit"):
                print("已放弃。", flush=True)
                return 1
            if line in ("f", "fit", "done"):
                break
            if line not in ("", "s", "sample"):
                print("未知命令。Enter=采样, f=拟合, q=退出", flush=True)
                continue

            # Re-lock target to current pose before settle (user may have moved)
            hold_current(arm)
            print(f"  settling {SETTLE_S}s ...", flush=True)
            time.sleep(0.15)
            q_mean, tau_mean = average_state(arm, SETTLE_S)
            samples_q.append(q_mean)
            samples_tau.append(tau_mean)
            preview = " ".join(
                f"0x{cid:02X}:q={qq:+.3f},τ={tt:+.3f}"
                for cid, qq, tt in zip(can_ids, q_mean, tau_mean)
            )
            print(f"  sample#{len(samples_q)}  {preview}", flush=True)

        if len(samples_q) < 3:
            print(f"样本不足（{len(samples_q)} < 3），无法拟合。", flush=True)
            return 2

        print(f"\n拟合 {len(samples_q)} 个姿态 ...", flush=True)
        fits = fit_all_joints(samples_q, samples_tau)
        for cid, fit in zip(can_ids, fits):
            print(
                f"  0x{cid:02X}: amp={fit.amp:+.4f} phase={fit.phase:+.4f} "
                f"bias={fit.bias:+.4f}  rmse={fit.rmse:.4f} (n={fit.n})",
                flush=True,
            )

        yaml_text = format_gravity_yaml(
            fits,
            enabled=True,
            scale=1.0,
            use_measured_q=True,
            comments=names,
        )
        print("\n--- 粘贴到 arm_5dof.yaml ---\n", flush=True)
        print(yaml_text, flush=True)

        out = Path("gravity_identified.yaml")
        out.write_text(yaml_text, encoding="utf-8")
        print(f"已写入 {out.resolve()}", flush=True)
        print(
            "请检查 rmse 与 amp 是否合理，确认后把 gravity: 段合并进 "
            "dm_openarm/config/arm_5dof.yaml，再运行 hold_with_gravity.py。",
            flush=True,
        )
        return 0

    except KeyboardInterrupt:
        print("\n中断。", flush=True)
        return 130
    finally:
        try:
            arm.stop_mit_loop()
            arm.disable()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
