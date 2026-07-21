#!/usr/bin/env python3
"""Visible multi-joint trajectory demo with coupled gravity.

Default motion (meant to be obvious by eye):
  - Elbow (0x04): +0.5 rad  (~29°)
  - Shoulder (0x05): -0.5 rad (~29°)
  - Other joints fixed
  - Duration ~2.5 s per leg (not a 4 s crawl)
  - Round-trip: start → pose A → hold → back to start → hold

Safety:
  - start_mit_loop(home=False); no slam to zero
  - Confirm before motion (unless --yes)
  - Soft MIT lag is normal; MoveResult reports max_abs_err

If still hard to see: raise --delta / lower --duration / raise --kp.
"""
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

from dm_openarm import Arm

CONFIG = "dm_openarm/config/arm.yaml"

# Slightly firmer than soft-hold defaults so lag does not hide the path.
KP_WRIST = 14.0
KD_WRIST = 0.7
KP_BIG = 14.0
KD_BIG = 1.0


def _fmt_q(q: list[float]) -> str:
    deg = [f"{math.degrees(x):+.1f}°" for x in q]
    rad = [f"{x:+.3f}" for x in q]
    return f"rad={rad}  deg={deg}"


def _gains(can_ids: list[int], kp_scale: float) -> tuple[list[float], list[float]]:
    kp: list[float] = []
    kd: list[float] = []
    for cid in can_ids:
        if cid in (0x04, 0x05):
            kp.append(KP_BIG * kp_scale)
            kd.append(KD_BIG)
        else:
            kp.append(KP_WRIST * kp_scale)
            kd.append(KD_WRIST)
    return kp, kd


def _apply_deltas(q0: list[float], can_ids: list[int], delta: float) -> list[float]:
    """Elbow +, shoulder − (left 0x04/0x05 or right 0x24/0x25)."""
    q = list(q0)
    for i, cid in enumerate(can_ids):
        if cid in (0x04, 0x24):
            q[i] = q0[i] + delta
        elif cid in (0x05, 0x25):
            q[i] = q0[i] - delta
    return q


def main() -> int:
    parser = argparse.ArgumentParser(description="Visible joint trajectory demo with gravity")
    parser.add_argument("--config", default=CONFIG)
    parser.add_argument(
        "--side",
        choices=("left", "right"),
        default="left",
        help="which limb (dual-arm configs; default left)",
    )
    parser.add_argument("--scale", type=float, default=None, help="gravity scale override")
    parser.add_argument(
        "--duration",
        type=float,
        default=2.5,
        help="duration per leg [s] (default 2.5; shorter = snappier)",
    )
    parser.add_argument(
        "--delta",
        type=float,
        default=0.5,
        help="elbow/shoulder step [rad] (default 0.5 ≈ 29°)",
    )
    parser.add_argument("--rate", type=float, default=100.0, help="command stream rate [Hz]")
    parser.add_argument(
        "--pause",
        type=float,
        default=1.0,
        help="hold at each endpoint before next leg [s]",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=3.0,
        help="final hold after return [s]",
    )
    parser.add_argument(
        "--one-way",
        action="store_true",
        help="only go to goal (no return trip)",
    )
    parser.add_argument(
        "--kp-scale",
        type=float,
        default=1.0,
        help="multiply tracking kp (try 1.5 if lag hides motion)",
    )
    parser.add_argument("--no-gravity", action="store_true", help="disable gravity FF")
    parser.add_argument("--yes", action="store_true", help="skip confirmation prompt")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print planned targets only (no hardware)",
    )
    args = parser.parse_args()

    deg = math.degrees(args.delta)
    legs = 1 if args.one_way else 2
    if args.dry_run:
        print(
            f"dry-run: delta={args.delta:.3f} rad ({deg:.1f}°) on elbow+/shoulder−, "
            f"duration={args.duration}s × {legs} leg(s), rate={args.rate} Hz, "
            f"kp_scale={args.kp_scale}"
        )
        return 0

    cfg = Path(args.config)
    if not cfg.is_file():
        print(f"config not found: {cfg}")
        return 1

    arm = Arm.from_yaml(str(cfg))
    try:
        arm.enable()
        limb = arm.left if args.side == "left" else arm.right
        if limb is None or not limb.present:
            print(f"side {args.side} not configured in {cfg}")
            return 1

        states = limb.states()
        can_ids = [int(s.can_id) for s in states]
        kp_list, kd_list = _gains(can_ids, args.kp_scale)

        # Seed hold on this limb (and soft-hold other side at current if dual)
        for s in arm.states():
            kp = 10.0 if int(s.can_id) in (0x04, 0x05, 0x24, 0x25) else 12.0
            kd = 0.8 if int(s.can_id) in (0x04, 0x05, 0x24, 0x25) else 0.6
            arm.mit(s.can_id, kp=kp, kd=kd, q=s.position, dq=0.0, tau=0.0)

        arm.start_mit_loop(hz=1000.0, home=False)
        if args.no_gravity:
            limb.set_gravity_enabled(False)
        else:
            limb.set_gravity_enabled(True)
        if args.scale is not None:
            limb.set_gravity_scale(args.scale)

        states = limb.states()
        q0 = [float(s.position) for s in states]
        can_ids = list(limb.can_ids)
        q_goal = _apply_deltas(q0, can_ids, args.delta)
        kp_list, kd_list = _gains(can_ids, args.kp_scale)

        print("=== move_joints demo (visible shoulder/elbow step) ===")
        print(f"side={args.side} dual={arm.is_dual()}")
        print(f"gravity={limb.gravity_enabled()} scale={limb.gravity_scale()}")
        print(f"q0    {_fmt_q(q0)}")
        print(f"qgoal {_fmt_q(q_goal)}")
        elbow_id = 0x04 if args.side == "left" else 0x24
        shoulder_id = 0x05 if args.side == "left" else 0x25
        print(
            f"Δelbow(0x{elbow_id:02X})=+{args.delta:.2f} rad ({deg:.0f}°), "
            f"Δshoulder(0x{shoulder_id:02X})=-{args.delta:.2f} rad ({deg:.0f}°)"
        )
        print(
            f"duration={args.duration}s/leg  rate={args.rate}Hz  "
            f"round_trip={not args.one_way}  kp_scale={args.kp_scale}"
        )
        print(f"注意看 {args.side} 肩/肘；对侧应保持不动。")
        if not args.yes:
            ans = input("Enter 开始 / q 放弃: ").strip().lower()
            if ans in ("q", "quit", "n", "no"):
                print("aborted")
                return 1

        def do_leg(label: str, q_target: list[float]) -> None:
            print(f"\n-- {label} --")
            result = limb.move_joints(
                q_target,
                duration=args.duration,
                rate_hz=args.rate,
                kp=kp_list,
                kd=kd_list,
                settle_s=0.25,
                gravity=None,
            )
            print(
                f"  T={result.duration:.2f}s  max|err|={result.max_abs_err:.3f} rad "
                f"({math.degrees(result.max_abs_err):.1f}°)  "
                f"err={[f'{e:+.3f}' for e in result.err]}"
            )
            print(f"  meas {_fmt_q(result.q_meas)}")

        do_leg("go → goal (elbow+/shoulder−)", q_goal)
        if args.pause > 0:
            print(f"pause {args.pause:.1f}s at goal …")
            time.sleep(args.pause)

        if not args.one_way:
            do_leg("return → start", q0)
            if args.hold > 0:
                print(f"final hold {args.hold:.1f}s …")
                time.sleep(args.hold)
        else:
            if args.hold > 0:
                print(f"hold at goal {args.hold:.1f}s …")
                time.sleep(args.hold)

        print("\ndone.")
        return 0
    except KeyboardInterrupt:
        print("\ninterrupt")
        return 130
    finally:
        try:
            arm.disable()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
