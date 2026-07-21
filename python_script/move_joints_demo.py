#!/usr/bin/env python3
"""Small-range multi-joint trajectory demo with coupled gravity.

Safety:
  - Uses start_mit_loop(home=False); does NOT slam to zero.
  - Default deltas are small; confirm before motion.
  - Soft MIT + residual gravity may leave steady lag — that is expected;
    MoveResult reports max_abs_err and does not hard-fail on position.

Acceptance notes:
  - Gravity ON: tracking should not sag badly on shoulder/elbow.
  - Gravity OFF: more collapse on 0x04/0x05.
  - Hold after move ~5 s should match hold_with_gravity quality.
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

from dm_openarm import Arm

CONFIG = "dm_openarm/config/arm_5dof.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(description="Joint trajectory demo with gravity")
    parser.add_argument("--config", default=CONFIG)
    parser.add_argument("--scale", type=float, default=None, help="gravity scale override")
    parser.add_argument("--duration", type=float, default=4.0, help="move duration [s]")
    parser.add_argument(
        "--delta",
        type=float,
        default=0.15,
        help="shoulder/elbow step magnitude [rad] (default 0.15)",
    )
    parser.add_argument("--rate", type=float, default=100.0, help="command stream rate [Hz]")
    parser.add_argument("--hold", type=float, default=5.0, help="hold time after move [s]")
    parser.add_argument("--no-gravity", action="store_true", help="disable gravity FF")
    parser.add_argument("--yes", action="store_true", help="skip confirmation prompt")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print planned targets only (no hardware)",
    )
    args = parser.parse_args()

    if args.dry_run:
        print(
            f"dry-run: duration={args.duration}s delta=±{args.delta} rad "
            f"on elbow/shoulder (indices 3,4), rate={args.rate} Hz"
        )
        return 0

    cfg = Path(args.config)
    if not cfg.is_file():
        print(f"config not found: {cfg}")
        return 1

    arm = Arm.from_yaml(str(cfg))
    try:
        arm.enable()
        # Soft hold current pose as command seed
        states = arm.states()
        for s in states:
            kp = 10.0 if s.can_id in (0x04, 0x05) else 12.0
            kd = 0.8 if s.can_id in (0x04, 0x05) else 0.6
            arm.mit(s.can_id, kp=kp, kd=kd, q=s.position, dq=0.0, tau=0.0)

        arm.start_mit_loop(hz=1000.0, home=False)
        if args.no_gravity:
            arm.set_gravity_enabled(False)
        else:
            arm.set_gravity_enabled(True)
        if args.scale is not None:
            arm.set_gravity_scale(args.scale)

        states = arm.states()
        q0 = [s.position for s in states]
        can_ids = [s.can_id for s in states]
        q_goal = list(q0)
        # Elbow (0x04) + / shoulder (0x05) − small step if present
        for i, cid in enumerate(can_ids):
            if cid == 0x04:
                q_goal[i] = q0[i] + args.delta
            elif cid == 0x05:
                q_goal[i] = q0[i] - args.delta

        print("=== move_joints demo ===")
        print(f"gravity={arm.gravity_enabled()} scale={arm.gravity_scale()}")
        print(f"q0    = {[f'{x:+.3f}' for x in q0]}")
        print(f"qgoal = {[f'{x:+.3f}' for x in q_goal]}")
        print(f"duration={args.duration}s rate={args.rate}Hz")
        if not args.yes:
            ans = input("Enter 开始 / q 放弃: ").strip().lower()
            if ans in ("q", "quit", "n", "no"):
                print("aborted")
                return 1

        result = arm.move_joints(
            q_goal,
            duration=args.duration,
            rate_hz=args.rate,
            settle_s=0.3,
            gravity=None,
        )
        print(
            f"done: T={result.duration:.2f}s max|err|={result.max_abs_err:.4f} rad "
            f"err={[f'{e:+.3f}' for e in result.err]}"
        )
        print(f"holding {args.hold:.1f}s …")
        time.sleep(args.hold)
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
