#!/usr/bin/env python3
"""Scan configured motors for online feedback (MIT all-zero, no motion).

Default: dual arm left 0x01–0x05 + right 0x21–0x25.
"""
from __future__ import annotations

import argparse
import time
from collections import defaultdict

from dm_openarm import Arm

DEFAULT_CONFIG = "dm_openarm/config/arm.yaml"


def is_feedback(s) -> bool:
    return (
        s.feedback_interval_s > 0.0
        or abs(s.position) > 1e-9
        or abs(s.velocity) > 1e-9
        or abs(s.torque) > 1e-9
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Scan motors online status")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--duration", type=float, default=2.5)
    parser.add_argument("--min-hits", type=int, default=5)
    args = parser.parse_args()

    print("=== scan motors online ===", flush=True)
    print(f"config={args.config}", flush=True)

    arm = Arm.from_yaml(args.config)
    hits: dict[int, int] = defaultdict(int)
    last: dict[int, object] = {}

    try:
        arm.enable()
        left_ids = set(arm.left.can_ids) if arm.left else set()
        right_ids = set(arm.right.can_ids) if arm.right else set()
        print(
            f"left={sorted(f'0x{c:02X}' for c in left_ids)} "
            f"right={sorted(f'0x{c:02X}' for c in right_ids)} "
            f"dual={arm.is_dual()}",
            flush=True,
        )

        t0 = time.perf_counter()
        while time.perf_counter() - t0 < args.duration:
            arm._arm.send_zero_mit_all()
            for s in arm.states():
                cid = int(s.can_id)
                if is_feedback(s):
                    hits[cid] += 1
                    last[cid] = s
            time.sleep(0.01)

        order = [int(s.can_id) for s in arm.states()]
        print(flush=True)
        print(
            f"{'can_id':>8} {'side':>6} {'hits':>6} {'status':>8} "
            f"{'pos':>10} {'vel':>10} {'tau':>10}",
            flush=True,
        )
        online_n = 0
        offline = []
        for cid in order:
            if cid in left_ids:
                side = "L"
            elif cid in right_ids:
                side = "R"
            else:
                side = "?"
            n = hits.get(cid, 0)
            ok = n >= args.min_hits
            if ok:
                online_n += 1
                st = "ONLINE"
            else:
                offline.append(cid)
                st = "OFFLINE"
            s = last.get(cid)
            if s is not None:
                print(
                    f"0x{cid:02X} {side:>6} {n:6d} {st:>8} "
                    f"{s.position:10.4f} {s.velocity:10.4f} {s.torque:10.4f}",
                    flush=True,
                )
            else:
                print(f"0x{cid:02X} {side:>6} {n:6d} {st:>8}", flush=True)

        total = len(order)
        print(flush=True)
        print(f"summary: {online_n}/{total} online", flush=True)
        if offline:
            print("OFFLINE: " + ", ".join(f"0x{c:02X}" for c in offline), flush=True)
            return 3
        print("PASS: all configured motors online", flush=True)
        return 0
    except Exception as exc:
        print(f"FAIL: {exc}", flush=True)
        return 1
    finally:
        try:
            arm.disable()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
