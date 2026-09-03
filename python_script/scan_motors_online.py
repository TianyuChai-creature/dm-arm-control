#!/usr/bin/env python3
"""Scan configured motors for online feedback (MIT all-zero, no motion).

Default: dual arm left 0x00–0x04 + right 0x05–0x09.
"""
from __future__ import annotations

import argparse
import time
from collections import defaultdict

from dm_openarm import Arm

DEFAULT_CONFIG = "dm_openarm/config/arm.yaml"


def is_feedback(s) -> bool:
    return bool(s.feedback_fresh(0.1))


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
    last_sequence: dict[int, int] = {}

    try:
        arm.enable()
        left_ids = set(arm.left.can_ids)
        right_ids = set(arm.right.can_ids)
        print(
            f"left={sorted(f'0x{c:02X}' for c in left_ids)} "
            f"right={sorted(f'0x{c:02X}' for c in right_ids)}",
            flush=True,
        )

        t0 = time.perf_counter()
        while time.perf_counter() - t0 < args.duration:
            arm._arm.send_zero_mit_all()
            for s in arm.states():
                cid = int(s.can_id)
                sequence = int(s.rx_sequence)
                if is_feedback(s) and sequence != last_sequence.get(cid):
                    hits[cid] += 1
                    last[cid] = s
                    last_sequence[cid] = sequence
            time.sleep(0.01)

        order = [int(s.can_id) for s in arm.states()]
        print(flush=True)
        print(
            f"{'can_id':>8} {'side':>6} {'hits':>6} {'status':>8} "
            f"{'pos':>10} {'vel':>10} {'tau':>10} {'err':>5}",
            flush=True,
        )
        online_n = 0
        offline = []
        faulty = []
        for cid in order:
            if cid in left_ids:
                side = "L"
            elif cid in right_ids:
                side = "R"
            else:
                side = "?"
            n = hits.get(cid, 0)
            s = last.get(cid)
            fault = s is not None and int(s.error_code) != 0
            ok = n >= args.min_hits and not fault
            if ok:
                online_n += 1
                st = "ONLINE"
            elif fault:
                faulty.append(cid)
                st = "FAULT"
            else:
                offline.append(cid)
                st = "OFFLINE"
            if s is not None:
                print(
                    f"0x{cid:02X} {side:>6} {n:6d} {st:>8} "
                    f"{s.position:10.4f} {s.velocity:10.4f} {s.torque:10.4f} "
                    f"{s.error_code:5d}",
                    flush=True,
                )
            else:
                print(f"0x{cid:02X} {side:>6} {n:6d} {st:>8}", flush=True)

        total = len(order)
        print(flush=True)
        print(f"summary: {online_n}/{total} online", flush=True)
        if offline:
            print("OFFLINE: " + ", ".join(f"0x{c:02X}" for c in offline), flush=True)
        if faulty:
            print("FAULT: " + ", ".join(f"0x{c:02X}" for c in faulty), flush=True)
            return 4
        if offline:
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
