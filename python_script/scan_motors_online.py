#!/usr/bin/env python3
"""Probe configured motors without enabling them or sending MIT commands."""
from __future__ import annotations

import argparse
import math
import time
from collections import defaultdict

from dm_openarm import Arm

DEFAULT_CONFIG = "dm_openarm/config/arm.yaml"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Passively query motor online status")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--duration", type=float, default=2.5)
    parser.add_argument("--min-hits", type=int, default=5)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be positive and finite")
    if args.min_hits < 1:
        parser.error("--min-hits must be positive")

    print("=== passive motor online scan ===", flush=True)
    print(f"config={args.config}", flush=True)
    arm = None
    hits: dict[int, int] = defaultdict(int)
    result = 0
    try:
        arm = Arm.from_yaml(args.config)
        arm.connect_passive()
        left_ids = set(arm.left.can_ids)
        right_ids = set(arm.right.can_ids)
        order = list(arm.left.can_ids) + list(arm.right.can_ids)
        deadline = time.monotonic() + args.duration
        while (remaining := deadline - time.monotonic()) > 0:
            # One hit per axis per query round, never cached motion samples.
            for cid in set(arm.probe_status(min(0.2, remaining))):
                if cid in left_ids or cid in right_ids:
                    hits[cid] += 1
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
    except KeyboardInterrupt:
        print("FAIL: scan interrupted", flush=True)
        result = 130
    except Exception as exc:
        print(f"FAIL: scan failed: {exc}", flush=True)
        result = 1
    finally:
        if arm is not None:
            try:
                arm.disable()  # Passive owners only close their connection.
            except Exception as exc:
                print(f"FAIL: shutdown failed: {exc}", flush=True)
                result = 2

    if result:
        return result
    print(f"{'can_id':>8} {'side':>6} {'replies':>8} {'status':>8}", flush=True)
    offline = []
    for cid in order:
        online = hits[cid] >= args.min_hits
        if not online:
            offline.append(cid)
        side = "L" if cid in left_ids else "R"
        print(f"0x{cid:02X} {side:>6} {hits[cid]:8d} {'ONLINE' if online else 'OFFLINE':>8}", flush=True)
    print(f"summary: {len(order) - len(offline)}/{len(order)} online", flush=True)
    print("Online means query replies were observed; motor health and position are not evaluated.", flush=True)
    if offline:
        print("OFFLINE: " + ", ".join(f"0x{cid:02X}" for cid in offline), flush=True)
        return 3
    print("PASS: all configured motors online; connection closed", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
