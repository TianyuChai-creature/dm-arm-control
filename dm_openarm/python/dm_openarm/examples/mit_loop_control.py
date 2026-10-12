"""Minimal MIT control example (hardware)."""
import sys
import time

from dm_openarm import Arm, MitCommand


def main() -> None:
    arm = Arm.from_yaml("dm_openarm/config/arm.yaml")
    # Replace these illustrative gains with values calibrated for your station.
    gains = [MitCommand(kp=12.0, kd=0.6) for _ in arm.left.can_ids + arm.right.can_ids]

    try:
        arm.enable_seeded(gains, hz=250.0)
        time.sleep(2.0)
    except KeyboardInterrupt:
        print("Software stop requested; this is not a hardware emergency stop.")
    finally:
        try:
            arm.disable()
        except Exception as exc:
            print(f"Software stop failed ({arm.safety_state}): {exc}", file=sys.stderr)
            if arm.safety_state == "SHUTDOWN_UNKNOWN" or "shutdown unknown" in str(exc):
                print("Disable unconfirmed: use the independent hardware stop and inspect before recovery.", file=sys.stderr)
            raise


if __name__ == "__main__":
    main()
