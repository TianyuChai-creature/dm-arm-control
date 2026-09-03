"""Minimal MIT control example (hardware)."""
from dm_openarm import Arm


def main() -> None:
    arm = Arm.from_yaml("dm_openarm/config/arm.yaml")

    try:
        arm.enable()
        # Seed commands at current pose, then start without homing.
        for s in arm.states():
            arm.mit(s.can_id, kp=12.0, kd=0.6, q=s.position, dq=0.0, tau=0.0)
        arm.start_mit_loop(hz=1000.0, home=False)
        time.sleep(2.0)
    finally:
        arm.disable()


if __name__ == "__main__":
    main()
