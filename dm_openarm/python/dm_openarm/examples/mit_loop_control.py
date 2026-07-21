"""Minimal multi-joint trajectory example (hardware).

Uses time-boxed move_joints; does not wait on position tolerance
(soft MIT + gravity residual may leave steady lag).
"""
from dm_openarm import Arm


def main() -> None:
    arm = Arm.from_yaml("dm_openarm/config/arm.yaml")

    try:
        arm.enable()
        # Seed commands at current pose, then start without homing.
        for s in arm.states():
            arm.mit(s.can_id, kp=12.0, kd=0.6, q=s.position, dq=0.0, tau=0.0)
        arm.start_mit_loop(hz=1000.0, home=False)
        arm.set_gravity_enabled(True)

        q = [s.position for s in arm.states()]
        if len(q) >= 1:
            q[0] += 0.10
            result = arm.move_joints(q, duration=2.0)
            print("max|err|", result.max_abs_err)
            q[0] -= 0.10
            result = arm.move_joints(q, duration=2.0)
            print("max|err|", result.max_abs_err)
    finally:
        arm.disable()


if __name__ == "__main__":
    main()
