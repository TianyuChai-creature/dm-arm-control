"""Low-level enable/disable demonstration; no seeded MIT loop or CommandGuard."""
from dm_openarm import Arm


def main() -> None:
    arm = Arm.from_yaml("config/arm.yaml")
    try:
        arm.enable()
        input("Press Enter for software disable (not a hardware emergency stop)...")
    finally:
        arm.disable()  # Propagate an unconfirmed shutdown instead of reporting success.


if __name__ == "__main__":
    main()
