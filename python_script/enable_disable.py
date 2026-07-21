from dm_openarm import Arm


def main() -> None:
    arm = Arm.from_yaml("dm_openarm/config/arm.yaml")
    arm.enable()
    input("Press Enter to disable...")
    arm.disable()


if __name__ == "__main__":
    main()
