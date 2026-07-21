import time

from dm_openarm import Arm, MitCommand


def main() -> None:
    arm = Arm.from_yaml("dm_openarm/config/arm.yaml")

    try:
        arm.enable()

        # start_mit_loop() 启动后会先自动回零，再返回
        arm.start_mit_loop(hz=1000.0)
        time.sleep(2.0)  # 等待回零完成

        while True:
            # Single motor — keyword arguments
            # arm.mit(0x01, kp=12.0, kd=0.6, q=0.40, dq=0.0, tau=0.0)
            # time.sleep(0.1)

            # Multiple motors — dict of MitCommand
            arm.mit({
                # 0x01: MitCommand(kp=12.0, kd=0.6, q=0.0),
                # 0x02: MitCommand(kp=12.0, kd=0.6, q=0.5, tau=0.2),
                0x03: MitCommand(kp=12.0, kd=0.6, q=1.0, tau=0.5),
                # 0x04: MitCommand(kp=12.0, kd=0.6, q=-0.8, tau=0.8),
                # 0x05: MitCommand(kp=12.0, kd=0.6, q=-0.8, tau=1.0),
            })
            time.sleep(0.1)

    finally:
        arm.stop_mit_loop()
        arm.disable()


if __name__ == "__main__":
    main()
