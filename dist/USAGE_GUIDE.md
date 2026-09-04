# dm_openarm SDK 使用指南

## 安装

当前 wheel 适用于 **CPython 3.12 / Linux x86_64**。其他 Python 版本或平台需要重新构建 wheel。

安装当前目录下的 wheel：

```bash
python -m pip install dm_openarm-0.1.2-*.whl
```

当前 SDK 默认控制频率为 **1000 Hz**，即当前支持的最高默认频率；控制频率不再从
`arm.yaml` 读取，YAML 中也不再配置 `control.loop_period_ms`。如需降低频率，运行时显式传入
`arm.start_mit_loop(hz=500.0)`。

## 电机部位与 ID 对照

下面是当前 `dm_openarm/config/arm.yaml` 的固定映射。API 不允许在运行时覆盖这些
ID；如更换电机或总线配置，请先修改 YAML。

| 手侧 | 电机位置 | 型号 | CAN ID | MST ID |
| --- | --- | --- | --- | --- |
| 左手 | `L_shoulder` | DM8009 | `0x00` | `0x10` |
| 左手 | `L_elbow` | DM8009 | `0x01` | `0x11` |
| 左手 | `L_wrist_2` | DM4310 | `0x02` | `0x12` |
| 左手 | `L_wrist_3` | DM4310 | `0x03` | `0x13` |
| 左手 | `L_end_effector` | DM4310 | `0x04` | `0x14` |
| 右手 | `R_shoulder` | DM8009 | `0x05` | `0x15` |
| 右手 | `R_elbow` | DM8009 | `0x06` | `0x16` |
| 右手 | `R_wrist_3` | DM4310 | `0x07` | `0x17` |
| 右手 | `R_wrist_2` | DM4310 | `0x08` | `0x18` |
| 右手 | `R_end_effector` | DM4310 | `0x09` | `0x19` |

也可以从配置对象读取当前映射，避免在应用里重复维护 ID：

```python
for limb in (arm.left, arm.right):
    for name, can_id in zip(limb.names, limb.can_ids):
        print(name, f"0x{can_id:02X}")
```

## 最小使用

```python
from dm_openarm import Arm

arm = Arm.from_yaml("dm_openarm/config/arm.yaml")

try:
    arm.enable()

    # 先把目标位置设置为当前反馈位置，避免启动时产生突变。
    for state in arm.states():
        arm.mit(
            state.can_id,
            kp=12.0,
            kd=0.6,
            q=state.position,
            dq=0.0,
            tau=0.0,
        )

    # 默认 1000 Hz，不回零。
    arm.start_mit_loop()

    # 由上层应用计算动力学/重力前馈后，写入 tau。
    arm.mit(0x01, kp=10.0, kd=0.8, q=0.2, dq=0.0, tau=0.0)

    for state in arm.states():
        if state.feedback_fresh(0.1):
            print(state.can_id, state.position, state.error_code)
finally:
    arm.disable()
```

`start_mit_loop(home=True)` 会让全部电机运动到已保存零位，只有确认安全时才使用。

## 主要 API

### `Arm`

- `Arm.from_yaml(path, side=None)`：加载配置，不连接设备；指定 `left`/`right` 时仅控制该侧。
- `enable()` / `disable()`：连接并使能 / 停止控制循环、失能并断开。
- `states()`：读取全总线反馈。
- `mit(can_id, kp, kd, q, dq, tau)`：更新单轴 MIT 命令。
- `mit({can_id: MitCommand(...)})`：批量更新命令。
- `start_mit_loop(hz=1000.0, home=False)`：启动控制循环。
- `stop_mit_loop()`：停止控制循环并发送零 MIT。
- `set_zero(can_id, persist=True)` / `set_zero_all(...)`：设置零位。
- `deadline_misses`：读取控制循环超期次数。
- `left` / `right`：访问左右侧 `Limb`。

### `Limb`

- `can_ids`、`names`、`n_joints`：本侧电机元数据。
- `states()`、`commands()`：读取本侧状态和命令。
- `mit(...)`：发送本侧 MIT 命令，会拒绝另一侧的 CAN ID。
- `set_zero(...)` / `set_zero_all(...)`：设置本侧零位。

### `MotorState`

主要字段：`can_id`、`mst_id`、`position`、`velocity`、`torque`、
`last_rx_age_s`、`rx_sequence`、`error_code`。

使用 `state.feedback_fresh(max_age_s=0.1)` 判断反馈是否在指定时间内更新。

## 配置

默认配置：

```text
dm_openarm/config/arm.yaml
```

当前配置必须包含 `left.motors` 和 `right.motors`。SDK 不包含轨迹规划、动力学模型
或重力辨识；这些功能由上层应用计算，结果通过 `MitCommand.tau` 传入。

## 关闭

推荐始终使用 `try/finally`，退出时调用：

```python
arm.disable()
```
