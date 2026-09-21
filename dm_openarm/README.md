# dm_openarm 使用说明

`dm_openarm` 是一个面向当前 5 自由度达妙机械臂的 C++17 控制库，并提供
Python API。底层复用达妙 USB-CANFD SDK，当前版本只封装 MIT 模式。

本库的典型使用方式是：

- C++ 后台线程以 1 kHz 发送 MIT 控制帧。
- Python 只负责更新每个电机的目标位置、增益和前馈力矩。
- 电机配置从 `config/arm.yaml` 读取。

## 当前硬件配置

与工位实测 `resources/u2canfd` 对齐（见 `resources/u2canfd/HARDWARE_CONFIG.md`）。

| 项目 | 值 | 单位/说明 |
| --- | --- | --- |
| 设备驱动 | `libdm_device.so`（dmcan API） | 与 u2canfd 相同栈，不是旧 `libu2canfd.a` |
| 链路模式 | **经典 CAN** | YAML：`canfd: false`，`brs: false` |
| USB-CANFD SN | `52A871B1AA5EF4E239371A5083463F26` | 如果适配器不同，需要改 YAML |
| nominal baud | `1000000` | bit/s |
| data baud | `1000000` | classic CAN 下数据域不使用 |
| 控制模式 | MIT | 当前只支持 MIT |
| 控制循环 | `1000` | Hz，默认 1 kHz |

> 工位实测：CAN FD（含数据域 1M/5M）无回帧；请勿把文档默认写成 5M CANFD。

电机顺序和 ID：

| CAN ID | MST ID | 型号 | 位置 | YAML 名称 |
| --- | --- | --- | --- | --- |
| `0x04` | `0x14` | DM4310 | 左末端 | `L_end_effector` |
| `0x02` | `0x12` | DM4310 | 左腕二 | `L_wrist_2` |
| `0x03` | `0x13` | DM4310 | 左腕三 | `L_wrist_3` |
| `0x01` | `0x11` | DM8009 | 左肘 | `L_elbow` |
| `0x00` | `0x10` | DM8009 | 左肩 | `L_shoulder` |
| `0x09` | `0x19` | DM4310 | 右末端 | `R_end_effector` |
| `0x08` | `0x18` | DM4310 | 右腕二 | `R_wrist_2` |
| `0x07` | `0x17` | DM4310 | 右腕三 | `R_wrist_3` |
| `0x06` | `0x16` | DM8009 | 右肘 | `R_elbow` |
| `0x05` | `0x15` | DM8009 | 右肩 | `R_shoulder` |

说明：当前 SDK 中没有单独的 `DM8009P` 枚举，本库统一写作 `DM8009`。

## 配置文件

默认配置文件：

```text
config/arm.yaml
```

如果 USB-CANFD 适配器 SN、波特率或电机 ID 不同，先修改这个 YAML。修改
YAML 后不需要重新编译；修改 C++ 或 Python 包代码后才需要重新安装/编译。

## SDK API

### Python

| 对象 | API | 作用 |
| --- | --- | --- |
| `Arm` | `from_yaml(path, side=None)` | 加载双臂或指定单臂配置，不连接设备 |
| `Arm` | `connect_passive()` / `probe_status(timeout_s=0.2)` | 打开 USB 后逐个只读查询 CAN ID，不使能电机；返回有回复的 ID |
| `Arm` | `enable()` / `disable()` | 连接并使能 / 停止并失能 |
| `Arm` | `enable_seeded(gains, hz=250, ...)` | 原生使能、取反馈并 seed，进入 HOLD |
| `Arm` | `states()` | 读取全总线状态 |
| `Arm` | `mit(...)` | 按 CAN ID 写单轴或批量 MIT 命令 |
| `Arm` | `start_mit_loop(hz=1000, home=False)` | 默认以最高频率 1000 Hz 启动控制环 |
| `Arm` | `stop_mit_loop()` | 停止控制环并发送零 MIT |
| `Arm` | `hold_command()` | 保持最后实际执行的命令，前馈不衰减 |
| `Arm` | `hold(reset_fault=False)` | 实测位置保持；可显式解除超时锁存 |
| `Arm` | `safety_state` / `fault` / `sent_commands()` | 原生保护状态与实际下发命令 |
| `Arm` | `set_zero(...)` / `set_zero_all(...)` | 设置电机零位 |
| `Arm` | `deadline_misses` | 读取控制环超期次数 |
| `Arm` | `left` / `right` | 获取左右侧 `Limb` |
| `Limb` | `can_ids` / `names` / `states()` | 读取本侧元数据和状态 |
| `Limb` | `mit(...)` | 本侧 MIT 命令 |
| `Limb` | `commands()` | 读取本侧当前命令 |
| `Limb` | `set_zero(...)` / `set_zero_all(...)` | 设置本侧零位 |

基础类型：`MitCommand(kp, kd, q, dq, tau)`、`MotorConfig`、`MotorState`、
`MotorTimingStats`、`TimingStats`、`ArmConfig`、`MotorModel` 和 `ControlMode`。`MotorState.feedback_fresh(0.1)`
可判断最近 0.1 秒内是否收到反馈。

### C++

| 类型/函数 | API |
| --- | --- |
| 配置 | `load_arm_config(path)` |
| `DmArm` | `connect()`、`enable()`、`disable()`、`disconnect()`、`connected()` |
| `DmArm` | `probe_status(timeout_s)` 返回查询后有回复的 CAN ID，不使能电机 |
| `DmArm` | `send_mit_all()`、`send_zero_mit_all()`、`states()` |
| `DmArm` | `set_zero()`、`set_zero_all()`、`config()` |
| `MitLoopController` | `start()`、`stop()`、`running()` |
| `MitLoopController` | `set_command()`、`set_all_commands()`、`commands()` |
| `MitLoopController` | `deadline_misses()` |
| `MitLoopController` | `timing_stats()` |

SDK 不包含轨迹规划、动力学模型或重力辨识；上层应用计算前馈力矩后写入
`MitCommand.tau`。

## 单位约定

Python 和 C++ API 中的主要物理量单位如下：

| 字段 | 含义 | 单位 |
| --- | --- | --- |
| `q` | 目标绝对位置 | rad |
| `dq` | 目标速度 | rad/s |
| `tau` | 前馈力矩 | N·m |
| `kp` | 位置增益 | N·m/rad |
| `kd` | 速度阻尼增益 | N·m/(rad/s) |
| `position` | 反馈位置 | rad |
| `velocity` | 反馈速度 | rad/s |
| `torque` | 反馈力矩 | N·m |
| `feedback_hz` | 最近 1 秒滑动窗口反馈频率 | Hz |
| `feedback_interval_s` | 反馈间隔 | s |
| `last_rx_age_s` | 距离最近反馈的时间 | s |
| `rx_sequence` | 已接收反馈帧计数 | 帧 |
| `error_code` | 电机反馈错误码 | - |
| `hz` | 控制频率 | Hz |
| `timeout` / `zero_timeout` | 等待时间 | s |

`q` 是绝对位置，不是相对位移。它的零点来自电机中保存的零位参数。

例如：

```python
arm.mit(0x01, kp=12.0, kd=0.6, q=0.8, dq=0.0, tau=0.0)
```

表示让 `0x01` 电机运动到绝对位置 `0.8 rad`，不是在当前位置上增加
`0.8 rad`。

达妙 MIT 控制帧只有 `kp`、`kd`、`q`、`dq`、`tau` 这 5 个控制量，没有单独的
`tau_gain`。如果要调整前馈力矩强弱，直接调整 `tau` 数值。初次位置控制建议
保持 `tau=0.0`。

## Python 安装

在包目录内安装：

```bash
cd dm_openarm
python -m pip install .
```

如果你在仓库根目录，也可以这样安装：

```bash
python -m pip install ./dm_openarm
```

开发调试时建议使用 editable install：

```bash
python -m pip install -e ./dm_openarm
```

注意：如果你在仓库根目录直接执行 `python -m pip install .`，但根目录没有
`pyproject.toml`，会报：

```text
Directory '.' is not installable. Neither 'setup.py' nor 'pyproject.toml' found.
```

正确做法是进入 `dm_openarm/` 后安装，或从仓库根目录安装 `./dm_openarm`。

## Python 快速开始

基础结构建议始终使用 `try/finally`，保证异常或 `Ctrl+C` 时可以停止 MIT loop
并失能电机：

```python
from dm_openarm import Arm

arm = Arm.from_yaml("dm_openarm/config/arm.yaml")

try:
    arm.enable()
    arm.start_mit_loop(hz=1000.0, home=False)

    arm.mit(0x01, kp=12.0, kd=0.6, q=0.4, dq=0.0, tau=0.0)

finally:
    arm.stop_mit_loop()
    arm.disable()
```

如果脚本在仓库根目录运行，配置路径使用：

```python
Arm.from_yaml("dm_openarm/config/arm.yaml")
```

如果脚本在 `dm_openarm/` 包目录内运行，配置路径使用：

```python
Arm.from_yaml("config/arm.yaml")
```

## Python API

### 创建机械臂对象

```python
from dm_openarm import Arm

arm = Arm.from_yaml("dm_openarm/config/arm.yaml")
```

`Arm.from_yaml(path)` 会读取双臂配置，但不会连接硬件，也不会使能电机。
扫描或维护单臂时使用 `Arm.from_yaml(path, side="left"|"right")`，SDK 只注册、使能和发送该侧5轴。

### 使能和失能

```python
arm.enable()
arm.disable()
```

`arm.enable()` 会连接 USB-CANFD，并通过底层达妙 SDK 使能 YAML 中配置的所有
电机。

`arm.disable()` 会先停止 MIT loop，再失能电机。

### 读取状态

```python
for state in arm.states():
    print(
        state.can_id,
        state.mst_id,
        state.position,
        state.velocity,
        state.torque,
        state.feedback_hz,
        state.feedback_interval_s,
        state.last_rx_age_s,
        state.rx_sequence,
        state.error_code,
    )
```

状态字段单位：

| 字段 | 单位 |
| --- | --- |
| `position` | rad |
| `velocity` | rad/s |
| `torque` | N·m |
| `feedback_hz` | Hz |
| `feedback_interval_s` | s |
| `last_rx_age_s` | s |
| `rx_sequence` | 帧 |
| `error_code` | - |

`state.feedback_fresh(0.1)` 可用于判断最近 0.1 秒内是否收到反馈。

### 收发频率统计

```python
stats = arm.timing_stats()
print(
    stats.connected,
    stats.running,
    stats.target_tx_hz,
    stats.actual_tx_hz,
    stats.tx_cycles,
    stats.deadline_misses,
)
for motor in stats.motors:
    print(
        motor.can_id,
        motor.rx_hz,
        motor.rx_interval_s,
        motor.last_rx_age_s,
        motor.rx_frames,
    )
```

发送与接收频率均使用固定 1 秒滑动窗口。`actual_tx_hz` 是整臂 MIT 控制周期频率；
总线发送帧率约为该值乘以电机数量。逐电机 `rx_hz` 只统计有效运动反馈，参数回复和无效帧不计入。
样本不足两个或最近 1 秒没有有效数据时频率为 `0.0`。

### 设置零位

Python API 已经开放设置零位：

```python
arm.enable()
arm.set_zero(0x01, persist=True)
arm.disable()
```

设置全部电机零位：

```python
arm.enable()
arm.set_zero_all(persist=True)
arm.disable()
```

参数说明：

- `can_id`：目标电机 CAN ID，例如 `0x01`。
- `persist=True`：把当前位置设置为零位，并保存到电机 flash。
- `persist=False`：只做临时设零，不保证程序结束或断电后保持。

重要区别：

- `set_zero(...)` 是“把当前位置写成新的零位”。
- `start_mit_loop(..., home=True)` 才会运动到已保存零位 `q=0`，不是重新设置零位。

默认建议使用 `persist=True`，这样零位会在后续 Python/C++ 程序和断电重启后保持。
但不要频繁反复写 flash；只在机械零位确认正确后执行。

### 启动 MIT 后台循环

```python
arm.start_mit_loop(hz=1000.0, home=False)
```

行为：

- 启动 C++ 后台 MIT 控制线程。
- 控制线程按 `hz` 频率发送当前命令，默认 `1000 Hz`。
- 默认不回零；传入 `home=True` 才会命令所有电机运动到已保存零位。
- 显式回零时等待容差为 `0.05 rad`，`zero_timeout` 默认 `5.0 s`。

注意：这个回零动作可能导致真实机械臂运动。运行前必须确认机械臂周围没有干涉。

停止后台循环：

```python
arm.stop_mit_loop()
```

停止时底层会发送一次零 MIT 指令，然后停止线程。

### 发送 MIT 命令

统一使用：

```python
arm.mit(...)
```

单电机控制：

```python
arm.mit(
    0x01,
    kp=12.0,
    kd=0.6,
    q=0.4,
    dq=0.0,
    tau=0.0,
)
```

多电机控制：

```python
from dm_openarm import MitCommand

arm.mit({
    0x04: MitCommand(kp=12.0, kd=0.6, q=0.0, dq=0.0, tau=0.0),
    0x02: MitCommand(kp=12.0, kd=0.6, q=0.5, dq=0.0, tau=0.2),
    0x03: MitCommand(kp=12.0, kd=0.6, q=1.0, dq=0.0, tau=0.5),
    0x01: MitCommand(kp=10.0, kd=0.8, q=-0.8, dq=0.0, tau=0.8),
    0x00: MitCommand(kp=10.0, kd=0.8, q=-0.8, dq=0.0, tau=1.0),
})
```

多电机 dict 中没有出现的电机，会保持之前已经设置的命令；首次启动时默认是全零
增益、全零力矩。

`MitCommand` 字段单位：

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `kp` | N·m/rad | 位置增益 |
| `kd` | N·m/(rad/s) | 速度阻尼增益 |
| `q` | rad | 目标绝对位置 |
| `dq` | rad/s | 目标速度 |
| `tau` | N·m | 前馈力矩 |

初始测试建议：

| 电机 | `kp` | `kd` | `tau` |
| --- | --- | --- | --- |
| DM4310 | `12.0` | `0.6` | `0.0` |
| DM8009 | `10.0` | `0.8` | `0.0` |

如果电机不明显运动，可以逐步提高 `kp`，但应小步调整，并确认没有机械干涉。

## Python 示例脚本

示例脚本位于仓库根目录的 `python_script/`。

运行前先安装包：

```bash
python -m pip install ./dm_openarm
```

然后从仓库根目录运行：

```bash
python python_script/enable_disable.py
python python_script/read_states.py
python python_script/mit_control_one_motor.py
```

脚本说明：

| 脚本 | 作用 |
| --- | --- |
| `enable_disable.py` | 使能所有电机，等待回车后失能 |
| `read_states.py` | 使能后读取并打印电机状态 |
| `mit_control_one_motor.py` | 启动 MIT loop，并按脚本中的目标持续发送 MIT 命令 |

`mit_control_one_motor.py` 会真实驱动机械臂。运行前确认机械臂没有碰撞风险。
按 `Ctrl+C` 退出时，脚本会进入 `finally`，停止 MIT loop 并失能电机。

## C++ 构建

构建 C++ 库、示例程序和无硬件测试：

```bash
cd dm_openarm
mkdir -p build
cd build
cmake ..
make
ctest --output-on-failure
```

构建完成后，示例程序在 `dm_openarm/build/`。

## C++ 示例程序

```bash
cd dm_openarm/build

./check_comm ../config/arm.yaml
./enable_disable ../config/arm.yaml
./hold_position ../config/arm.yaml
./set_zero_position ../config/arm.yaml all
./arm_mit_control ../config/arm.yaml
```

### C++ 设零

```bash
./set_zero_position ../config/arm.yaml all
./set_zero_position ../config/arm.yaml 0x01
./set_zero_position ../config/arm.yaml 1
```

默认会保存到 flash。临时设零使用：

```bash
./set_zero_position ../config/arm.yaml all --runtime-only
```

### C++ FIFO MIT 控制

启动：

```bash
./arm_mit_control ../config/arm.yaml
```

另一个终端发送命令：

```bash
echo "status" > /tmp/dm_openarm_mit_control.cmd
echo "move 0x01 0.08" > /tmp/dm_openarm_mit_control.cmd
echo "home 0x01" > /tmp/dm_openarm_mit_control.cmd
echo "home all" > /tmp/dm_openarm_mit_control.cmd
echo "stop" > /tmp/dm_openarm_mit_control.cmd
```

FIFO 示例中的 `move` 是示例程序自己的命令格式，内部有限制和斜坡：

- 单次 `move` 限制：`[-0.10, 0.10] rad`
- 目标位置斜坡：约 `0.6 rad/s`
- 默认 MIT 参数：`kp=8.0`，`kd=0.4`，`dq=0.0 rad/s`，`tau=0.0 N·m`

Python 的 `arm.mit(...)` 不自动使用 FIFO 示例里的单步限制和斜坡。如果需要更平滑
的运动，需要在 Python 业务脚本里分多次更新目标位置，或后续在库里新增平滑轨迹 API。

## C++ API 简要说明

主要头文件：

| 头文件 | 作用 |
| --- | --- |
| `dm_openarm/yaml_loader.hpp` | 加载 YAML 配置 |
| `dm_openarm/dm_arm.hpp` | 连接、失能、发送 MIT、读取状态、设零 |
| `dm_openarm/mit_loop_controller.hpp` | 后台 MIT 控制循环 |
| `dm_openarm/types.hpp` | `MitCommand`、`MotorState` 等类型 |

最小 C++ 示例：

```cpp
#include "dm_openarm/dm_arm.hpp"
#include "dm_openarm/mit_loop_controller.hpp"
#include "dm_openarm/types.hpp"
#include "dm_openarm/yaml_loader.hpp"

#include <chrono>
#include <thread>

int main()
{
auto config = dm_openarm::load_arm_config("../config/arm.yaml");
dm_openarm::DmArm arm(config);

arm.enable();

  dm_openarm::MitLoopController loop(arm);
  loop.start(1000.0);
  loop.set_command(
    0x01,
    dm_openarm::MitCommand{12.0, 0.6, 0.4, 0.0, 0.0});

  std::this_thread::sleep_for(std::chrono::seconds(2));

loop.stop();
arm.disable();
arm.disconnect();
  return 0;
}
```

在其他 CMake 项目中作为源码依赖：

```cmake
add_subdirectory(path/to/dm_openarm)
target_link_libraries(my_app PRIVATE dm_openarm)
```

## 安全注意事项

- `q` 是绝对位置，单位 rad，不是相对移动量。
- `start_mit_loop()` 默认不回零；只有 `home=True` 会运动到已保存零位。
- `set_zero()` 会改写电机零位，`persist=True` 会写 flash，不要频繁调用。
- `arm.mit(...)` 会立即更新目标命令；Python API 当前没有自动斜坡限制。
- 初次测试建议让单个电机、小角度、低增益运行。
- 任何会运动机械臂的脚本都应使用 `try/finally`，确保退出时执行
  `stop_mit_loop()` 和 `disable()`。
