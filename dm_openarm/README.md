# dm_openarm 使用说明

当前发行版本：**0.1.13**。

本版统一设零、被动扫描、停止状态及空命令行为，升级说明见
[0.1.13 更新日志](../dist/RELEASE_NOTES_0.1.13.md)，整体架构见[根目录 README](../README.md#架构怎么用-api)。

`dm_openarm` 是一个面向当前 5 自由度达妙机械臂的 C++17 控制库，并提供
Python API。底层复用达妙 USB-CANFD SDK，当前版本只封装 MIT 模式。

本库的典型使用方式是：

- 默认 guarded 启动以 250 Hz 发送 MIT 控制帧，运行时最高可指定 1000 Hz。
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
| 控制循环 | `250` / `1000` | Hz，guarded 默认 250，兼容入口默认及上限 1000 |

> 工位实测：CAN FD（含数据域 1M/5M）无回帧；请勿把文档默认写成 5M CANFD。

电机顺序和 ID：

| CAN ID | MST ID | 型号 | 位置 | YAML 名称 |
| --- | --- | --- | --- | --- |
| `0x04` | `0x14` | DM4340P | 左末端 | `L_end_effector` |
| `0x02` | `0x12` | DM4340P | 左腕二 | `L_wrist_2` |
| `0x03` | `0x13` | DM4340P | 左腕三 | `L_wrist_3` |
| `0x01` | `0x11` | DM8009 | 左肘 | `L_elbow` |
| `0x00` | `0x10` | DM8009 | 左肩 | `L_shoulder` |
| `0x09` | `0x19` | DM4340P | 右末端 | `R_end_effector` |
| `0x08` | `0x18` | DM4340P | 右腕二 | `R_wrist_2` |
| `0x07` | `0x17` | DM4340P | 右腕三 | `R_wrist_3` |
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
| `Arm` | `enable_seeded(gains, hz=250, ...)` | 原生 seed，首次完整保持批次提交成功后返回 |
| `Arm` | `states()` | 读取全总线状态 |
| `Arm` | `mit(...)` | 按 CAN ID 写单轴或批量 MIT 命令 |
| `Arm` | `start_mit_loop(hz=1000, home=False)` | 兼容裸入口，新对象不会自动开启 CommandGuard |
| `Arm` | `stop_mit_loop()` | 停环、发送零 MIT 和失能帧，要求新鲜反馈确认失能 |
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
| `MitLoopController` | `set_command()`、`set_commands(map<CAN_ID, MitCommand>)`、`set_all_commands()`、`commands()` |
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

默认使用带 CommandGuard 的启动流程；`finally` 尝试软件停止，失败继续向调用者抛出。

```python
import sys
from dm_openarm import Arm, MitCommand

arm = Arm.from_yaml("dm_openarm/config/arm.yaml")
# 示例增益需替换为工位标定值，顺序与所选电机一致。
gains = [MitCommand(kp=12.0, kd=0.6) for _ in arm.left.can_ids + arm.right.can_ids]
try:
    arm.enable_seeded(gains, hz=250.0)
    for state in arm.states():
        print(state.can_id, state.position)
finally:
    try:
        arm.disable()
    except Exception as exc:
        print(f"软件停止失败（{arm.safety_state}）：{exc}", file=sys.stderr)
        if arm.safety_state == "SHUTDOWN_UNKNOWN" or "shutdown unknown" in str(exc):
            print("失能未确认：使用独立硬件急停并人工检查，禁止自动恢复。", file=sys.stderr)
        raise
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

`arm.enable()` 只发送使能流程，不为新对象自动启用 CommandGuard。默认控制使用 `enable_seeded()`；
裸入口调用者负责新鲜反馈、目标基线、更新期限与故障处置。

`usb.device_index` 选择设备，`usb.serial` 核验实际所选句柄；不匹配、为空或不可读会在配置通道前拒绝连接。

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
arm.connect_passive()
arm.set_zero(0x01, persist=True)
arm.disable()
```

设置全部电机零位：

```python
arm.connect_passive()
arm.set_zero_all(persist=True)
arm.disable()
```

参数说明：

- `can_id`：目标电机 CAN ID，例如 `0x01`。
- `persist=True`：把当前位置设置为零位，并保存到电机 flash。
- `persist=False`：只做临时设零，不保证程序结束或断电后保持。

两种模式均要求已停止控制循环、目标轴失能，并验证新的 status-0 运动反馈。
SDK 主动查询并观察至少 300 ms、三个不同接收样本；样本至少间隔 20 ms，
速度绝对值不超过 0.05 rad/s、位置极差不超过 0.005 rad、反馈年龄不超过 0.2 s。
每个观察阶段最多等待 1 s，反馈中断重新累计静止窗口。这些静止阈值需实机工位验收。
设零结束保持失能，不再自动重新使能；失能期间机械支撑要求与标定保存相同。

单轴、整臂及 `Limb.set_zero_all()` 均使用同一原生批量实现：全部目标先检查，
每轴写入前再复查。预检查失败不写入；开始写入后失败会锁存坐标未知，
异常包含失败 CAN ID、已完成数量和 ID，不自动回滚或重发写入。

重要区别：

- `set_zero(...)` 是“把当前位置写成新的零位”。
- `start_mit_loop(..., home=True)` 才会运动到已保存零位 `q=0`，不是重新设置零位。

### 命令任务与停止状态

`command_timeout` 监护当前 `Arm` 的整张命令表，任一非空有效局部更新都会续期；
相同的有效目标也可续期。空字典和非法更新不续期，也不增加接受序号。
一个控制任务负责整张表；`left`/`right` 视图不构成独立超时域。
左右控制任务若独立运行，应分别创建 `Arm.from_yaml(path, side="left")` 和
`Arm.from_yaml(path, side="right")`，各自启动 guarded 循环，底层共享 USB 连接。

正常停止并确认失能后，`safety_state` 为 `DISABLED`；执行故障随后失能成功为 `FAULT`；
失能未确认为 `SHUTDOWN_UNKNOWN`。重复停止保留故障。停止清除可执行的旧目标和 seed，
保留最后循环发送快照用于诊断；重新 guarded 启动从新反馈取当前位置。
兼容裸入口在正常停止后保持无 CommandGuard 的语义，不继承旧保持目标；故障锁存后不能
通过裸入口绕过保护，须检查设备状态后显式使用 guarded 启动。

在线扫描脚本默认只调用被动连接和查询；`ONLINE` 仅表示收到查询应答，不判定驱动健康或机械状态。
`read_states.py` 与 `link_test.py` 为主动反馈诊断，会使能并发送零 MIT。
诊断脚本在关闭完成后才报告成功，读取与关闭同时失败时保留两个错误并返回非零退出码。

默认建议使用 `persist=True`，这样零位会在后续 Python/C++ 程序和断电重启后保持。
但不要频繁反复写 flash；只在机械零位确认正确后执行。

### 兼容裸入口：启动 MIT 后台循环

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
| DM4340P | `12.0` | `0.6` | `0.0` |
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
按 `Ctrl+C` 退出时，脚本会进入 `finally` 尝试软件停止和失能；失败继续抛出，不能视为独立硬件急停。

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

  dm_openarm::MitLoopController loop(arm);
  // 示例增益必须替换为工位标定值。
  std::vector<dm_openarm::MitCommand> gains(config.motors.size(), {12.0, 0.6, 0, 0, 0});
  loop.enable_seeded(gains, 250.0, 0.1, 0.2);

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


`mit(dict)`／`Limb.mit(dict)` 在 C++ 锁内原子合并到当前有效命令表；未更新轴保留 seed 或保持基线，
包括当前已衰减的前馈。`set_all_commands()` 仍完整替换命令表。输入验证失败不修改命令表或接受序号。

`hold()`／`hold_command()` 成功返回表示对应的完整保持批次已由发送接口提交；保持确认等待期间拒绝
运动更新。启动确认期限为 3 秒，保持确认期限为 `feedback_timeout`；底层调用仍须返回才能完成清理。
`HOLD` 是保护逻辑状态，机械稳定需要另行验证。Ctrl+C／`disable()` 不能替代独立硬件急停。
停止失败时保留异常；`SHUTDOWN_UNKNOWN`／`shutdown unknown` 表示失能未确认，必须人工处置，禁止自动恢复。

0.1.13 的源码提交、构建环境、验证记录和 wheel hash 见 [`../dist/BUILD_MANIFEST.json`](../dist/BUILD_MANIFEST.json)。

### Python 双臂自动零位标定

```python
from dm_openarm import Arm, CalibrationError

arm = Arm.from_yaml("dm_openarm/config/arm.yaml")
try:
    report = arm.calibrate_zero()  # CAN 0/1/5/6 kp=120，其余kp=60，kd=5
    # report = arm.calibrate_zero(kp=160, kd=5)
    # report = arm.calibrate_zero(gains={cid: (160, 5) for cid in range(10)})
except CalibrationError as exc:
    report = exc.report
    print(report.phase, report.error)
    # 反馈健康时失败会继续锁存保持；支撑好机械臂后再 stop/disable。
else:
    arm.disable()  # 成功后已失能，此处释放设备。
```

此接口随 0.1.12 的 CPython 3.13 wheel 发布；使用旧 wheel 的环境需要升级安装。
调用前应停止原有控制环并失能，且必须持有完整十轴配置；不要先调用 `enable()`。
参数支持统一 `kp/kd` 或完整十轴 `gains`，两种方式不能混用；`0<kp≤500`、`0<kd≤5`。
无参数默认 CAN 0/1/5/6 的 kp=120，其余六轴 kp=60；全轴 kd=5。

左右臂并行，各自按 J1→J5 执行。CAN ID 左 `0–4`、右 `5–9`，现有 YAML 向量顺序不变。
未扫描轴保持启动位置，完成轴保持测得的零位；整个扫描直接使用电机内部 MIT 位置控制，
`dq=0、tau=0`，不将位置 PD 换算为主机纯力矩控制。

| 关节 | 左范围 rad | 右范围 rad |
| --- | --- | --- |
| J1 / DM8009 | [-3.4907, 1.3963] | [-1.3963, 3.4907] |
| J2 / DM8009 | [-3.3161, 0.17453] | [-0.17453, 3.3161] |
| J3 / DM4340P | [-1.5708, 1.5708] | [-1.5708, 1.5708] |
| J4 / DM4340P | [0, 2.4435] | [0, 2.4435] |
| J5 / DM4340P | [-1.5708, 1.5708] | [-1.5708, 1.5708] |

默认250 Hz原生发送，命令超时0.5 s、反馈超时0.3 s。
启动使用已确认的工位型号及编码范围，不再逐次查询 P_MAX/VMAX/TMAX；更换电机或修改固件配置后须重新核对配置。
使能前和失能保存期间主动请求新的角度反馈，要求十轴反馈有效且处于失能状态。
此快速流程要求现有零位可信，理想范围仅用于提前制动，最终端点仍实际测量。
理想范围仅用于参考轨迹提前减速，不再按实测与理想范围的差值拒绝标定；
端点仍须由停滞和负载变化确认并复测。快速接近仍依赖已有零位提供大致位置参考。

快移参考速度上限2 rad/s、参考加速度上限6 rad/s²；接近端点前，
按制动距离、0.15 s反馈延迟预算和0.2 rad端点余量提前减速，触限复测速度0.2 rad/s。
这些是参考轨迹约束，不保证实际匀速，也不增加单帧超速即失能的保护。
快移时跟随预算为 `接触预算 + (kd/kp + 0.15) × |参考速度|` rad；
触限低速段预算DM8009为0.05 rad、DM4340P为0.025 rad，默认静止时的位置项分别约6 / 1.5 N·m。
减速时若原参考已超过缩小后的预算，会逐渐卸除多余顶压；这些不是总力矩硬限幅。

端点要求0.25 s窗口内位置极差≤0.002 rad、目标继续推进至少0.003 rad，
同时推进方向的 `torque` 反馈在最近1.2 s内升高，并维持升高后的负载至少0.1 s；最低增量DM8009为0.15 N·m、
DM4340P为0.10 N·m。这里使用力矩反馈，没有独立实测电流字段。
仅处理新增反馈，并以至少20 ms的接收时间间隔过滤同批缓存样本；至少三个观察样本才判定。
达到领先预算后两秒仍未确认接触则失败，不继续顶压。

每端退让0.03 rad，以0.2 rad/s复测，采用复测位置作为端点，重复误差仅记录、不作为失败条件；实际跨度与理想跨度可以不同，仅记录差值，不设跨度一致性容差。
保留电机协议位置范围、反馈连续性与超时保护，不把理想跨度作为实测行程上限。
每轴耗时目标12秒，完整流程含启动和保存的耗时目标120秒；超过目标仅在逐轴及总报告中设置
`time_target_exceeded=True`，不因此停止运动或阻止保存。反馈、接触、位置稳定及通信保护仍然有效。
零位使用线性插值：

```text
fraction = (0 - ideal_lower) / (ideal_upper - ideal_lower)
zero_raw = measured_lower + fraction * (measured_upper - measured_lower)
```

J4理想下限为0，上式自然得到实测下限。只用此插值找到要写零的位置，
保存后仍直接使用编码器角度，不对后续位置、速度、刚度或力矩做比例缩放。
非有限端点、上下限颠倒或零跨度仍拒绝标定。

退让以实际离开挡块至少0.015 rad且反馈速度稳定为完成条件，不要求负载下0.005 rad的跟随误差。
逐轴回零采用1°误差，并在必要时按每两秒增加初值25%的规则提升kp。
十轴完成后，在各自零位1°以内连续保持一秒。未达标轴每两秒将kp增加初值的25%，
上限为初值两倍且不超过500，kd不变；十秒仍未收敛则失败，不写零。
之后停止控制环、确认全臂失能，逐轴读取位置并观察至少一秒，误差和漂移均须小于1°。
每轴置零前核对当前坐标；十轴置零和Flash保存命令均已发送后，统一读回全轴坐标，全部接近零才标记为保存完成，不等待保存回执。
最终角度回读使用统一1秒窗口，每50毫秒查询尚未收到新角度的轴，已收到的角度保留到窗口结束。
窗口内收齐十轴且每轴绝对角度小于1°就立即成功；任一轴角度越界立即失败，窗口结束仍未收齐则失败。
仅重试角度查询，不重发置零或Flash保存命令。
成功后清除旧坐标命令并保持失能。

`CalibrationReport` 有 `success`、`phase`、`error`、`joints`；每轴包含CAN ID、上下限、
重复误差、行程误差、原始零位、最终误差、最终kp/kd、阶段及保存状态。
`save_state` 为 `NOT_ATTEMPTED` / `UNKNOWN` / `SAVED`，后者表示保存命令已发送且整批位置读回通过。
集中保存并非十轴原子操作，失败不自动重试或回滚；部分写入后阻止继续控制，需要人工核查。
协议无请求nonce，不能严格排除延迟的旧同类回复，断电保持仍须实机验收。

其他线程可调用 `arm.calibration_status()` 取得独立快照，或 `arm.cancel_zero_calibration()`
请求取消。Ctrl+C也会转为携带部分报告的 `CalibrationError`。
反馈健康且尚未尝试任何写零时，可重新调用 `calibrate_zero()` 从锁存保持重试，不发送失能/使能帧。扫描失败或取消且反馈健康时，
保留原生锁存保持及总线所有权；保持依赖进程继续运行。反馈故障沿用原生失能清理。
保存开始后取消只停止后续轴，已写入的坐标不会撤回。

2026-10-15完成部分实机验证：左右J1–J4完成扫描、复测和回零，单轴约9–12秒；
J5尚未通过，未执行实机写零或Flash保存。左J5的停滞未被现场确认为机械限位，
有限双向探测仍只有少量编码器位移，已按现场要求失能并等待检查。
停滞和负载升高本身不能证明接触的是机械挡块；当日完整流程、集中保存及断电保持尚未验收。
实测记录见 [hardware_verification_20261015.json](hardware_verification_20261015.json)。
现场需确保扫描路径、机械挡块、失能支撑和独立急停可用。

2026-10-16实机十轴扫描及置零/保存命令已完成，独立回读十轴均约−0.000191 rad且失能。
该轮API因单次查询缺反馈报错；本版随后增加1秒窗口内收齐反馈的逻辑，并通过离线验证，尚未用最终窗口版本重新驱动实机。
断电保持尚未验收，记录见 [hardware_verification_20261016.json](hardware_verification_20261016.json)。
