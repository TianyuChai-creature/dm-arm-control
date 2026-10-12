# dm_openarm SDK 使用指南

当前发行版本：**0.1.10**。带原生执行保护的用法：

```python
arm.enable_seeded(gains, hz=250, command_timeout=0.1, feedback_timeout=0.2)
# 返回前确认首次完整保持批次提交成功；机械稳定另行判断。
# ACTIVE 命令必须定期提交，mit(dict) 在原生锁内原子合并局部命令。
arm.mit(commands)
arm.hold_command()             # 正常静止：保持最后实际执行的命令，前馈不衰减
arm.hold()                     # 人工暂停：实测位置保持，原前馈一秒内撤去
arm.hold(reset_fault=True)     # 显式解除超时锁存，仍保持
```

`gains` 为 SDK 电机顺序的 `MitCommand` 列表，只取经验证的保持 kp/kd；kp 必须为正。
启动反馈超时、无效反馈或驱动错误锁存 FAULT；应用命令超时且反馈正常锁存 FAULT_HOLD。
普通新命令无法解除锁存。无抱闸机器在通信/供电/驱动失效时不能保证继续支撑负载。

兼容入口 `enable()+start_mit_loop()` 不会为新对象自动启用 CommandGuard；调用者负责反馈验证、启动基线、命令更新期限与故障处置。

## 安装

当前 wheel 适用于 **CPython 3.12 / Linux x86_64**。其他 Python 版本或平台需要重新构建 wheel。

安装当前目录下的 wheel：

```bash
python -m pip install dm_openarm-0.1.10-cp312-cp312-linux_x86_64.whl
```

guarded 启动默认 **250 Hz**，兼容入口默认 1000 Hz，最高支持 1000 Hz。
频率不从 YAML 读取；运行时通过 `enable_seeded(..., hz=...)` 显式指定。

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

`start_mit_loop(home=True)` 会让全部电机运动到已保存零位，只有确认安全时才使用。

## 主要 API

### `Arm`

- `Arm.from_yaml(path, side=None)`：加载配置，不连接设备；指定 `left`/`right` 时仅控制该侧。
- `enable()` / `disable()`：连接并使能 / 停止控制循环、失能并断开。
- `enable_seeded(gains, hz=250, command_timeout=0.1, feedback_timeout=0.2)`：原生使能、取反馈并 seed。
- `states()`：读取全总线反馈。
- `mit(can_id, kp, kd, q, dq, tau)`：更新单轴 MIT 命令。
- `mit({can_id: MitCommand(...)})`：批量更新命令，原生锁内原子合并局部命令。
- `start_mit_loop(hz=1000.0, home=False)`：启动控制循环。
- `stop_mit_loop()`：停止控制循环、发送零 MIT 与失能帧，要求新鲜反馈确认失能；失败抛错。
- `hold_command()`：保持最后实际执行的静止命令，前馈不衰减。
- `hold(reset_fault=False)`：实测位置保持；`reset_fault=True` 解除超时锁存。
- `safety_state` / `fault` / `accepted_sequence` / `sent_sequence` / `sent_commands()`：原生保护只读接口。
- `set_zero(can_id, persist=True)` / `set_zero_all(...)`：设置零位。
- `deadline_misses`：读取控制循环超期次数。
- `timing_stats()`：读取 1 秒滑动窗口的整臂发送频率和逐电机接收频率。
- `left` / `right`：访问左右侧 `Limb`。

### `Limb`

- `can_ids`、`names`、`n_joints`：本侧电机元数据。
- `states()`、`commands()`：读取本侧状态和命令。
- `mit(...)`：发送本侧 MIT 命令，会拒绝另一侧的 CAN ID。
- `set_zero(...)` / `set_zero_all(...)`：设置本侧零位。

### `MotorState`

主要字段：`can_id`、`mst_id`、`position`、`velocity`、`torque`、
`feedback_hz`、`last_rx_age_s`、`rx_sequence`、`error_code`。

使用 `state.feedback_fresh(max_age_s=0.1)` 判断反馈是否在指定时间内更新。

## 配置

`usb.device_index` 选择设备，`usb.serial` 与实际所选句柄的 SN 必须一致。
SN 不匹配、为空或不可读会在配置通道／发送电机命令之前拒绝连接，不自动选择其他索引。

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

`hold()`、`hold_command()` 是同步确认接口；等待期间拒绝运动更新。保持确认期限取 `feedback_timeout`，
启动总期限为 3 秒。底层发送调用必须返回才能完成异常清理，这些期限不提供硬实时急停保证。
`HOLD` 表示保护逻辑状态；同步返回表示所有轴的保持帧已被发送接口提交，不能据此断言机械稳定。
ACTIVE 命令仍须在 `command_timeout` 内定期提交；新命令从当前有效保持基线恢复。

Ctrl+C／`disable()` 只触发软件停止与失能。失能失败或未确认时，`SHUTDOWN_UNKNOWN`／
`shutdown unknown` 表示无法证明电机已经失能：报告并保留错误，使用独立硬件处置、人工检查，禁止自动恢复。

0.1.10 修正：有效命令基线、原生原子局部更新、保持发送确认与 USB SN 校验；默认文档改为 guarded 启动。

0.1.4 修正：反馈状态 `0x1`（已使能）在后端转换为 `error_code=0`，真实报警和未知非零码保持不变。

0.1.5 修正：按回复的完整电机 ID、长度和操作码识别参数读写/保存回复；延迟回复不再成为位置反馈或启动保持目标。

0.1.6 新增 `model: DM4340P`，按工位确认的 PMAX=12.5 rad、VMAX=20 rad/s、TMAX=28 Nm 编码命令与解码反馈。新型号通过 `MotorModel.DM4340P` 暴露给 Python。

0.1.7 新增 `Arm.hold_command()`：在健康位置控制下保持最后实际执行的 q/kp/kd/dq/tau，前馈不衰减。

0.1.8 新增 `Arm.timing_stats()`：使用固定 1 秒滑动窗口报告整臂控制周期频率和逐电机有效反馈频率。


## 构建与来源记录

版本同时定义于 Python 包、项目元数据和 CMake，发布前必须一致。先提交源码与文档，
取得干净源码提交，再用 `git archive` 导出该提交构建，避免混入旧扩展或工作区临时文件。
0.1.10 的实际环境、完整命令、源码提交、验证结果及 SHA-256 见 [BUILD_MANIFEST.json](BUILD_MANIFEST.json)。

```bash
# 激活干净 CPython 3.12 构建环境，按 manifest 安装相同版本的构建工具。
git archive HEAD | tar -x -C /tmp/dm-openarm-source
cd /tmp/dm-openarm-source/dm_openarm
python -m build --wheel --no-isolation --outdir /tmp/dm-openarm-artifacts \
  -Cbuild-dir=/tmp/dm-openarm-wheel-0.1.10
```

构建目录须为空并事先创建导出目录。记录编译器、系统库、Python 构建依赖、SDK 输入文件与
包内文件 hash、构建起止时间、wheel hash。先记录源码提交，再单独提交 wheel 与 manifest，
产物提交不作为源码来源提交。离线测试需启用 C++ 断言；安装验证在独立环境运行，校验导入路径
属于该环境，不从源码目录导入包。构建本身不等于完成实机验收。

历史 0.1.9 原样保留；其元数据与 hash 可核验，源码提交和构建环境尚未核实，不推定与某份源码行为一致。
