# DM OpenArm

`DM OpenArm` 是面向达妙电机机械臂的控制库（C++17 + Python）。  
底层通信栈与工位实测可用的 `resources/u2canfd` **对齐**：`libdm_device.so`（dmcan API）+ **经典 CAN 1 Mbps**。

当前只封装 **MIT 模式**：

- C++：打开设备、发 MIT 帧、读反馈、1 kHz 后台循环
- Python：`Arm` 高层 API（使能、状态、MIT 目标、设零）
- 配置：`dm_openarm/config/arm_5dof.yaml`

更细的包内说明见 [`dm_openarm/README.md`](dm_openarm/README.md)。  
工位硬件实测记录见 [`resources/u2canfd/HARDWARE_CONFIG.md`](resources/u2canfd/HARDWARE_CONFIG.md)。

## 目录结构

| 路径 | 说明 |
| --- | --- |
| `dm_openarm/` | 库源码、Python 绑定、配置、测试、C++ 示例 |
| `dm_openarm/config/arm_5dof.yaml` | 默认 5 轴配置 |
| `dm_openarm/third_party/damiao_sdk/lib/libdm_device.so` | 达妙 dmcan 设备库（与 u2canfd 相同） |
| `python_script/` | 根目录可运行 Python 脚本 |
| `resources/u2canfd/` | 实测可用的 Python 参考例程与硬件记录 |
| `resources/DMmotor/` | 上游 C++ 旧例程参考（`libu2canfd.a`，本库已不作为运行路径） |

## 硬件与通信（当前工位）

| 项目 | 值 |
| --- | --- |
| 设备驱动 | `libdm_device.so`（dmcan），**不是**旧版 `libu2canfd.a` |
| 链路模式 | **经典 CAN**（YAML：`canfd: false`，`brs: false`） |
| USB 适配器 SN | `52A871B1AA5EF4E239371A5083463F26` |
| 标称波特率 | `1000000`（1 Mbps） |
| 数据域波特率 | 经典 CAN 下不使用（配置里写 1M 即可） |
| 控制模式 | MIT |
| 默认控制频率 | 1000 Hz |

> 工位实测：CAN FD（含数据域 1M/5M）无回帧。不要按旧文档默认的 5M CANFD 配置。

默认电机表：

| CAN ID | MST ID | 型号 | YAML 名称 | 位置 |
| --- | --- | --- | --- | --- |
| `0x01` | `0x11` | DM4310 | `end_effector` | 末端 |
| `0x02` | `0x12` | DM4310 | `wrist_2` | 末端上数第二 |
| `0x03` | `0x13` | DM4310 | `wrist_3` | 末端上数第三 |
| `0x04` | `0x14` | DM8009 | `elbow` | 肘部 |
| `0x05` | `0x15` | DM8009 | `shoulder` | 肩部 |

换适配器 / 改波特率 / 改 ID：编辑 `dm_openarm/config/arm_5dof.yaml` 即可，无需改协议代码。  
改 C++/绑定代码后需重新 `pip install -e ./dm_openarm` 或重新编译。

### YAML 关键字段示例

```yaml
usb:
  serial: "52A871B1AA5EF4E239371A5083463F26"
  nominal_baud: 1000000
  data_baud: 1000000
  canfd: false
  brs: false
  device_index: 0
```

## 单位约定

| 字段 | 含义 | 单位 |
| --- | --- | --- |
| `q` | 目标**绝对**位置（相对电机零点） | rad |
| `dq` | 目标速度 | rad/s |
| `tau` | 前馈力矩 | N·m |
| `kp` | 位置增益 | N·m/rad |
| `kd` | 速度阻尼 | N·m/(rad/s) |
| `position` / `velocity` / `torque` | 反馈 | rad / rad/s / N·m |
| `feedback_interval_s` | 反馈间隔 | s |

`q=0.8` 表示运动到绝对位置 `0.8 rad`，**不是**相对当前位置 +0.8。

## 系统依赖与安装

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config \
  libusb-1.0-0-dev libudev-dev libyaml-cpp-dev

# udev（USB 权限，通常只需设置一次）
# SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6877", MODE="0666"
```

Python（在仓库根目录）：

```bash
cd /home/creature/Desktop/dm-arm-control
python3 -m venv .venv
source .venv/bin/activate
pip install -e ./dm_openarm
```

**不要**在仓库根目录执行 `pip install .`（根目录没有 `pyproject.toml`）。

## 快速验证（推荐顺序）

```bash
cd /home/creature/Desktop/dm-arm-control
source .venv/bin/activate

# 1) 联通：MIT 全 0，不回零
python python_script/link_test.py

# 2) 读状态
python python_script/read_states.py

# 3) 使能/失能
python python_script/enable_disable.py
```

C++ 联通检查：

```bash
cd dm_openarm/build   # 若无则 cmake .. && make
./check_comm ../config/arm_5dof.yaml
```

关闭设备时，底层可能打印若干行 `libusb_transfer_cancelled or error`，  
属 `libdm_device` 收尾时的已知现象，**一般可忽略**（进程仍正常退出）。

## Python 使用

```python
from dm_openarm import Arm, MitCommand

arm = Arm.from_yaml("dm_openarm/config/arm_5dof.yaml")

try:
    arm.enable()
    arm.start_mit_loop(hz=1000.0)  # 会尝试回零 q=0，注意安全

    arm.mit(0x01, kp=12.0, kd=0.6, q=0.4, dq=0.0, tau=0.0)

    for s in arm.states():
        print(f"0x{s.can_id:02X} pos={s.position:.4f} vel={s.velocity:.4f}")

finally:
    arm.stop_mit_loop()
    arm.disable()
```

### 示例脚本

| 脚本 | 作用 |
| --- | --- |
| `python_script/link_test.py` | 联通测试（MIT 全 0，不回零） |
| `python_script/read_states.py` | 持续打印各轴状态（Ctrl+C 退出） |
| `python_script/enable_disable.py` | 使能后等待回车再失能 |
| `python_script/mit_control_one_motor.py` | 启动 MIT loop 并驱动单轴（会运动） |
| `python_script/identify_gravity.py` | 交互采样姿态，拟合 gravity YAML |
| `python_script/hold_with_gravity.py` | 重力前馈 + 软刚度保持当前姿态 |

## Python API：`Arm`

| API | 说明 |
| --- | --- |
| `Arm.from_yaml(path)` | 加载配置，不连硬件 |
| `enable()` | 打开设备并使能电机 |
| `disable()` | 停 loop + 失能 + 释放设备 |
| `states()` | 各轴反馈列表 |
| `start_mit_loop(hz=1000, zero_timeout=5)` | 后台 1 kHz 循环，并先命令回零 |
| `stop_mit_loop()` | 停止后台循环 |
| `mit(can_id, kp=..., kd=..., q=..., dq=..., tau=...)` | 更新单轴 MIT 目标 |
| `mit({can_id: MitCommand(...), ...})` | 批量更新（未列出的轴保持原命令） |
| `commands()` | 当前命令表 |
| `mit_loop_running` | loop 是否在跑 |
| `set_zero(can_id, persist=True)` | 当前位置写为零位（`persist` 写 flash） |
| `set_zero_all(persist=True)` | 全部轴设零 |

数据类型：`MitCommand`、`MotorState`、`ArmConfig`、`MotorConfig`、`MotorModel`、`ControlMode`。  
`ArmConfig` 现含 `canfd` / `brs` / `device_index`。

底层绑定（一般不必用）：`dm_openarm._core.DmArm`、`MitLoopController`、`send_zero_mit_all()` 等。

## C++ 构建

```bash
cd dm_openarm
mkdir -p build && cd build
cmake ..
make -j$(nproc)
ctest --output-on-failure
```

示例：

```bash
./check_comm ../config/arm_5dof.yaml
./enable_disable ../config/arm_5dof.yaml
./hold_position ../config/arm_5dof.yaml
./set_zero_position ../config/arm_5dof.yaml all
./arm_mit_control ../config/arm_5dof.yaml
```

头文件：`dm_arm.hpp`、`mit_loop_controller.hpp`、`yaml_loader.hpp`、`types.hpp`、`config.hpp`。

## 安全注意

- `start_mit_loop()` 会驱动各轴尝试回到已保存零位 `q=0`，周围勿有干涉。
- `mit()` **无自动斜坡**；大角度 + 高 `kp` 可能猛动。
- `set_zero(persist=True)` 写 flash，勿频繁调用。
- 运动脚本务必 `try/finally` 中 `stop_mit_loop()` + `disable()`。
- 初次建议单轴、小角度、低增益、`tau=0`。

## 与 u2canfd 的关系

| 项 | `resources/u2canfd` | `dm_openarm`（当前） |
| --- | --- | --- |
| 设备库 | `dlls/libdm_device.so` | `third_party/.../libdm_device.so` |
| API | Python dmcan-sdk | C++ dmcan + `Arm` / `DmArm` |
| 总线 | classic CAN 1M | 相同（YAML 默认） |
| 用途 | 实测联通 / 扫描例程 | 5 轴控制库与业务脚本 |

旧栈 `libu2canfd.a`（`usb_class`）仅作历史参考，**不再作为本库运行路径**。
