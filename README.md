# DM OpenArm

`DM OpenArm` 是面向达妙电机机械臂的控制库（C++17 + Python）。  
底层通信栈与工位实测可用的 `resources/u2canfd` **对齐**：`libdm_device.so`（dmcan API）+ **经典 CAN 1 Mbps**。

当前封装 **MIT 模式**，并支持 **重力前馈**（`tau_ff`）：

- C++：打开设备、发 MIT 帧、读反馈、1 kHz 后台循环（可叠加 \(g(q)\)）
- Python：`Arm` 高层 API（使能、状态、MIT、设零、重力开关/比例）
- 配置：`dm_openarm/config/arm_5dof.yaml`（含工位辨识后的 gravity 段）

更细的包内说明见 [`dm_openarm/README.md`](dm_openarm/README.md)。  
工位硬件实测记录见 [`resources/u2canfd/HARDWARE_CONFIG.md`](resources/u2canfd/HARDWARE_CONFIG.md)。  
重力原始拟合结果备份见仓库根目录 [`gravity_identified.yaml`](gravity_identified.yaml)。

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

gravity:
  enabled: true
  scale: 0.9          # 默认略欠补偿，软刚度下更稳
  use_measured_q: true
  coupled:            # 仅支持耦合模型 τ = W·φ(q)
    basis: [one, sin_q0, ..., sin_q3_q4, cos_q3_q4]
    weights:          # 与 motors[] 同序，每行长度 = basis
      - [...]
```

## 重力补偿（工位现状）

### 模型

MIT 下发力矩为：

\[
\tau_{\text{sent}} = k_p(q_{\text{des}}-q) + k_d(\dot q_{\text{des}}-\dot q) + \tau_{\text{cmd}} + s\cdot\hat\tau_g(q)
\]

**仅保留耦合模型**（解耦 `amp/phase/bias` 已移除）：

\[
\hat\tau_{g,i}(\mathbf{q})=\sum_k w_{ik}\,\phi_k(\mathbf{q})
\]

默认基函数：`one`, `sin/cos q0..q4`, `sin/cos(q3+q4)`（肩肘耦合）。

- \(s\) = YAML / API 中的 `scale`（默认 **0.9** 欠补偿）  
- loop 内：`tau_sent = cmd.tau + scale * g(q)`  
- 软保持时建议锁定姿态后用 **目标角** \(q_{\text{des}}\) 算 \(g\)（避免下沉正反馈）；跟手移动时可用测量角

### 工位辨识结果（2026-07-21，耦合已验收并写入配置）

空载、当前机械零位；**0x01–0x03 接近 0，0x04/0x05 主承力**。参数见 `arm_5dof.yaml` / `gravity_coupled_identified.yaml`。

配置中默认 `gravity.enabled: true`，`scale: **0.9**`。

### 验收结论（当前）

| 项 | 状态 |
| --- | --- |
| 通信 / 使能 / 读状态 | 可用（dmcan + 经典 CAN 1M） |
| 耦合重力前馈 | **已验收**（肩肘明显托住，优于旧解耦） |
| 调参方向 | 过冲 → 降 `scale`；慢沉 → 略升 `scale` 或略加 `kp`/`kd` |
| 末端负载模型 | **不做**（工具未接入前） |
| 轨迹跟踪 | 后续分支 |

### 辨识流程（换零位或换空载结构后重做）

```bash
source .venv/bin/activate
pip install -e ./dm_openarm

# 1) 机械摆到期望零位后设零（写 flash）
python -c "
from dm_openarm import Arm
arm = Arm.from_yaml('dm_openarm/config/arm_5dof.yaml')
arm.enable()
arm.set_zero_all(persist=True)
arm.disable()
"

# 2) 交互耦合辨识（多轴组合姿态 ≥20 点）
python python_script/identify_gravity.py
# 输出 gravity_coupled_identified.yaml → 合并进 arm_5dof.yaml 的 gravity:
```

辨识脚本命令：

| 命令 | 含义 |
| --- | --- |
| `u` | 解锁跟手（**双手托住**再掰） |
| `l` | 锁定当前角（可松手检查） |
| `Enter` | 在**已锁定目标**上加硬采样（不改目标角，避免突然松弛） |
| `f` | 拟合并写入 `gravity_coupled_identified.yaml` |
| `q` | 放弃 |

要点：

- **强调肩+肘组合姿态**，避免大量「只动一轴」  
- 每点**停稳**再锁、再采；工作空间多点覆盖  
- 拟合后把 `gravity:` 段合并进 `arm_5dof.yaml`

### 软保持试跑

```bash
# 默认 scale=0.9
python python_script/hold_with_gravity.py

# 仍下沉
python python_script/hold_with_gravity.py --scale 1.0

# 个别点过冲 / 顶过头
python python_script/hold_with_gravity.py --scale 0.8
```

用法：启动后锁定当前姿态 → **托着**移到新姿态 → 停约 0.4s 见 `[锁定]` → 再松手。  
Ctrl+C 退出。

### 末端加负载后是否要重标定？

**要。** 当前 \(W\) 对应**空载**质量分布。末端加工具/工件后肩、肘变化最大。

| 情况 | 建议 |
| --- | --- |
| 固定一种负载 | **带该负载**再跑 `identify_gravity.py`，更新 YAML |
| 几种已知负载 | 多套 `gravity` 配置或按工具切换 |
| 负载常变/未知 | 需在线估计或力传感；单次空载标定不够 |
| 临时凑合 | 可略调 `scale`，个别姿态仍会差 |

### 软刚度与补偿误差

软 `kp` 时闭环几乎不“硬顶”，残差 \(\tau_{\text{true}}-s\hat\tau_g\) 会直接变成漂移/过冲：

- **欠补偿** → 慢沉（更安全）  
- **过补偿** → 往上顶、过冲（危险）  

因此默认 `scale=0.9`。

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

# 2) 读状态（Ctrl+C 退出）
python python_script/read_states.py

# 3) 使能/失能
python python_script/enable_disable.py

# 4) 重力软保持（需已写入 gravity 参数；Ctrl+C 退出）
python python_script/hold_with_gravity.py

# 5) 小范围关节轨迹（默认 home=False，需确认后才动）
python python_script/move_joints_demo.py
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
| `python_script/move_joints_demo.py` | 五次多项式关节轨迹 + 耦合重力（小角度） |

## 双臂 left / right

同一 USB-CAN 上左臂 `0x01–0x05`、右臂 `0x21–0x25`。设备共享 `enable` / `start_mit_loop`，控制按侧：

```python
from dm_openarm import Arm

arm = Arm.from_yaml("dm_openarm/config/arm_dual_10dof.yaml")
arm.enable()
# 先 seed 当前姿态再开环，避免 home 回零
for s in arm.states():
    arm.mit(s.can_id, kp=12, kd=0.6, q=s.position)
arm.start_mit_loop(home=False)

arm.left.set_gravity_enabled(True)
arm.right.set_gravity_enabled(False)  # 右臂未辨识重力前保持关

arm.left.move_joints([...], duration=2.5)
arm.right.move_joints([...], duration=2.5)
arm.disable()
```

扫描：`python python_script/scan_motors_online.py`  
配置：`dm_openarm/config/arm_dual_10dof.yaml`（旧 `arm_5dof.yaml` 仍为单左臂）。

## 关节轨迹跟踪

软 MIT 下**完成条件是规划时长 \(T\)**，不是「位置误差进容差」。残差重力 + 低 \(k_p\) 会产生稳态滞后，属预期；`MoveResult` 只报告 `max_abs_err`，不抛「未到位」。

```python
from dm_openarm import Arm

arm = Arm.from_yaml("dm_openarm/config/arm_5dof.yaml")
arm.enable()
arm.start_mit_loop(hz=1000.0, home=False)  # 勿默认回零
arm.set_gravity_enabled(True)

q = [s.position for s in arm.states()]
q[3] += 0.15   # elbow
q[4] -= 0.15   # shoulder
result = arm.move_joints(q, duration=4.0, rate_hz=100.0)
print(result.max_abs_err, result.err)

arm.disable()
```

- 规划：同步多关节 rest-to-rest 五次多项式 \((q_d,\dot q_d)\)，默认约 100 Hz 写入 MIT loop  
- 重力：C++ 每 1 ms 仍 `tau += scale * g(q)`（耦合模型）  
- **无**末端负载模型；**无**硬位置门槛 API（不做 `wait_until_reached` 主路径）

## Python API：`Arm`

| API | 说明 |
| --- | --- |
| `Arm.from_yaml(path)` | 加载配置，不连硬件 |
| `enable()` | 打开设备并使能电机 |
| `disable()` | 停 loop + 失能 + 释放设备 |
| `states()` | 各轴反馈列表 |
| `start_mit_loop(hz=1000, zero_timeout=5, home=True)` | 后台 1 kHz 循环；`home=False` 时不回零 |
| `stop_mit_loop()` | 停止后台循环 |
| `mit(can_id, kp=..., kd=..., q=..., dq=..., tau=...)` | 更新单轴 MIT 目标 |
| `mit({can_id: MitCommand(...), ...})` | 批量更新（未列出的轴保持原命令） |
| `move_joints(q_goal, duration=None, ...)` | 五次多项式同步运动 → `MoveResult`（时间盒完成） |
| `move_to` / `hold_at` | 瞬时写单轴目标，不插补、不阻塞 |
| `set_gravity_enabled(bool)` | 打开/关闭 loop 内重力前馈 |
| `set_gravity_scale(float)` | 重力前馈比例 |
| `gravity_torques(q=None)` | 计算 \(g(q)\)（默认用当前测量角） |
| `commands()` | 当前命令表 |
| `mit_loop_running` | loop 是否在跑 |
| `set_zero(can_id, persist=True)` | 当前位置写为零位（`persist` 写 flash） |
| `set_zero_all(persist=True)` | 全部轴设零 |

数据类型：`MitCommand`、`MotorState`、`MoveResult`、`ArmConfig`、`MotorConfig`、`MotorModel`、`ControlMode`。  
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
