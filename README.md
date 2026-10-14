# DM OpenArm

达妙电机**双臂**控制库（C++17 + Python）。  
通信：`libdm_device.so`（dmcan）+ **经典 CAN 1 Mbps**（与 `resources/u2canfd` 工位实测一致）。

| 能力 | 说明 |
|------|------|
| MIT 力矩模式 | 1 kHz 后台循环 |
| 双臂 | 左 `0x00–0x04` / 右 `0x05–0x09`，同一总线 |
| 前馈力矩 | 由上层应用计算后写入 `MitCommand.tau` |
| 收发频率诊断 | 1 秒滑动窗口统计整臂 TX 和逐电机 RX |

---

## 快速开始

```bash
# 依赖（Debian/Ubuntu）
sudo apt install -y build-essential cmake pkg-config libyaml-cpp-dev libusb-1.0-0-dev
python3 -m venv .venv && source .venv/bin/activate
pip install -e ./dm_openarm

# 扫描 10 轴是否在线
python python_script/scan_motors_online.py
```

唯一站位配置：`dm_openarm/config/arm.yaml`（**必须**含 `left:` + `right:`，无根级 `motors:`）。

改 C++ 后需重新 `pip install -e ./dm_openarm`。

构建发布 wheel：

```bash
python -m build --wheel --no-isolation --outdir dist dm_openarm
```

当前版本 **0.1.11**。wheel、构建记录和安装说明位于 `dist/`；可追溯构建步骤见
[`dist/USAGE_GUIDE.md`](dist/USAGE_GUIDE.md#构建与来源记录)。guarded 启动默认 **250 Hz**，
兼容裸入口默认 1000 Hz；最高支持 1000 Hz，频率均由运行时参数指定。

---

## 硬件与 ID

| 项目 | 值 |
|------|-----|
| 链路 | 经典 CAN 1M（`canfd: false`, `brs: false`） |
| USB SN | `52A871B1AA5EF4E239371A5083463F26`（改适配器改 YAML） |
| 控制 | MIT @ 1 kHz |

| 侧 | CAN | MST | 顺序 | 型号 |
|----|-----|-----|------|------|
| **left** | 0x00…0x04 | 0x10…0x14 | 配置向量：腕远端 → 肩 | 腕 DM4310 ×3，肘/肩 DM8009 |
| **right** | 0x05…0x09 | 0x15…0x19 | 同上 | 同上 |

---

## 架构（怎么用 API）

```
Python Arm
├── 设备级：enable_seeded / hold / disable / states / mit / timing_stats
├── arm.left  (Limb)  → mit / set_zero …
└── arm.right (Limb)  → 同上
        │
        ▼
C++ MitLoopController @ 250 Hz guarded（最高 1 kHz）
  tau_sent = tau_cmd
```

- **设备级**：开设备、共享 1 kHz 环（两侧一起进 loop）。  
- **侧级 `Limb`**：按侧限制 CAN ID 并提供基础 MIT/设零操作。
- 默认使用 `enable_seeded()` 从新鲜反馈保持当前位置，不自动回零。

---

## Python API

### 设备级 `Arm`

| 方法 | 作用 |
|------|------|
| `Arm.from_yaml(path, side=None)` | 加载双臂或指定单臂配置，不连设备 |
| `enable()` / `disable()` | 连接使能 / 停 loop + 失能 |
| `enable_seeded(gains, hz=250, ...)` | guarded 启动，首次完整保持批次提交成功后返回 |
| `start_mit_loop(hz=1000, home=False)` | 兼容入口；新对象不会自动启用 CommandGuard |
| `stop_mit_loop()` | 停环 |
| `hold_command()` | 保持最后实际执行的命令，前馈不衰减 |
| `hold(reset_fault=False)` | 实测位置保持；`reset_fault=True` 解除超时锁存 |
| `states()` | 全总线反馈（左后右） |
| `safety_state` / `fault` | 原生保护状态与故障原因 |
| `sent_commands()` | 实际下发的 MIT 快照 |
| `deadline_misses` | MIT 循环累计超期次数 |
| `timing_stats()` | 1 秒窗口的控制下发频率和逐电机反馈频率 |
| `mit(can_id, kp=, kd=, q=, dq=, tau=)` | 写任意轴 MIT（底层） |
| `mit({can_id: MitCommand(...), ...})` | 锁内原子合并局部命令，保留未更新轴的有效基线 |
| `set_zero` / `set_zero_all` | 当前位置写零（可 `persist` 写 flash） |
| `left` / `right` | `Limb` 对象 |

`MotorState` 还提供 `feedback_hz`、`last_rx_age_s`、`rx_sequence`、`error_code` 和
`feedback_fresh(max_age_s=0.1)`，用于判断反馈是否新鲜。

```python
stats = arm.timing_stats()
print(stats.target_tx_hz, stats.actual_tx_hz, stats.deadline_misses)
for motor in stats.motors:
    print(motor.can_id, motor.rx_hz, motor.last_rx_age_s, motor.rx_frames)
```

### 侧级 `Limb`（`arm.left` / `arm.right`）

| 方法 | 作用 |
|------|------|
| `can_ids` / `names` / `n_joints` / `states()` | 本侧元数据与反馈 |
| `mit` | 本侧 MIT（拒绝异侧 can_id） |
| `commands()` | 本侧当前 MIT 命令 |
| `set_zero` / `set_zero_all` | 本侧设零 |

---

## 调用示例

### 1. 扫描在线

```bash
python python_script/scan_motors_online.py
```

### 2. 最小控制（读状态 + 保持）

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

### 3. 当前位置设为零位（写 flash）

```python
arm = Arm.from_yaml("dm_openarm/config/arm.yaml")
arm.connect_passive()  # 需先取得新鲜 status-0、低速反馈；不满足条件会拒绝设零。
arm.set_zero_all(persist=True)   # 左右全部 10 轴
# 或 arm.left.set_zero_all() / arm.right.set_zero_all()
arm.disable()
```

> 动力学模型和辨识结果由上层应用管理，不写入 SDK 配置。

---

## 脚本一览

| 脚本 | 用途 |
|------|------|
| `scan_motors_online.py` | 10 轴在线扫描 |
| `read_states.py` | 持续打印状态 |
| `enable_disable.py` / `link_test.py` | 使能 / 联通 |
| `mit_control_one_motor.py` | 单轴 MIT 示例 |

---

## 配置片段（dual）

```yaml
usb:
  serial: "52A871B1AA5EF4E239371A5083463F26"
  nominal_baud: 1000000
  data_baud: 1000000
  canfd: false
  brs: false

left:
  motors: [ {can_id: 0x04, mst_id: 0x14, ...}, ... ]

right:
  motors: [ {can_id: 0x09, mst_id: 0x19, ...}, ... ]
```

单位：`q` rad，`dq` rad/s，`tau` N·m，`kp` N·m/rad，`kd` N·m/(rad/s)。  
`q` 为**绝对**位置（相对电机零点），不是相对位移。

---

## C++ 构建

```bash
cd dm_openarm && mkdir -p build && cd build
cmake .. && make -j$(nproc) && ctest --output-on-failure
./check_comm ../config/arm.yaml
```

关设备时可能打印 `libusb_transfer_cancelled`，可忽略。

---

## 安全注意

1. 默认 guarded 启动；保持增益需按工位标定，上层应用负责轨迹及前馈。
2. `hold()` / `hold_command()` 成功返回表示完整保持批次已由发送接口提交；`HOLD` 是保护逻辑状态，机械稳定另需反馈验收。
3. Ctrl+C / `disable()` 是软件停止与失能流程，不能替代独立硬件急停。
4. `SHUTDOWN_UNKNOWN` 或失能未确认时报告异常、使用独立硬件处置并人工检查，禁止自动恢复。
5. `usb.device_index` 选择适配器，`usb.serial` 核验所选句柄的身份；索引与 SN 不匹配会拒绝连接，不自动换设备。

更底层的包说明见 [`dm_openarm/README.md`](dm_openarm/README.md)；工位链路笔记见 [`resources/u2canfd/`](resources/u2canfd/)。
