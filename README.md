# DM OpenArm

达妙电机**双臂**控制库（C++17 + Python）。  
通信：`libdm_device.so`（dmcan）+ **经典 CAN 1 Mbps**（与 `resources/u2canfd` 工位实测一致）。

| 能力 | 说明 |
|------|------|
| MIT 力矩模式 | 1 kHz 后台循环 |
| 双臂 | 左 `0x00–0x04` / 右 `0x05–0x09`，同一总线 |
| 前馈力矩 | 由上层应用计算后写入 `MitCommand.tau` |

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
uv build --wheel --out-dir dist --no-create-gitignore dm_openarm
```

生成的 wheel 和安装使用说明位于 `dist/`。当前 SDK 默认控制频率固定为最高默认频率
`1000 Hz`，不从 YAML 读取控制频率；需要降频时在 `start_mit_loop(hz=...)` 中显式指定。

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
├── 设备级：enable / disable / start_mit_loop / states / mit
├── arm.left  (Limb)  → mit / set_zero …
└── arm.right (Limb)  → 同上
        │
        ▼
C++ MitLoopController @ 1 kHz
  tau_sent = tau_cmd
```

- **设备级**：开设备、共享 1 kHz 环（两侧一起进 loop）。  
- **侧级 `Limb`**：按侧限制 CAN ID 并提供基础 MIT/设零操作。
- 双臂务必 `start_mit_loop(home=False)`，避免全体硬回零惊吓。

---

## Python API

### 设备级 `Arm`

| 方法 | 作用 |
|------|------|
| `Arm.from_yaml(path, side=None)` | 加载双臂或指定单臂配置，不连设备 |
| `enable()` / `disable()` | 连接使能 / 停 loop + 失能 |
| `start_mit_loop(hz=1000, home=False)` | 默认以最高频率 1000 Hz 启动控制环 |
| `stop_mit_loop()` | 停环 |
| `states()` | 全总线反馈（左后右） |
| `deadline_misses` | MIT 循环累计超期次数 |
| `mit(can_id, kp=, kd=, q=, dq=, tau=)` | 写任意轴 MIT（底层） |
| `mit({can_id: MitCommand(...), ...})` | 批量 |
| `set_zero` / `set_zero_all` | 当前位置写零（可 `persist` 写 flash） |
| `left` / `right` | `Limb` 对象 |

`MotorState` 还提供 `last_rx_age_s`、`rx_sequence`、`error_code` 和
`feedback_fresh(max_age_s=0.1)`，用于判断反馈是否新鲜。

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
from dm_openarm import Arm

arm = Arm.from_yaml("dm_openarm/config/arm.yaml")
arm.enable()

# 先 seed 当前角，再开环（勿 home=True）
for s in arm.states():
    arm.mit(int(s.can_id), kp=12.0, kd=0.6, q=float(s.position), dq=0.0, tau=0.0)

arm.start_mit_loop(hz=1000.0, home=False)

for s in arm.left.states():
    print(f"L 0x{s.can_id:02X} q={s.position:+.3f}")

arm.disable()
```

### 3. 当前位置设为零位（写 flash）

```python
arm = Arm.from_yaml("dm_openarm/config/arm.yaml")
arm.enable()
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

1. 双臂 **`home=False`**，先 seed 再开 loop。  
2. 上层应用负责轨迹、动力学和前馈力矩。
3. 急停：Ctrl+C / `disable()`。

更底层的包说明见 [`dm_openarm/README.md`](dm_openarm/README.md)；工位链路笔记见 [`resources/u2canfd/`](resources/u2canfd/)。
