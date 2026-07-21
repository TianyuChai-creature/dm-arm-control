# DM OpenArm

达妙电机**双臂**控制库（C++17 + Python）。  
通信：`libdm_device.so`（dmcan）+ **经典 CAN 1 Mbps**（与 `resources/u2canfd` 工位实测一致）。

| 能力 | 说明 |
|------|------|
| MIT 力矩模式 | 1 kHz 后台循环 |
| 双臂 | 左 `0x01–0x05` / 右 `0x21–0x25`，同一总线 |
| 耦合重力 | 按侧 \(\tau_g=W\phi(q)\)，默认 `scale=0.9` |
| 关节轨迹 | 五次 rest-to-rest，`Limb.move_joints` |

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

---

## 硬件与 ID

| 项目 | 值 |
|------|-----|
| 链路 | 经典 CAN 1M（`canfd: false`, `brs: false`） |
| USB SN | `52A871B1AA5EF4E239371A5083463F26`（改适配器改 YAML） |
| 控制 | MIT @ 1 kHz |

| 侧 | CAN | MST | 顺序 | 型号 |
|----|-----|-----|------|------|
| **left** | 0x01…0x05 | 0x11…0x15 | 腕远端 → 肩 | 腕 DM4310 ×3，肘/肩 DM8009 |
| **right** | 0x21…0x25 | 0x31…0x35 | 同上 | 同上 |

---

## 架构（怎么用 API）

```
Python Arm
├── 设备级：enable / disable / start_mit_loop / states / mit
├── arm.left  (Limb)  → mit / move_joints / 重力 / set_zero …
└── arm.right (Limb)  → 同上
        │
        ▼
C++ MitLoopController @ 1 kHz
  tau_sent = PD + tau_cmd + scale·g_side(q)
```

- **设备级**：开设备、共享 1 kHz 环（两侧一起进 loop）。  
- **侧级 `Limb`**：轨迹、重力、示教保持——**一律用 `arm.left` / `arm.right`**。  
- 根上 **没有** `arm.move_joints`（会报错）。  
- 双臂务必 `start_mit_loop(home=False)`，避免全体硬回零惊吓。

### 控制律

\[
\tau_{\text{sent}}
= k_p(q_d-q)+k_d(\dot q_d-\dot q)+\tau_{\text{cmd}}
+ s\cdot\hat\tau_g(\mathbf{q})
\]

耦合模型（每侧独立 5 轴）：

\[
\hat\tau_{g,i}=\sum_k w_{ik}\,\phi_k(\mathbf{q}),\quad
\phi\in\{1,\sin/\cos q_j,\sin/\cos(q_3+q_4)\}
\]

软 MIT 下轨迹**按规划时长结束**（`MoveResult` 报告误差），不硬等位置进容差。

---

## Python API

### 设备级 `Arm`

| 方法 | 作用 |
|------|------|
| `Arm.from_yaml(path)` | 加载配置，不连设备 |
| `enable()` / `disable()` | 连接使能 / 停 loop + 失能 |
| `start_mit_loop(hz=1000, home=False)` | 启动 1 kHz 环；**双臂用 `home=False`** |
| `stop_mit_loop()` | 停环 |
| `states()` | 全总线反馈（左后右） |
| `mit(can_id, kp=, kd=, q=, dq=, tau=)` | 写任意轴 MIT（底层） |
| `mit({can_id: MitCommand(...), ...})` | 批量 |
| `set_zero` / `set_zero_all` | 当前位置写零（可 `persist` 写 flash） |
| `is_dual()` | 是否双侧都有电机 |
| `left` / `right` | `Limb` 对象 |

### 侧级 `Limb`（`arm.left` / `arm.right`）

| 方法 | 作用 |
|------|------|
| `can_ids` / `n_joints` / `states()` | 本侧元数据与反馈 |
| `mit` / `move_to` / `hold_at` | 本侧 MIT（拒绝异侧 can_id） |
| `move_joints(q_goal, duration=…)` | 本侧五次轨迹 → `MoveResult` |
| `set_gravity_enabled` / `set_gravity_scale` | 本侧重力 |
| `gravity_torques(q=None)` | 本侧 \(g(q)\) |
| `set_zero` / `set_zero_all` | 本侧设零 |

`MoveResult` 字段：`duration`, `q_cmd`, `q_meas`, `err`, `max_abs_err`。

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
arm.left.set_gravity_enabled(True)
arm.right.set_gravity_enabled(True)

for s in arm.left.states():
    print(f"L 0x{s.can_id:02X} q={s.position:+.3f}")

arm.disable()
```

### 3. 单侧轨迹

```python
# 肘 +0.3 rad、肩 -0.3 rad（本侧索引 3、4）
q = [s.position for s in arm.left.states()]
q[3] += 0.3
q[4] -= 0.3
result = arm.left.move_joints(q, duration=3.0, rate_hz=100.0)
print("max|err|", result.max_abs_err)

# 右手同理
qr = [0.0] * 5  # 回到侧内零位
arm.right.move_joints(qr, duration=3.0)
```

```bash
python python_script/move_joints_demo.py --side left
python python_script/move_joints_demo.py --side right
```

### 4. 重力软保持（示教）

```bash
python python_script/hold_with_gravity.py --side left --scale 0.9
python python_script/hold_with_gravity.py --side right --scale 0.9
```

### 5. 重力辨识（按侧）

```bash
# 只掰本侧；对侧软钉。输出 gravity_coupled_identified_{side}.yaml
python python_script/identify_gravity.py --side left
python python_script/identify_gravity.py --side right
# 将 YAML 片段合并进 arm.yaml 对应 left:/right: 的 gravity:
```

### 6. 当前位置设为零位（写 flash）

```python
arm = Arm.from_yaml("dm_openarm/config/arm.yaml")
arm.enable()
arm.set_zero_all(persist=True)   # 左右全部 10 轴
# 或 arm.left.set_zero_all() / arm.right.set_zero_all()
arm.disable()
```

> 设零后若机械零相对辨识姿态变了，应重做该侧重力辨识。

### 7. 双侧运动到关节 0

```python
arm.start_mit_loop(home=False)
arm.left.set_gravity_enabled(True)
arm.right.set_gravity_enabled(True)
arm.left.move_joints([0, 0, 0, 0, 0], duration=3.0)
arm.right.move_joints([0, 0, 0, 0, 0], duration=3.0)
```

---

## 脚本一览

| 脚本 | 用途 |
|------|------|
| `scan_motors_online.py` | 10 轴在线扫描 |
| `read_states.py` | 持续打印状态 |
| `hold_with_gravity.py --side …` | 本侧重力软保持 |
| `identify_gravity.py --side …` | 本侧耦合重力辨识 |
| `move_joints_demo.py --side …` | 本侧可见肩肘轨迹 |
| `enable_disable.py` / `link_test.py` | 使能 / 联通 |

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
  gravity:
    enabled: true
    scale: 0.9
    coupled: { basis: [...], weights: [5 rows] }
  motors: [ {can_id: 0x01, mst_id: 0x11, ...}, ... ]

right:
  gravity:
    enabled: true
    scale: 0.9
    coupled: { basis: [...], weights: [5 rows] }
  motors: [ {can_id: 0x21, mst_id: 0x31, ...}, ... ]
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
2. 重力默认欠补偿 `scale=0.9`：过冲降 scale，慢沉略升。  
3. 软刚度下会有静差，属预期。  
4. 急停：Ctrl+C / `disable()`。  

更底层的包说明见 [`dm_openarm/README.md`](dm_openarm/README.md)；工位链路笔记见 [`resources/u2canfd/`](resources/u2canfd/)。
