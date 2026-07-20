# DM-OpenArm 工程开发日志

## 项目概述

DM-OpenArm 是一个 5 自由度机械臂的控制库，基于达妙（Damiao）电机 SDK 构建。

## 目录结构

```
dm_openarm/
├── CMakeLists.txt              # CMake 构建配置
├── pyproject.toml              # Python 包构建配置
├── README.md                   # 项目说明
├── config/
│   └── arm_5dof.yaml           # 机械臂配置文件
├── examples/
│   ├── arm_mit_control.cpp     # MIT 控制示例
│   ├── check_comm.cpp          # 通讯检查示例
│   ├── enable_disable.cpp      # 电机使能/失能示例
│   ├── hold_position.cpp       # 位置保持示例
│   └── set_zero_position.cpp   # 设置零点示例
├── include/
│   └── dm_openarm/
│       ├── dm_arm.hpp          # 机械臂主接口
│       ├── mit_loop_controller.hpp  # MIT 控制循环控制器
│       ├── motor.hpp           # 单电机抽象
│       ├── motor_group.hpp     # 电机组管理
│       ├── types.hpp           # 公共类型定义
│       ├── config.hpp          # 配置数据结构
│       ├── yaml_loader.hpp     # YAML 配置加载器
│       └── backend/
│           ├── motor_backend.hpp       # 后端抽象接口
│           └── dm_serial_backend.hpp   # 达妙串口后端
├── python/
│   └── dm_openarm/
│       ├── __init__.py         # Python 包入口
│       ├── arm.py              # Arm Python API 封装
│       └── examples/
│           ├── basic_enable.py
│           └── mit_loop_control.py
├── src/
│   ├── dm_arm.cpp
│   ├── mit_loop_controller.cpp # MIT 控制循环实现
│   ├── motor.cpp
│   ├── motor_group.cpp
│   ├── python_bindings.cpp     # nanobind Python 绑定
│   ├── yaml_loader.cpp
│   └── backend/
│       └── dm_serial_backend.cpp
├── third_party/
│   └── damiao_sdk/             # 达妙 SDK 第三方依赖
└── tests/
    ├── test_mit_control_command.cpp
    ├── test_mit_loop_controller.cpp
    ├── test_python_api.py
    └── test_yaml_loader.cpp
python_script/                   # 独立 Python 脚本
├── enable_disable.py
├── mit_control_one_motor.py
└── read_states.py
```

## 开发日志

### 2026-05-08
- 初始化项目仓库，搭建整体目录结构
- 创建所有头文件和源文件（`dm_arm`、`motor`、`motor_group`、`mit_loop_controller`、`yaml_loader`、`python_bindings`、`dm_serial_backend` 等）
- 录入上位机电机初始配置（5× DM 电机，MIT 模式）
- 实现 MIT 控制循环（C++ `MitLoopController` + nanobind Python 绑定）
- 实现 Python API `arm.py`：
  - `Arm` 类封装 enable/disable、状态读取、MIT 命令等
  - `start_mit_loop()` 启动后自动命令所有电机回零，阻塞等待到位再返回
  - 统一 MIT 接口 `arm.mit()` 支持单电机 / 多电机命令
- 编写独立 Python 脚本 (`python_script/`)：`enable_disable.py`、`read_states.py`、`mit_control_one_motor.py`

---

## 电机初始配置

### 通讯配置

| 项目 | 配置 |
| --- | --- |
| CAN 协议 | CAN 2.0 |
| 上位机波特率 | 1M |
| 电机波特率 | 全部为 1M |
| 控制模式 | MIT 模式 |

### 电机配置表

| 序号 | CAN ID | MST ID | 型号 | 安装位置 | 模式 |
| --- | --- | --- | --- | --- | --- |
| 1 | 0x01 | 0x11 | DM4310 | 末端 | MIT |
| 2 | 0x02 | 0x12 | DM4310 | 末端上数第二 | MIT |
| 3 | 0x03 | 0x13 | DM4310 | 末端上数第三 | MIT |
| 4 | 0x04 | 0x14 | DM8009P | 末端上数第四 | MIT |
| 5 | 0x05 | 0x15 | DM8009P | 末端上数第五（肩膀位置） | MIT |

### 备注

- 当前 5 个电机均使用 **MIT 模式**。
- 上位机与电机侧波特率均设置为 **1M**。

---

---

## 技术笔记

### 2026-07-20 — u2canfd 链路移植

- **根因**：`dm_openarm` 原先链接旧栈 `libu2canfd.a`（`usb_class`），而工位实测可用的是 `resources/u2canfd` → `libdm_device.so`（dmcan API）+ **经典 CAN 1M**（`canfd=false, brs=false`）。
- **改造**：`Motor_Control` 改为 dmcan 开设备/设波特率/收发；YAML 增加 `canfd`/`brs`/`device_index`；`enable_all` 对电机去重。
- **关闭设备**：本机上同时调用 `dmcan_device_close` + `dmcan_context_destroy` 会触发 libusb 断言；现仅 `context_destroy` 收尾（与 u2canfd `test_link.py` 规避同类问题一致）。
- **验证**：`./check_comm` 对 5 轴反馈 PASS。

### 2026-07-20 — 重力补偿（分支 gravity-compensation）

- 模型：每轴 `tau_g = amp * sin(q + phase) + bias`（解耦一阶，便于辨识；后续可换耦合模型）
- MIT loop 每周期：`tau_sent = cmd.tau + scale * g(q)`，`q` 默认用测量位置
- YAML：`gravity.enabled / scale / use_measured_q / joints[]`
- Python：`set_gravity_enabled`、`set_gravity_scale`、`gravity_torques`；`start_mit_loop(home=False)` 支持不回零保持
- 示例：`python_script/hold_with_gravity.py`
- 辨识：`python_script/identify_gravity.py` + `dm_openarm.gravity_fit`（线性 LS 拟合 amp/phase/bias，输出 YAML）