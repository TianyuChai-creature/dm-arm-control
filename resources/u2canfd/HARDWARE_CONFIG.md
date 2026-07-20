# 硬件与通信配置记录

> 记录本仓库在当前工位实测可用的适配器与电机参数。  
> 更新日期：2026-07-20  
> 验证脚本：`dev_sn.py`、`test_link.py`、`scan_motors.py`

---

## 1. USB 转 CANFD 适配器

| 项 | 值 |
|----|-----|
| 设备类型 | 单路 USB2CANFD（`dmcan_device_type.USB2CANFD`） |
| VID / PID | `0x34B7` / `0x6877` |
| **Serial Number (SN)** | **`52A871B1AA5EF4E239371A5083463F26`** |
| 驱动库 | `dlls/libdm_device.so`（Linux x86_64） |
| 固件版本（读回） | app v1.0.0.6 |

查询 SN：

```bash
source .venv/bin/activate
python -u dev_sn.py
```

---

## 2. 适配器 / 总线波特率（本仓库实测可用）

本机电机在 **经典 CAN 1 Mbps** 下可正常收发；**CAN FD 模式（含数据域 1M 或 5M）实测总线静默、无回帧**。

| 项 | 配置值 | 说明 |
|----|--------|------|
| 链路模式 | **经典 CAN** | `canfd=False`，`brs=False` |
| **标称/控制通讯波特率** | **1 000 000 (1M)** | 适配器 `can_baudrate` |
| 数据域波特率 | 不使用（读回为 0） | 经典 CAN 无独立数据域 |
| 采样点 | 0.75（SDK 默认写入） | `can_sp` |
| 例程构造示例 | `Motor_Control(1000000, 1000000, SN, ..., canfd=False, brs=False)` | 第二参数在经典 CAN 下可忽略 |

**对比（实测无效，勿与当前电机混用）：**

| 模式 | 结果 |
|------|------|
| CAN FD，仲裁 1M / 数据 5M | 无 RX |
| CAN FD，仲裁 1M / 数据 1M | 无 RX |
| 经典 CAN 5M | 无 RX |
| **经典 CAN 1M** | **有 RX，多轴在线** |

> Windows 上位机侧若显示“数据域 1M”，与当前 Linux 侧 **经典 CAN 1M** 配置对应；**不是** 例程文档默认的 CANFD 数据域 5M。

---

## 3. 电机 ID 表（实测全部在线）

扫描时间：2026-07-20（`scan_motors.py`，MIT 全 0，不推力矩）。

| canid | mstid（主站/反馈 ID） | 在线 | 备注 |
|-------|----------------------|------|------|
| `0x01` | `0x11` | 是 | mstid = canid + 0x10 |
| `0x02` | `0x12` | 是 | 同上 |
| `0x03` | `0x13` | 是 | 同上 |
| `0x04` | `0x14` | 是 | 同上 |
| `0x05` | `0x15` | 是 | 同上 |

- 电机类型（例程注册）：`DM_Motor_Type.DM4310`（若实际型号不同，仅影响限位解码，不影响在线判定）  
- 控制模式：`Control_Mode.MIT_MODE`  
- 控制通讯波特率：与总线一致，**1M 经典 CAN**

复测：

```bash
source .venv/bin/activate
python -u scan_motors.py
```

---

## 4. 代码中的对应位置

| 参数 | 主要文件 |
|------|----------|
| SN | `damiao.py`、`test_link.py`、`scan_motors.py` |
| canid / mstid | `damiao.py`（单轴示例）、`scan_motors.py`（0x01~0x05） |
| 波特率 + classic CAN | `Motor_Control(..., canfd=False, brs=False)`，`nom_baud=1000000` |

---

## 5. 变更说明

更换适配器或改电机波特率/ID 后，请同步更新：

1. 本文件各表  
2. `damiao.py` / `test_link.py` / `scan_motors.py` 中的 `SN`、波特率与 ID 列表  
