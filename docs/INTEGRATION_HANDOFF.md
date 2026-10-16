# dm-openarm 0.1.15：SDK 与集成端对接记录

本文件随 SDK 维护和发行，用于之后合拢 real-Teleop；不表示集成端已经修改或现场已经验收。

## 基准与边界

- 问题来源：real-Teleop `origin/codex/debug-status-2026-10-16`，提交 `cb307efef53f9bbd3ba7eff802fd6996fadf1e30`，文件 `docs/debug-status-2026-10-16.md`。
- 已知集成主线：`6aa7b6421a1ad90495fe4b7ceb6c2b8fb8970299`，锁定 dm-openarm 0.1.14。该问题分支仅提交记录，未提交文中的本地代码草稿；不能将草稿当作已交付版本。
- SDK 交付版本：0.1.15。wheel 实际源码提交、构建工具链、哈希和测试结果以本次 Release 的 `BUILD_MANIFEST.json` 为准，源码提交与发布合并提交可能不同。
- 本轮只完成 SDK 软件实现、假 CAN/可控时钟回归和构建校验；不操作真实机械臂、Flash、手、Neck、相机或头显。抽动继续搁置，未定因、未关闭。
- 官方 `libdm_device.so` 不替换。新期限是待工位验证的工程默认值，不是机械稳定或硬实时急停保证。

## 固定问题编号与责任

| 编号 | 对应问题与 SDK 交付 | SDK 状态 | 集成端配合 | 实机状态 |
|---|---|---|---|---|
| DM-01 | 原记录第 2 项：普通命令迟到、短时反馈过期采用可恢复保持；持续反馈失效及硬故障仍保护 | 0.1.15 实现，见发行测试记录 | 待适配：取消普通 warning 的关闭/全局停止/故障锁存；区分 RPC 状态迟到与电机反馈失效 | 未验证 |
| DM-02 | 原记录第 2 项：HOLD 待确认、结构化告警和故障 | 0.1.15 实现 | 待适配：识别两种可恢复异常，展示原生状态、原因和请求/提交序号 | 未验证 |
| DM-03 | 原记录第 1/2 项中的关闭结果：失能后查询状态 0，保留未知关闭与所有权 | 0.1.15 实现；不能保证消除所有 SHUTDOWN_UNKNOWN | 待适配：保留原 owner，不将关闭未确认显示为已关闭，不自动重使能 | 未验证 |
| DM-04 | wheel、版本门控、依赖锁及构建身份 | 本仓发行 0.1.15；不改集成仓库 | 待升级并完成集成离线回归；现场进程版本另行核实 | 未验证 |
| APP-01 | 原记录第 3 项：Hand 额外门槛、左手独立控制、右手缺席 | 非 dm-arm-control 职责 | Hand 集成/SDK 后续处理；禁用缺席右手参与，不等同于禁用右臂 | 未完成 |
| APP-02 | 原记录第 4 项：腕状态与重力补偿耦合、实际末端负载 | 非 dm-arm-control 职责 | 删除主机不合理腕位门槛；按侧核对拆除右手后的模型质量；保留前馈有效性和限幅 | 未完成 |
| APP-03 | 原记录第 5 项：Neck pitch 回零失败、响动 | 非 dm-arm-control 职责 | Neck 链路继续暂停；保护清理与抬升能力分别验收 | 暂停、未解决 |
| APP-04 | 原记录第 6 项：视频起流和头显显示 | 非 dm-arm-control 职责 | 核对现场 ZED/pyzed 身份；拆分可选视频与任务强制视觉依赖 | 未完成 |
| OBS-01 | 原记录第 1 项左臂抽动，以及此前 STOP 后疑似抽动 | 责任未定，0.1.15 不宣称修复 | 继续搁置；两次现象不可直接合并为一个根因 | 未定因、未关闭 |

0.1.14 已修正被动查询将四字节请求误当回复的协议错误；该修复与上述超时策略不同。设零的失能/连续静止要求、批量预检查和坐标未知锁存仍保留。

## 默认行为与期限

仅适用于 `enable_seeded()` / `start_seeded()` 的普通 guarded 控制。兼容裸 `enable()+start_mit_loop()` 不新增 CommandGuard；自动标定沿用严格期限和故障锁存，维护与写零条件不放宽。

```python
arm.enable_seeded(
    gains, hz=250.0, command_timeout=0.1, feedback_timeout=0.2,
    feedback_hard_timeout=None, hold_timeout=0.2,
)
```

`feedback_hard_timeout=None` 在原生层解析为 `max(0.5, 2.5 * feedback_timeout)`。显式硬期限必须大于反馈新鲜度期限；所有期限必须有限且为正，非法配置在使能前拒绝。`hold_timeout` 只控制正常 HOLD 等待，不修改反馈失效判据或启动确认期限。

| 条件 | SDK 输出与状态 | 告警/故障 | 后续行为 |
|---|---|---|---|
| ACTIVE 命令年龄达到 command_timeout | HOLD；已提交静态位置命令保持 q/kp/kd/tau，不衰减前馈 | warning COMMAND_TIMEOUT；fault 为空 | 新有效提交直接恢复 ACTIVE，无须 reset |
| 上次提交为 dq 非零或 kp 非正 | 以最后有效实测位置和启动保持增益替代，dq=0，保留有限前馈 | 同上 | 不无限延续旧速度/无刚度运动 |
| 有效反馈年龄 <= feedback_timeout | 正常处理 | 无 FEEDBACK_STALE | 可接收新命令 |
| feedback_timeout < 年龄 < feedback_hard_timeout | HOLD；冻结有效保持输出 | 每轴 warning FEEDBACK_STALE | 拒绝新运动，不入队、不续期 |
| 反馈恢复新鲜且有效、确认使能 | 留在 HOLD，清除反馈告警 | COMMAND_TIMEOUT 若存在则保留到新提交 | 允许新命令；不重放暂停前目标，不重新使能 |
| 有效反馈总年龄 >= feedback_hard_timeout | 停环，尝试零 MIT 和失能 | fault FEEDBACK_LOST | 显式调查与恢复，不自动重使能 |
| 硬故障、意外失能、无效数据或发送失败 | 立即走故障保护，不等待反馈宽限 | 对应稳定原因码 | 保留原始故障与关闭错误 |
| 正常 HOLD 等待超时 | 请求仍待确认，循环继续 | warning HOLD_PENDING / HoldPendingError | 轮询原请求；不得盲目重发 HOLD |
| HOLD 迟到但完整提交成功 | 清除 HOLD_PENDING，更新提交序号 | 不因迟到产生硬故障 | 正常保持，不发送失能帧 |

自动恢复仅表示解除 SDK 的反馈告警和允许新提交。SDK 不知道上游帧的生成时间；集成端仍须检查时戳、来源、版本和授权，不能把旧缓存重新提交冒充新指令。

显式 `hold()` 仍是实测位置保持并让旧前馈渐退；`hold_command()` 仍保持最后已提交静态命令及前馈。正常超时产生的可恢复 HOLD 不等于这两个显式操作，也不等于物理稳定。

## 状态与异常接口

`arm.control_status()` 返回独立、只读的原生 `ControlStatus` 快照：

- `state`：沿用原状态字符串。
- `warnings`、`faults`：`ControlIssue` 列表；告警不会写入 `arm.fault`。
- 每个 issue 有 `code`、`message`；适用时提供 `can_id`、`age_s`、`rx_sequence`，否则为 `None`。故障列表保留初始原因及后续关闭错误。
- `hold_request_sequence`、`hold_submitted_sequence`：本轮 HOLD 请求与完整批次提交序号；开始/停止控制循环时重置，不能当跨会话全局 ID。

活动告警码：`COMMAND_TIMEOUT`、`FEEDBACK_STALE`、`HOLD_PENDING`。

故障码：`FEEDBACK_LOST`、`MOTOR_FAULT`、`MOTOR_DISABLED`、`FEEDBACK_INVALID`、`TX_FAILED`、`STARTUP_FAILED`、`SHUTDOWN_UNCONFIRMED`；维护和兜底原因还包括 `HOLD_FAILED`、`CALIBRATION_FAILED`、`CONTROL_ERROR`。严格自动标定的命令超时仍以 `COMMAND_TIMEOUT` 记入故障。调用方按 code 判断，不解析 message。

`HoldPendingError` 继承 `TimeoutError`；`FeedbackStaleError` 继承 `RuntimeError`。两者都表示本次操作未成功确认/接受，不能经过通用异常处理器无条件触发 close。成功时 `hold()` 仍返回 None，`hold_command()` 仍返回命令列表，`mit()` 仍返回 None。

```python
from dm_openarm import FeedbackStaleError, HoldPendingError

# commands 必须由集成端确认来源、生成时间、授权及版本仍然有效。
try:
    arm.mit(commands)
except FeedbackStaleError:
    show_warning(arm.control_status())   # 不重放、不 close，等待新反馈和新目标
except HoldPendingError:
    show_pending(arm.control_status())   # 不重复提交保持请求
except Exception:
    handle_real_fault(arm.control_status())  # 保留原因及 owner 的关闭结果
    raise
```

HOLD 超时后保存请求序号，轮询 `control_status()`；提交序号达到该请求且状态仍健康时才标为已提交。如果出现 faults、停环或 SHUTDOWN_UNKNOWN，转入实际故障流程。确认仅表示整批发送接口已提交，不表示电机 ACK 或机械稳定。

## 集成端修改清单

1. 在 UI/RPC 中分别传输 `state`、warnings、faults 和 HOLD 序号。warning 可展示和计数，不自动锁存为硬故障。
2. 修改 `DmOpenArmBackend.feedback()` / `_fail_and_close()`：不要仅因达到 0.2s 就关闭；先消费原生状态。保留反馈年龄用于展示，硬故障与真实停环仍处理。原生宽限策略不能与主机无条件关闭并存。
3. 修改 worker 上游超时和会话约 0.3s 状态检查：区分命令迟到、RPC 回报迟到、实际电机反馈失效与驱动器故障。状态包迟到不等于 CAN 反馈失效；不要重复发 HOLD 或扩散为无关设备的全局停止。
4. 在通用异常关闭之前捕获两个可恢复异常；HOLD_PENDING 轮询同一请求，反馈恢复后只发送通过时戳/来源检查的新目标。不要自动 reset 硬故障或重使能。
5. SHUTDOWN_UNKNOWN 保留同一 owner 的实际生命周期和错误；按既有人工恢复流程处理，禁止创建替代 owner 规避。
6. 保留一个 Arm 对应一个命令任务的契约；左右独立任务使用两个 side-selected Arm，共享总线但不共享命令期限。
7. APP-01..04 单独修改与验收；不要将 Hand、重力、Neck、视频的问题归入 SDK 告警修复，OBS-01 继续搁置。

## wheel 升级、身份校验与回退

本轮不执行以下集成操作；此清单供后续交接。

- 从本次 GitHub Release 获取 CPython 3.13/Linux x86_64 wheel、BUILD_MANIFEST、SHA256SUMS、更新日志和本文件。核对文件 SHA256 与构建源码提交；不要仅凭版本号相信同版本重打包。
- 同步 wheels 目录、依赖 manifest、pyproject、uv.lock、精确版本门控及 SDK 构建身份记录。使用集成 CI 固定的工具链生成锁，其他依赖不顺带升级。
- 在独立 Python 环境按冻结锁安装；验证实际 site-packages 来源、安装文件和 wheel 一致，再运行集成离线回归。已有现场进程不会随 Git 或磁盘 wheel 更新自动升级。
- 集成验收检查实际发送帧、warning/fault 传递、恢复不重放旧目标、左右独立控制及显式关闭；软件检查通过仍不能替代实机验收。
- 回退时同时回退集成代码、wheel、锁、门控与构建身份至先前一致提交；0.1.14 的旧超时锁存语义也会恢复。只更换 wheel 会造成接口和语义不一致。回退不得绕过 SHUTDOWN_UNKNOWN 或自动重启硬件。

## 阻塞与验收限制

SDK 继续串行发送完整批次；发送过程中可读取控制状态和观察已发起 HOLD 的待确认状态。新变更在批次边界应用，不并行向同一总线抢发失能帧。

官方库调用若一直不返回，SDK 无法保证停止/失能或首次进入操作的调用方按软件期限返回；0.5s 反馈失效检测也依赖原生循环获得运行机会。该限制必须保留，不能把软件超时文案当硬实时急停保证。

本次 Release 的实际离线结果和构建身份记录在 BUILD_MANIFEST.json。默认宽限、实际失能确认、静止阈值、负载支撑和断电保持均未实机验收。不得将 OBS-01 或整份集成问题清单标为已关闭。
