# dm_openarm 0.1.15

修正普通 guarded 控制将命令迟到、短时反馈过期和 HOLD 等待迟到升级为锁存故障/失能的策略。默认行为发生变化，集成端须同步消费可恢复告警和异常。

## 行为变化

- 命令超过默认 0.1s 时进入可恢复 HOLD，保留最后提交的静态位置、增益与前馈；不锁存 FAULT_HOLD、不自动失能。非静态位置命令转为以最后有效实测位置、启动增益、零速度及有限前馈保持。
- 默认反馈超过 0.2s 先告警并保持；最后有效反馈总年龄达到 0.5s 才作为持续失效保护。短时过期拒绝运动提交，不缓存旧目标；反馈恢复后保持，等待新提交，不自动重放或重使能。
- 启动接口追加 feedback_hard_timeout=None 与 hold_timeout=0.2。自动硬期限为 max(0.5, 2.5*feedback_timeout)；非法期限在使能前拒绝。
- HOLD 等待正常超时报告 HoldPendingError，保留原请求及运行循环；迟到成功不触发失能。整批发送保持串行，control_status() 读取不再等待完整 USB 发送期间的状态锁。
- 新增原生 control_status() 快照、ControlStatus/ControlIssue、FeedbackStaleError 与 HoldPendingError。告警不写入 fault；故障与关闭错误保留稳定原因码。
- 失能确认在原 200ms 窗口内，每 20ms 对未确认轴只读查询；要求逐轴失能前基线后的新鲜状态 0。无确认仍保留 SHUTDOWN_UNKNOWN 与所有权。

## 保留的边界

- 硬故障、意外失能、无效反馈和发送失败仍保护；启动、自动标定、静止设零、批量预检查与部分写入锁存不放宽。
- 显式 hold()/hold_command() 的位置与前馈语义不改变；兼容裸 MIT 入口不新增保护策略。
- 官方 libdm_device.so 原文件及哈希不变。底层调用不返回时，软件超时无法提供硬实时失能保证。
- 本轮未修改或升级 real-Teleop，旧集成代码仍可能自行关闭设备。Hand、重力、Neck、视频不在本版修复范围，抽动继续搁置，未定因、未关闭。

## 对接与验证

集成端待办、原因码、默认期限、异常处理示例和回退步骤见 [对接文档](../docs/INTEGRATION_HANDOFF.md)，同文作为 Release 附件发布。

Debug 原生回归和安装最终 CPython 3.13 wheel 后的 Python 回归、RECORD/安装内容、源码及构建工具链身份以 BUILD_MANIFEST.json 为准。构建和测试全程不操作实机；新期限与实际负载保持尚未实机验证。

本次发行验证：Debug CTest **6/6**；安装最终 CPython 3.13 wheel 后 Python **77/77**。wheel RECORD、安装内容、源码输入和官方库哈希校验通过。
