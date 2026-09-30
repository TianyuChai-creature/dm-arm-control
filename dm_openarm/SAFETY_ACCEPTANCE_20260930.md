# dm_openarm 离线安全改造验收（2026-09-30）

## 范围与结论

本次以 `f7e9a96f9d8e9748d410d44cb455572b3c588cfe`（0.1.8）为基线，在 `audit/dm-openarm-safety-20260930` 分支修改 SDK。离线软件测试通过；**没有实机验收、发布新 wheel 或升级消费端**。以下结论仅针对源码及假 CAN 传输测试。

## 变更

- FC/FD、模式、探测、设零及保存等关键发送失败显式上报；FD 后须观察新的、匹配的状态 0 反馈，否则标记失能未知并保留 CAN owner。厂商协议没有独立 FC/FD ACK，因此状态 0 观察不等于命令级 ACK，也不保证机械负载安全。
- 原生 MIT loop 正常停止和异常停止均先结束 worker，再发送零 MIT，并尝试 FD；FD 或状态确认失败会报告 `SHUTDOWN_UNKNOWN`。直接原生 disable/disconnect、维护设零须先停止 worker。
- 完整 MIT 批量命令按各轴型号范围预验证，避免可预知的后轴越界导致前轴已发送；真实链路故障仍无法保证 CAN 帧原子性。
- 持久设零不再无条件重使能本侧全部轴；先核验目标轴状态，保存或恢复失败会锁存坐标/状态未知。探测仅接受 0xCC 格式回复。因协议无 nonce，延迟旧回复仍无法严格归属于本次查询。
- 新增 `raw_status`、`enabled_confirmed` 诊断字段。原有 `Arm.from_yaml(side=)`、`enable_seeded`、`mit(dict)`、`states`、`feedback_fresh`、`hold_command`、`probe_status`、`disable` 调用形状保留；原生 ABI 已改变，升级须整体重新构建。

## 离线验证

- Debug CMake/native Python extension 完整构建成功；CTest 6/6（含假传输安全矩阵）通过。
- 源仓 Python API 测试 6/6 通过；消费端 real-Teleop 后端、ready、runtime 离线测试 28/28 通过，测试时通过 `PYTHONPATH` 指向临时新包，未改消费端 pin。
- 先前以两种实际电机型号（DM4340P、DM8009）验证合法 MIT 编码帧旧/新字节一致。`git diff --check` 通过。

## 升级与实机门槛

- real-Teleop 当前固定 `dm_openarm 0.1.7.post2` wheel/hash，本次未改锁文件。现有独立设零脚本紧接 `enable()` 执行 `set_zero_all(persist=True)`；新版需要及时观察状态 1，故该脚本原样运行可能被安全拒绝，须单独适配和验收。
- `Arm.disable()` 在失能反馈缺席时报告未知；消费端既有 `close()` 异常处理可能遮蔽原始错误。消费端升级前应保留并展示原始故障。
- 工位仍须验证单侧/双侧启停、状态反馈时序、断链后反馈行为、负载支撑、机械限位、250 Hz 十轴延迟/丢帧/温升、人工急停与维护设零流程。离线通过不能代替这些测试。
- 若升级失败，继续或恢复消费端已锁定的旧 wheel/hash，并先核实设备物理状态；不得因软件进程重启就推定电机已失能。
