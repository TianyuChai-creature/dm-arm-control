# dm_openarm 0.1.14

修复 `connect_passive()` 后 `probe_status()` 收到合法电机状态却返回空列表的问题。适用于本工位 DM8009（DM-J8009P-2EC）和 DM4340P（DM-J4340P-2EC 48V）的标准反馈协议。

## 修复

- `0xCC` 是只读刷新状态请求，回复按正常八字节位置、速度、力矩及温度反馈解析，不再等待四字节 `CANID_L CANID_H CC 00` 回显。
- 在线查询复用有效反馈接收序号，每轴在发出自己的查询前取基线；旧缓存和提前到达的其他轴反馈不能满足后续查询。
- 查询收到的反馈同时更新 `states()`。失能或报警状态均可证明在线，不代表电机健康、使能或已取得使能确认。
- 参数读写、Flash 保存回复与状态反馈继续隔离；短帧、错误通道、错误电机 ID、扩展帧和远程帧不计为状态反馈。
- 删除无效的查询回显计数器与未使用的响应计数接口，替换将四字节回显当作正确协议的旧测试。

## 官方依据

核对达妙官方 `dmBots/motor-sdk` 提交 `0b2ede457bdbf0882e29ab9958ab8fda047b7f4a` 的 [0xCC 请求](https://github.com/dmBots/motor-sdk/blob/0b2ede457bdbf0882e29ab9958ab8fda047b7f4a/Python%E4%BE%8B%E7%A8%8B/u2canfd/damiao.py#L464-L467)及[状态接收解析](https://github.com/dmBots/motor-sdk/blob/0b2ede457bdbf0882e29ab9958ab8fda047b7f4a/Python%E4%BE%8B%E7%A8%8B/u2canfd/damiao.py#L573-L605)，并核对两款电机官方手册的反馈帧表。`libdm_device.so` 保持原文件及 SHA-256 不变。

## 升级

Python 公开 API 保持不变；原生扩展整体重新构建。发行 wheel 为 CPython 3.13 / Linux x86_64，延续 Ubuntu 24.04 构建基线。

集成项目须同步精确版本门控、wheel、依赖锁、SHA-256 和构建身份记录。从 0.1.12 升级仍须遵循 0.1.13 的失能、连续静止设零要求，不能沿用先使能再设零的脚本。

协议没有请求序号，查询期间到达的延迟或主动状态反馈无法严格归属于当前请求；返回值表示查询后观察到有效反馈。STOP/HOLD 控制逻辑未改变，本版不宣称解决未定因的实机抽动。

## 验证

- Debug CTest 6/6；新增协议回归覆盖两种电机型号、MST_ID 回包、失能/使能/报警、反馈更新、缓存隔离、部分在线、逐轴查询基线及非法帧隔离。
- 安装发行 wheel 后 Python 离线测试 73/73；版本、RECORD、安装内容及输入文件 hash 见 `BUILD_MANIFEST.json`。
- 构建和测试未连接真实硬件；本次确认协议实现与离线回归，现场通信和 STOP 抽动不计为硬件验收通过。
