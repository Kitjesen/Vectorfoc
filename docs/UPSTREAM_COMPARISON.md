# 本地工作区与 GitHub 主线对照

更新时间：2026-10-07（Asia/Shanghai）。本次整理以 `4520d1c` 为起点，发布到独立 `codex/vector-control-cleanup` 分支；对照已重新抓取的 GitHub `origin/main` 提交 `a756902`。本分支未合并主线新增的 15 个提交，不覆盖远端 `main`。

## 结论

当前没有一个单独的“完整领先版本”。两边领先的方向不同：

| 维度 | 当前工作区 | GitHub `origin/main` |
| --- | --- | --- |
| 阅读结构 | 领先：`src/` 已压成 `app/boot/comm/config/foc/hal/motor/safety` 八个职责目录 | 仍是 `Src/ALGO`、`Src/APP`、`Src/COMM`、`Src/HAL`、`Src/UI` 多层目录 |
| FOC 调用链 | 领先：电流环已经并入 `src/foc/control_dispatcher.c` | 仍保留 `control.c → inner.c` 的内部转发 |
| 安全与故障硬化 | 需要追平 | 领先：看门狗监督、ADC 采样保护、编码器失效保护、更多状态机边界检查 |
| 位置传感器 | 当前接口较薄，MT6816/TMR3109 由 `encoder_interface.c` 条件编译选择 | 领先：增加统一 `position_sensor` 适配层、能力描述和健康状态 |
| 参数与通信边界 | 当前已扁平化，但部分远程新增设置/通信修复尚未移植 | 领先：运行时设置、参数绑定、通信启动和 CAN transport 修复更完整 |
| 测试数量 | 当前 19 项 host 检查及 16 组频率配置检查通过 | 主线包含约 62 个测试源文件和更宽的安全/接口覆盖；尚未在本工作区重跑主线测试 |

本分支的共同起点相对 `origin/main` 落后 15 个提交；本次提交包含目录迁移、纯算法分离、控制调度收缩和频率验证。两条分支存在功能差异，不能把本分支直接当作主线完整升级版本。

## GitHub 主线新增的主要内容

- `position_sensor/` 统一位置传感器的选择、采样、健康状态和失败计数。
- `watchdog_supervisor`、ADC 采样保护、编码器失效保护和更严格的 FSM 使能检查。
- `command_limiter`、`torque_utils` 以及运行时设置/参数绑定拆分。
- CAN transport 从 HAL BSP 侧迁入通信侧，并补充大量通信、参数、启动和故障测试。
- CI 增加更完整的测试目标。

## 当前工作区已经完成的整理

- 旧 `src/control`、`src/ui`、`executor/manager/protocol` 多层目录已从磁盘移除。
- 电流环已收敛到 `algorithm/foc_current_loop.c`，由 `src/foc/control_dispatcher.c` 统一调度，旧的内部转发已删除。
- 默认硬件适配的 ARM 构建通过；算法库也可独立主机构建。
- 19 项 host 检查通过，配置阶段另检查 16 组频率；控制、标定和前馈 5 组差分逐周期匹配。

## 合并顺序

后续应以当前八目录结构作为阅读入口，把 GitHub 主线的安全和传感器能力逐项移植到对应目录：

1. 先移植位置传感器健康状态与编码器失效保护，保持 `encoder_interface.c` 作为唯一运行入口。
2. 再移植 ADC 采样保护、看门狗和 FSM 边界检查，逐项增加当前 `tests/` 的回归用例。
3. 最后移植运行时设置、参数绑定和通信 transport 修复，核对协议字段、参数 ID 和 Flash 布局。
4. 完成 host 与默认 ARM 构建后，再整理为独立提交。

本文件只记录比较结果，不把 GitHub 主线代码直接覆盖到当前结构。后续合并需保留主线已有的安全修复，并在新目录中逐项验证。
