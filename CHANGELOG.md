# Changelog

本文件记录 VectorFOC 固件的所有重要变更。格式基于 [Keep a Changelog](https://keepachangelog.com/)。

---

## [1.0.0] — 2026-03-24

**首个正式发布版本**。STM32G431 FOC 电机控制器固件，经两轮专家级代码审查 + 43 项单元测试全部通过。

### 核心功能

#### FOC 算法引擎
- Clarke/Park/逆Park 变换 + SVPWM 调制，电流内环 20kHz
- 速度/位置外环 5kHz，支持 PID 和 LADRC 两种速度控制器
- 电压限幅 + Anti-windup + 解耦前馈补偿
- 电流物理量换算集中在 `FAC_CURRENT` 宏，参数唯一来源

#### DS402 状态机
- 完整 9 状态实现：NOT_READY → SWITCH_ON_DISABLED → READY_TO_SWITCH_ON → SWITCHED_ON → OPERATION_ENABLED → QUICK_STOP_ACTIVE → FAULT_REACTION_ACTIVE → FAULT → CALIBRATING
- `AutoAdvanceToTarget()` 多步自动转换机制（解决跨状态 RequestState 卡死问题）
- Per-instance 故障历史环形缓冲区
- `FAULT_REACTION_ACTIVE` 受控停机阶段（DS402 合规）
- 校准状态支持 Shutdown 命令退出

#### 三协议通信栈
- **Inovxio 私有协议**：CAN 2.0B 29-bit 扩展帧，20 条命令，支持广播寻址（Target=0x7F）
- **CANopen DS402**：NMT / SDO / PDO / Heartbeat / Emergency
- **MIT Cheetah 协议**：12 字节紧凑格式，适用于机器人关节控制
- 协议管理器统一路由，运行时切换 <1ms
- 指令执行器（Executor）分层解耦，业务逻辑与协议翻译分离

#### 控制模式
- 位置模式（梯形轨迹规划 + PID）
- 速度模式（PID / LADRC 可选）
- 电流/力矩模式（直接 Iq 设定）
- 混合控制（位置 + 速度 + 力矩前馈，MIT 风格）

#### 校准系统
- 编码器偏移校准
- 电阻/电感在线辨识
- 磁链校准（低速防护：ω_e < 1.0 rad/s 跳过采样，避免数值不稳定）

#### 参数系统
- Flash 持久化存储，CAN 远程读写（CMD 17/18）
- 参数集中管理，无魔法数字
- 保存命令（CMD 22）写入 Flash

#### OTA Bootloader
- AN3155 协议兼容，支持远程固件升级
- 系统 Bootloader 跳转（CMD 13）

#### 算法库
- PID 控制器：Anti-windup + 输出限幅，执行时间 ~1μs
- 速率限制器：可配置上升/下降速率，~0.5μs
- 梯形轨迹规划器：加速-匀速-减速三段式，~5μs

#### BSP 驱动
- STM32G431 ADC DMA 双通道采样（`volatile` 修饰，编译器优化安全）
- FDCAN 静态内存池（无 malloc），DLC 安全截断
- PWM 三相输出 + 使能控制

#### 调试接口
- VOFA+ 实时波形监视
- 接收缓冲区溢出保护 + null 终止
- 命令解析 strstr 精确定位

### 代码审查修复

#### 第一轮（2026-02-08）— 14 项修复

**Critical (4)**:
- `fsm.c`: `prev_controlword` 边沿检测 bug — 提前更新，覆盖所有 return 路径
- `fsm.c`: `EnterFault` 跳过 `FAULT_REACTION_ACTIVE` — 补全 DS402 受控停机
- `fsm.c`: `RequestState` 多步转换失效 — 新增 `AutoAdvanceToTarget()` 机制
- `bsp_adc.h/c`: ADC DMA 缓冲区缺少 `volatile` — 防止编译器优化导致读取过期值

**Medium (6)**:
- `fsm.c`: `ClearFault` ISR 安全隐患 — 改为直接 ExecuteTransition
- `fsm.c`: `HandleStateEntry` 缺少 3 个状态处理 — 补全 FAULT_REACTION_ACTIVE / QUICK_STOP_ACTIVE / NOT_READY
- `fsm.c`: 故障历史全局静态变量 — 移入 StateMachine 结构体（多实例安全）
- `executor.c`: 模式切换与设定值更新竞态 — 合并为单一临界区
- `flux_calib.c`: 低速磁链校准数值不稳定 — 阈值提高到 1.0 rad/s + 跳过采样
- `bsp_can.c`: malloc + 残缺函数 + DLC 越界 — 静态内存池 + 补全 + 安全截断

**Minor (4)**:
- `fsm.h`: 缺少 `STATE_COUNT` 边界标记 + 校准退出路径
- `foc_algorithm.c`: 魔法数字 → `ONE_OVER_SQRT3` / `VOLTAGE_MARGIN` 命名常量
- `vofa.c`: 接收缓冲区溢出 + null 终止
- `vofa.c`: `cmd_parse` 偏移量计算错误 → strstr 定位

#### 第二轮（2026-02-22）— 7 项全部 ✅

| 审查项 | 评分 |
|--------|------|
| FOC 闭环完整性 | ✅ 良好 |
| 参数唯一来源 | ✅ 良好 |
| 数学正确性 | ✅ 良好 |
| 任务调度与时序 | ✅ 良好 |
| 状态机与模式切换 | ✅ 良好 |
| 保护与限幅 | ✅ 良好 |
| 可测试性 | ✅ 良好 |

### 测试

- **43/43 单元测试通过**（7 个测试套件）
- 覆盖：FOC 算法、状态机、通信协议、PID、速率限制器、轨迹规划、参数存储

### 已知限制（计划 v1.1 修复）

- CAN 总线扫描：固件侧已支持广播响应，主机侧扫描流程待实现（v1.1 F1）
- Flash 参数写入非原子：掉电可能损坏参数（v1.1 F2）
- `bsp_can.c` 错误时 `while(1)` 死循环无 watchdog feed
- PID / RateLimiter 缺少 NaN/Inf 输入防护

### 构建

- **MCU**: STM32G431CBU6
- **工具链**: arm-none-eabi-gcc
- **CI/CD**: GitHub Actions — tag 推送自动编译 + 发布 Release（.elf / .bin / .hex）

---

*VectorFOC — 穹沛科技 FOC 电机控制固件*
