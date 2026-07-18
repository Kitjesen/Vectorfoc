# VectorFOC 主机仿真与回归说明

本文档只描述仓库中可重复运行的主机级验证。它们不需要 STM32 硬件，但也不能替代示波器、测功机、真实 CAN/USB 链路或故障注入台架。

## 已注册的主机模型与回归

`test/CMakeLists.txt` 将下列测试注册进默认 CTest 套件：

- `test_runner_integration`：文件内简化 PMSM 模型。覆盖电流环阶跃、速度环阶跃、速度设定切换、负载扰动、位置设定跟踪和五组小范围参数扫掠；对有限值和速度/电流边界做断言。
- `test_runner_foc_closed_loop`：把真实 `MotorStateTask` 与 mock HAL、`motor_plant.c` 连接，检查速度闭环的最终误差、PWM duty、d/q 电流和 plant 状态均保持有限且有界；CSV 无法创建也会使测试失败。
- `test_runner_fault_injection`：使用确定性伪随机 ADC 电流噪声，而不是空实现的“噪声开关”；检查噪声下电流、速度、PWM 和 d/q 轴状态的边界。
- `test_runner_foc_state`：真实 `MotorStateTask` 的运行、故障去使能和电流标定路径。
- `test_runner_foc_readiness_gate`：真实 ADC injected callback 在初始化完成前不得触碰 ADC 更新、编码器、安全、控制器、observer、PWM 或心跳路径；ready 后才进入该运行路径；X-STAR-S 分支还验证 ADC2 未完成、错误和陈旧完成标记都会拒绝样本。
- `test_runner_app_freertos_runtime_stats`：调度器未启动、任务句柄不完整和三个核心任务就绪三种场景下，验证任务栈高水位诊断的安全返回与采样值。
- `test_runner_adc_shared_irq`：ADC1/ADC2 共用 IRQ 时，只分发 pending 且已启用中断源的 ADC handle。
- `test_runner_fsm_safety`：验证 PWM HAL 调用不在全局 critical section 内执行，并覆盖 PWM 过渡时故障的直接桥臂关断。
- `test_runner_cmd_service_persistent_rollback`：验证命令保存和直接标定式保存连续失败到终态时均会清理对应请求并回滚运行时参数。
- `test_clock_configuration`：检查 8 MHz HSE、PLL 注释和构建期 `HSE_VALUE` 宏保持一致。

这些测试能证明主机模型和选定的真实 C 入口在其 mock 边界内工作，不能证明实际 MOSFET、电流采样、编码器、USB CDC、CAN 仲裁或实时调度已经在板上通过。

## 运行方式

```bash
cmake -S test -B build-test
cmake --build build-test --parallel
ctest --test-dir build-test --output-on-failure
```

只运行简化 PMSM harness：

```bash
ctest --test-dir build-test -R test_runner_integration --output-on-failure
```

只运行真实 `MotorStateTask` + plant 回归：

```bash
ctest --test-dir build-test -R "test_runner_(foc_closed_loop|fault_injection|foc_state|foc_readiness_gate)" --output-on-failure
```

Visual Studio 等多配置生成器需要显式配置：

```powershell
cmake --build build-test --config Debug --parallel
ctest --test-dir build-test -C Debug --output-on-failure
```

## 分析速度设定切换

`test_runner_integration` 在其工作目录生成 `foc_setpoint_switch.csv`。在构建目录运行：

```bash
python ../test/analyze_results.py foc_setpoint_switch.csv
```

脚本读取 `RefSpeed`/`ActualSpeed` 或 `RefVel`/`ActVel`，输出目标速度、最终速度、误差和超调量，并生成 `foc_setpoint_switch.png`。CSV/PNG 都是可再生构建产物，不纳入版本库；数值是本机模型回归证据，不是电机性能规格。

## 已覆盖与尚未覆盖的故障

主机侧已覆盖确定性 ADC 噪声、ADC 样本保护、X-STAR ADC1/ADC2 paired-sample gate、ADC shared IRQ 分发、编码器失败守卫、看门狗监督、CAN timeout 的 arm/run-state 语义、CANopen STOP 输入门控、Vector GET_ID 畸形帧拒绝、快速故障上报/回调重试、FSM 过渡期桥臂关断、scheduled-save 终态回滚、故障清除状态契约和通信命令边界。

下列场景仍需要硬件在环（HIL）或台架验证，不能声称已经由主机仿真覆盖：

- 三相缺相、栅极/PWM 卡死、实际过流和母线浪涌；
- 编码器断线、冻结、跳变和磁干扰；
- 真实 ADC 饱和、ADC1/ADC2 触发相位、采样时序漂移、DMA/中断延迟；
- PWM 使能/去使能的最坏延迟与故障到桥臂实际关断的硬件时序；
- CAN 总线拥塞、仲裁丢失、物理 TX abort 和 USB CDC 断连/重连；
- 真实负载堵转、热模型、低压启动和掉电恢复；
- 真实 Flash 擦写/校验故障注入，以及运行时栈高水位、CCM/RAM 余量和看门狗复位恢复。固件现已在 `task_guard` 的每秒诊断日志中输出 `stack_free_w=default/guard/comm`（单位为 FreeRTOS word），但尚未在真实负载下采集这些数据。

建议的下一步是以受限电压和无负载台架开始，逐项记录电流限幅、PWM 去使能、错误历史、CAN/USB 应答、复位恢复、`stack_free_w` 与 CAN `rx_queue_peak`，再进入带负载的 HIL 测试。在这些数据证明余量前，不要缩小任务栈、USB/VOFA 队列或 CAN 队列。
