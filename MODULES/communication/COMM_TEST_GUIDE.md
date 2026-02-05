# 通信链路负载测试与实时性验证指南

## 目的
验证通信架构优化后的性能指标，确保高负载下不丢帧，电机控制实时性不退化。

---

## 测试环境准备

### 硬件要求
- VectorFOC 开发板（STM32G431）
- CAN 收发器和总线
- 上位机（用于发送 CAN 指令）
- 示波器或逻辑分析仪（可选，用于测量时序）

### 软件要求
- 编译时启用 DEBUG 模式以记录处理耗时统计
- VOFA+ 或串口调试工具（用于监控统计数据）

### 测试代码准备

在 `task_comm.c` 或主循环中添加统计输出：

```c
#include "manager.h"

// 在通信任务中周期性输出统计（每 1 秒）
void StartCustomTask(void const *argument) {
  (void)argument;
  CmdService_Init();
  
  uint32_t last_log_time = 0;
  
  for (;;) {
    Protocol_ProcessQueuedFrames();
    CmdService_Process();
    
    // 每秒输出统计
    uint32_t now = HAL_GetTick();
    if (now - last_log_time >= 1000) {
      CommStats_t stats;
      Protocol_GetStats(&stats);
      
      printf("CAN Stats: RX=%lu (dropped=%lu, overflow=%lu), TX=%lu (fail=%lu), "
             "QDepth=%lu (peak=%lu), ParseErr=%lu, ExecMax=%luus\r\n",
             stats.rx_frames_total, stats.rx_frames_dropped, stats.rx_overflow_events,
             stats.tx_frames_total, stats.tx_frames_failed,
             stats.rx_queue_depth, stats.rx_queue_peak,
             stats.parse_errors, stats.exec_time_max_us);
      
      last_log_time = now;
    }
    
    osDelay(2);
  }
}
```

---

## 测试项目

### 1. CAN 高负载测试

**目标**: 验证队列不溢出，丢帧率在可控范围。

**测试步骤**:
1. 上位机以最大速率发送 CAN 指令（100% 总线负载）
2. 持续发送 60 秒
3. 监控统计输出：
   - `rx_frames_dropped` 应为 0 或极低（<0.1%）
   - `rx_overflow_events` 应为 0
   - `rx_queue_peak` 应 < 队列长度（32）

**上位机测试脚本示例（Python + python-can）**:

```python
import can
import time

bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)

# 发送速度控制命令（Inovxio 协议 CMD 0x10）
def send_velocity_cmd(can_id, velocity):
    msg_id = (0x10 << 24) | (can_id << 8) | 0x00
    data = int(velocity * 100).to_bytes(4, 'little') + b'\x00' * 4
    msg = can.Message(arbitration_id=msg_id, is_extended_id=True, data=data)
    bus.send(msg)

# 高负载测试：500Hz 发送频率
start_time = time.time()
count = 0
while time.time() - start_time < 60:  # 持续 60 秒
    send_velocity_cmd(127, 5.0)  # 发送速度指令 5 turn/s
    count += 1
    time.sleep(0.002)  # 2ms 间隔 (500Hz)

print(f"Total sent: {count} frames")
```

**预期结果**:
- 丢帧率 < 0.1%
- 队列峰值 < 20（远小于队列容量 32）
- 无溢出事件

---

### 2. 电流环实时性测试

**目标**: 确认电流环控制周期不受通信影响。

**测试步骤**:
1. 使能电机并运行速度控制模式
2. 同时施加 CAN 高负载（如测试 1）
3. 使用 VOFA+ 监控电流波形和控制误差

**检查项**:
- 电流环频率应保持 20kHz（无抖动）
- Iq 跟随 Iq_ref 的延迟 < 100us
- 电流波形无毛刺或突变

**VOFA+ 通道配置**:
```
Ch0: Iq (交轴电流)
Ch1: Iq_ref (交轴电流给定)
Ch2: Id (直轴电流)
Ch3: 电角度
Ch4: 速度估计
```

**预期结果**:
- 电流环响应时间无退化（对比优化前）
- 控制误差 < 5%
- 无异常震荡或失步

---

### 3. Flash 写入路径验证

**目标**: 确认参数写入和 Flash 持久化不在 ISR 上下文触发。

**测试方法**:
1. 在 `Param_WriteFloat` 和 `Param_ScheduleSave` 中添加断点或断言
2. 在调试器中查看调用栈
3. 确认调用来自任务上下文，而非 ISR

**添加断言（可选）**:

```c
// 在 param_access.c 的 Param_WriteFloat 中添加
ParamStatus Param_WriteFloat(uint16_t index, float value) {
  // 确认不在中断中调用
  assert(__get_IPSR() == 0);  // IPSR == 0 表示线程模式
  // ... 原有代码
}
```

**预期结果**:
- 所有参数写入和 Flash 触发均在任务上下文
- 无断言失败

---

### 4. 队列溢出恢复测试

**目标**: 验证队列溢出时的错误处理和恢复能力。

**测试步骤**:
1. 临时将队列长度设置为 4（在 `manager.c` 中）
2. 以高速率发送 CAN 指令触发溢出
3. 观察错误日志和统计
4. 停止高负载后，验证通信恢复正常

**预期结果**:
- 溢出时 `rx_overflow_events` 计数增加
- 错误管理器记录溢出事件
- 停止高负载后，通信恢复正常（无死锁或挂起）

---

## 性能基线参考

### 优化前（假设 ISR 中处理协议）
- 电流环 jitter: ~5-10us
- CAN 处理时间: 50-200us/帧（取决于操作）
- 最大 CAN 负载: 300 帧/秒（丢帧率 <1%）

### 优化后（ISR 仅入队）
- 电流环 jitter: <2us
- CAN 入队时间: ~5-10us/帧
- CAN 处理时间（任务中）: 20-50us/帧
- 最大 CAN 负载: >1000 帧/秒（无丢帧）

---

## 故障排查

### 问题 1: 丢帧率过高
**可能原因**:
- 队列长度不足
- 任务调度延迟过大

**解决方案**:
- 增加 `PROTOCOL_RX_QUEUE_LEN` (在 `manager.c` 中)
- 提高通信任务优先级
- 降低其他任务的 CPU 占用

### 问题 2: 电流环性能退化
**可能原因**:
- 其他高优先级中断抢占
- DMA 冲突

**解决方案**:
- 检查中断优先级配置（电流环应为最高优先级）
- 优化 DMA 通道分配

### 问题 3: 队列溢出频繁
**可能原因**:
- 通信任务周期过长（2ms 可能过慢）
- 协议处理耗时过长

**解决方案**:
- 缩短任务周期至 1ms 或 0.5ms
- 优化协议解析和执行逻辑

---

## 测试报告模板

| 测试项 | 指标 | 目标值 | 实测值 | 通过 |
|--------|------|--------|--------|------|
| CAN 高负载 | 丢帧率 | <0.1% | ___ | ☐ |
| CAN 高负载 | 队列峰值 | <20 | ___ | ☐ |
| 电流环实时性 | Jitter | <2us | ___ | ☐ |
| 电流环响应 | 延迟 | <100us | ___ | ☐ |
| Flash 写入 | ISR 调用 | 0 次 | ___ | ☐ |
| 溢出恢复 | 恢复时间 | <1s | ___ | ☐ |

---

## 自动化测试建议

### 集成到 CI/CD
在 `.github/workflows/vectorfoc-ci.yml` 中添加硬件在环测试（HIL）步骤：

```yaml
- name: Hardware-in-Loop CAN Test
  run: |
    # 烧录固件
    openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
      -c "program build/VectorFoc.hex verify reset exit"
    # 运行自动化测试脚本
    python tests/can_load_test.py --duration 60
```

### 持续监控
在生产环境中周期性记录统计并上传到监控系统。

---

## 参考文档
- [AGENTS.md](../../AGENTS.md) - 项目架构与已知问题
- [bsp_can.c](../../BSP/can/bsp_can.c) - CAN 底层驱动
- [manager.c](manager.c) - 协议管理器实现
