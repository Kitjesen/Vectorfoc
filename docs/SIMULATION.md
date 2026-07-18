# VectorFOC 仿真测试说明

本文档说明当前仓库可复现的主机侧仿真测试入口。它不需要 STM32 硬件。

## 已注册的仿真测试

`test_runner_integration` 是默认 CTest 套件中的闭环仿真测试，定义在 `test/test_foc_integration.c`。当前覆盖内容包括：

- 电流环阶跃响应；
- 速度环阶跃响应；
- 速度设定从 50 rad/s 切换到 20 rad/s 的跟踪场景；
- 负载扰动响应；
- 位置环响应。

该测试使用文件内的简化 PMSM 模型，不等同于硬件在台架上的性能保证。

## 运行测试

先配置和构建主机测试：

```bash
cmake -S test -B build-test
cmake --build build-test --parallel
```

只运行闭环仿真测试：

```bash
ctest --test-dir build-test -R test_runner_integration --output-on-failure
```

如果使用 Visual Studio 这类多配置生成器，需要指定配置：

```powershell
ctest --test-dir build-test -C Debug -R test_runner_integration --output-on-failure
```

## 分析速度设定切换结果

`test_runner_integration` 会在运行目录生成 `foc_setpoint_switch.csv`。在构建目录中运行：

```bash
python ../test/analyze_results.py foc_setpoint_switch.csv
```

脚本会读取 CSV 中的 `RefSpeed`/`ActualSpeed` 或 `RefVel`/`ActVel` 列，输出目标速度、最终速度、误差、超调量，并保存同名 PNG 图，例如 `foc_setpoint_switch.png`。

本轮核验命令（`<build-dir>` 替换为你的主机构建目录）：

```bash
ctest --test-dir <build-dir> -R test_runner_integration --output-on-failure
cd <build-dir>
python ../test/analyze_results.py foc_setpoint_switch.csv
```

本轮输出：

```text
Target Vel: 20.0 rad/s
Final Vel:  19.75 rad/s
Error:      0.2450 rad/s
Overshoot:  30.75 rad/s
Plot saved to foc_setpoint_switch.png
```

这些数值是当前实现和本机工具链下的一次核验结果，不应写成固定性能规格。

CSV/PNG 是构建目录中的可再生输出，不纳入版本库；本文件保留了生成命令与本次输出，便于在其他主机上复核。

## 未注册的仿真源文件

`test/test_foc_closed_loop.c` 与 `test/motor_plant.c` 也包含闭环仿真代码，但 `test_foc_closed_loop.c` 当前不是默认 CTest 注册目标。除非后续把它接入 `test/CMakeLists.txt` 并验证，否则文档和发布说明不应把它列为默认测试。
