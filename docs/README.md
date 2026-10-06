# 固件文档

当前工程入口是 [../README.md](../README.md)。所有构建命令均从 `firmware/` 根目录执行；文档目录不参与构建。

| 文档 | 用途 |
| --- | --- |
| [BUILD_GUIDE.md](BUILD_GUIDE.md) | 当前目录下的固件构建与 host 测试命令 |
| [UPSTREAM_COMPARISON.md](UPSTREAM_COMPARISON.md) | 当前工作区与 GitHub 主线的版本、结构和安全能力对照 |
| [OTA_BOOTLOADER.md](OTA_BOOTLOADER.md) | bootloader 入口与既有 OTA 设计说明 |
| [协议字段](../src/comm/PROTOCOL_CN.md) | 现有 CAN 通信字段 |
| [Vector 调试记录](../../docs/DEBUGGING.md) | 当前离线验证与未解决事项 |
| [历史记录](history/README.md) | 旧审计、设计、优化建议与变更记录 |
| [images/foc.png](images/foc.png) | 原 FOC 示意图 |

历史报告统一放在 `history/`，保留正文与历史结论；其中的旧路径、API、测试数量和完成标记不能作为当前验证依据。
