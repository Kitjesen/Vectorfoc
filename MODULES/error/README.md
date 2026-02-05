# error 模块

提供统一的错误码定义与错误上报接口。

- `error_types.h`: 错误码与分级定义
- `error_manager.c/.h`: 错误上报、清除、域管理
- `error_config.h`: 配置项与开关

典型用法：`ERROR_REPORT(code, "msg")`
