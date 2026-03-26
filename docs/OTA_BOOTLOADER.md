# VectorFOC OTA Bootloader

## 概述

VectorFOC 支持通过 USB-CDC 进行 OTA (Over-The-Air) 固件升级。系统包含两个独立的固件：

- **Bootloader**: 位于 Flash 起始位置，负责固件升级和 App 启动
- **Application**: 主应用程序，包含 FOC 控制逻辑

## Flash 布局

```
STM32G4 (256KB Flash)
┌─────────────────────────────────────────────────────────┐
│ 0x08000000 - 0x08003FFF │ Bootloader (16KB, 8 pages)    │
├─────────────────────────────────────────────────────────┤
│ 0x08004000 - 0x0803BFFF │ Application (224KB, 112 pages)│
├─────────────────────────────────────────────────────────┤
│ 0x0803C000 - 0x0803FFFF │ Config/Params (16KB, 8 pages) │
└─────────────────────────────────────────────────────────┘
```

## 启动流程

```
┌──────────────┐
│   上电复位    │
└──────┬───────┘
       ▼
┌──────────────┐     是
│ 按键按下？    │────────────┐
└──────┬───────┘            │
       │ 否                  │
       ▼                     │
┌──────────────┐     是      │
│ 升级标志？    │────────────┤
└──────┬───────┘            │
       │ 否                  │
       ▼                     │
┌──────────────┐     否      │
│ App 有效？    │────────────┤
└──────┬───────┘            │
       │ 是                  ▼
       ▼              ┌──────────────┐
┌──────────────┐      │  升级模式    │
│  跳转到 App   │      │ (USB-CDC)   │
└──────────────┘      └──────────────┘
```

## 升级协议

基于现有 VOFA+ 文本协议扩展，命令格式：`cmd=xxx\n` 或 `cmd,arg1,arg2\n`

### 命令列表

| 命令 | 方向 | 说明 |
|------|------|------|
| `boot_enter` | Host→Device | 请求进入 Bootloader (App 端处理) |
| `boot_ready` | Device→Host | Bootloader 就绪 |
| `boot_erase` | Host→Device | 擦除 App 区域 |
| `boot_write,addr,len` | Host→Device | 写入数据 (后跟二进制数据) |
| `boot_verify,crc,size` | Host→Device | 校验 CRC |
| `boot_reboot` | Host→Device | 重启到 App |
| `boot_info` | Host→Device | 获取 Bootloader 信息 |
| `boot_ack,status` | Device→Host | 操作结果 (0=OK) |

### 状态码

| 值 | 含义 |
|----|------|
| 0 | BOOT_OK - 成功 |
| 1 | BOOT_ERR_INVALID_ADDR - 地址无效 |
| 2 | BOOT_ERR_ERASE_FAIL - 擦除失败 |
| 3 | BOOT_ERR_WRITE_FAIL - 写入失败 |
| 4 | BOOT_ERR_VERIFY_FAIL - 验证失败 |
| 5 | BOOT_ERR_CRC_MISMATCH - CRC 不匹配 |
| 6 | BOOT_ERR_TIMEOUT - 超时 |
| 7 | BOOT_ERR_INVALID_CMD - 无效命令 |
| 8 | BOOT_ERR_APP_INVALID - App 无效 |

## 使用方法

### 1. 编译 Bootloader

```bash
# 创建 Bootloader 构建目录
mkdir build_boot && cd build_boot

# 配置 (使用 Bootloader CMakeLists)
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake \
      -DCMAKE_BUILD_TYPE=Release \
      ..

# 编译
make -j4
```

### 2. 编译 Application

```bash
# 使用默认 CMakeLists (已配置为 App 模式)
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake ..
make -j4
```

### 3. 首次烧录

首次需要通过 SWD/JTAG 烧录 Bootloader 和 Application：

```bash
# 烧录 Bootloader (0x08000000)
st-flash write VectorFoc_Bootloader.bin 0x08000000

# 烧录 Application (0x08004000)
st-flash write VectorFoc.bin 0x08004000
```

### 4. OTA 升级

后续升级可通过 USB-CDC：

```bash
# 使用 Python 脚本
python scripts/ota_upload.py VectorFoc.bin --port COM3

# 或手动发送命令
# 1. 发送 "boot_enter" 进入 Bootloader
# 2. 等待 "boot_ready"
# 3. 发送 "boot_erase" 擦除
# 4. 循环发送 "boot_write,addr,len" + 数据
# 5. 发送 "boot_verify,crc" 校验
# 6. 发送 "boot_reboot" 重启
```

### 5. 强制进入 Bootloader

如果 App 损坏无法响应 `boot_enter`：

1. 按住指定按键 (默认 PB12)
2. 上电或复位
3. 设备将直接进入 Bootloader 模式

## 文件结构

```
Src/BOOT/
├── boot_config.h      # 配置 (地址、Magic Number 等)
├── bootloader.c       # Bootloader 主逻辑
├── bootloader.h
├── flash_ops.c        # Flash 读写操作
├── flash_ops.h
├── boot_protocol.c    # 升级协议解析
└── boot_protocol.h

Lib/
├── stm32g431xx_bootloader.ld  # Bootloader 链接脚本
└── stm32g431xx_app.ld         # Application 链接脚本

scripts/
└── ota_upload.py      # OTA 上传工具
```

## App Header

Application 在 Flash 中包含一个 Header 结构，用于 Bootloader 验证：

```c
typedef struct {
    uint32_t magic;         // 0x56464F43 ("VFOC")
    uint32_t version;       // (major<<16) | (minor<<8) | patch
    uint32_t size;          // App 大小
    uint32_t crc32;         // CRC32 校验
    uint32_t build_time;    // 构建时间戳
    uint32_t reserved[3];   // 保留
} AppHeader_t;
```

Header 位于 `0x08004200` (向量表后 512 字节)。

## 注意事项

1. **Bootloader 大小限制**: Bootloader 必须小于 16KB
2. **App 起始地址**: Application 必须从 0x08004000 开始
3. **向量表重映射**: App 启动时会重映射向量表到 0x08004000
4. **升级标志**: 使用 RAM 末尾 16 字节存储升级标志，复位不清除
5. **CRC 校验**: 使用 IEEE 802.3 CRC32 多项式

## 故障排除

### 无法进入 Bootloader
- 检查按键连接 (PB12)
- 确认 App 中已添加 `boot_enter` 命令处理

### 升级失败
- 检查 USB 连接
- 确认固件文件正确
- 查看返回的错误码

### App 无法启动
- 检查 App 链接脚本是否正确 (起始地址 0x08004000)
- 确认向量表位置正确
- 使用强制按键进入 Bootloader 重新升级
