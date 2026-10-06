// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file boot_config.h
 * @brief OTA Bootloader 配置 - Flash 布局与常量定义
 *
 * Flash Layout (STM32G4, 256KB):
 *   0x08000000 - 0x08003FFF : Bootloader (16KB, 8 pages)
 *   0x08004000 - 0x0803BFFF : Application (224KB, 112 pages)
 *   0x0803C000 - 0x0803FFFF : Config/Params (16KB, 8 pages)
 */
#ifndef BOOT_CONFIG_H
#define BOOT_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * Flash 地址定义
 * ============================================================================ */
#define BOOT_FLASH_BASE         0x08000000
#define BOOT_FLASH_PAGE_SIZE    2048        /* STM32G4: 2KB per page */

/* Bootloader 区域 (16KB = 8 pages) */
#define BOOT_ADDR_START         0x08000000
#define BOOT_ADDR_END           0x08003FFF
#define BOOT_SIZE               (16 * 1024)

/* Application 区域 (224KB = 112 pages) */
#define APP_ADDR_START          0x08004000
#define APP_ADDR_END            0x0803BFFF
#define APP_SIZE                (224 * 1024)
#define APP_PAGE_START          8           /* Page 8 */
#define APP_PAGE_COUNT          112

/* Config/Params 区域 (16KB = 8 pages) */
#define CONFIG_ADDR_START       0x0803C000
#define CONFIG_ADDR_END         0x0803FFFF
#define CONFIG_SIZE             (16 * 1024)

/* ============================================================================
 * App 有效性检查
 * ============================================================================ */
/* App Header 结构 (位于 APP_ADDR_START) */
#define APP_MAGIC_NUMBER        0x56464F43  /* "VFOC" in little-endian */
#define APP_HEADER_OFFSET       0x200       /* Vector table 后的偏移 (512 bytes) */
#define APP_HEADER_ADDR         (APP_ADDR_START + APP_HEADER_OFFSET)

/* App Header 结构体 */
typedef struct {
    uint32_t magic;             /* Magic number: APP_MAGIC_NUMBER */
    uint32_t version;           /* 版本号: (major<<16) | (minor<<8) | patch */
    uint32_t size;              /* App 大小 (不含 header) */
    uint32_t crc32;             /* App CRC32 (从 header 后开始计算) */
    uint32_t build_time;        /* 构建时间戳 */
    uint32_t reserved[3];       /* 保留 */
} AppHeader_t;

/* ============================================================================
 * 升级标志 (RAM 中，复位不清除)
 * ============================================================================ */
/* 使用 RAM 末尾的特殊区域存放升级标志 */
#define BOOT_FLAG_ADDR          0x20007FF0  /* RAM 末尾 16 bytes */
#define BOOT_FLAG_MAGIC         0x424F4F54  /* "BOOT" */
#define BOOT_FLAG_UPGRADE       0x55504752  /* "UPGR" - 请求升级 */
#define BOOT_FLAG_APP_VALID     0x56414C44  /* "VALD" - App 有效 */

/* 升级标志结构 */
typedef struct {
    uint32_t magic;             /* BOOT_FLAG_MAGIC */
    uint32_t request;           /* BOOT_FLAG_UPGRADE = 请求升级 */
    uint32_t reserved[2];
} BootFlag_t;

/* ============================================================================
 * 协议配置
 * ============================================================================ */
#define BOOT_PROTOCOL_TIMEOUT_MS    5000    /* 协议超时 */
#define BOOT_WRITE_BLOCK_SIZE       256     /* 单次写入块大小 */
#define BOOT_RX_BUFFER_SIZE         512     /* 接收缓冲区大小 */

/* ============================================================================
 * 按键配置 (强制进入 Bootloader)
 * ============================================================================ */
/* 根据实际硬件配置修改 */
#define BOOT_FORCE_GPIO_PORT        GPIOB
#define BOOT_FORCE_GPIO_PIN         GPIO_PIN_12
#define BOOT_FORCE_ACTIVE_LOW       1       /* 1=低电平有效, 0=高电平有效 */

/* ============================================================================
 * 状态码
 * ============================================================================ */
typedef enum {
    BOOT_OK = 0,
    BOOT_ERR_INVALID_ADDR,
    BOOT_ERR_ERASE_FAIL,
    BOOT_ERR_WRITE_FAIL,
    BOOT_ERR_VERIFY_FAIL,
    BOOT_ERR_CRC_MISMATCH,
    BOOT_ERR_TIMEOUT,
    BOOT_ERR_INVALID_CMD,
    BOOT_ERR_APP_INVALID,
} BootStatus_t;

#endif /* BOOT_CONFIG_H */
