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
 * @file app_header.c
 * @brief Application Header (用于 OTA 校验)
 *
 * 此文件定义 App Header，放置在 Flash 的固定位置 (0x08004200)
 * Bootloader 通过读取此 Header 来验证 App 的有效性
 */
#include "boot_config.h"
#include "version.h"

/* App Header 放置在 .app_header section */
__attribute__((section(".app_header"), used))
const AppHeader_t g_app_header = {
    .magic = APP_MAGIC_NUMBER,
    .version = (FW_VERSION_MAJOR << 16) | (FW_VERSION_MINOR << 8) | FW_VERSION_PATCH,
    .size = 0,          /* 由构建脚本填充 */
    .crc32 = 0,         /* 由构建脚本填充 */
    .build_time = 0,    /* 由构建脚本填充 */
    .reserved = {0, 0, 0}
};
