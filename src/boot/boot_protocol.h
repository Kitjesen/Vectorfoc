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
 * @file boot_protocol.h
 * @brief OTA 升级协议接口
 *
 * 协议格式 (基于 VOFA+ 扩展):
 *   文本命令: "cmd=xxx\n" 或 "cmd=xxx,arg1,arg2\n"
 *
 * 命令列表:
 *   Host → Device:
 *     boot_enter      - 请求进入 Bootloader
 *     boot_erase      - 擦除 App 区域
 *     boot_write,addr,len - 写入数据 (后跟二进制数据)
 *     boot_verify,crc - 校验 CRC
 *     boot_reboot     - 重启到 App
 *
 *   Device → Host:
 *     boot_ready      - Bootloader 就绪
 *     boot_ack,status - 操作结果 (0=OK, 其他=错误码)
 */
#ifndef BOOT_PROTOCOL_H
#define BOOT_PROTOCOL_H

#include "boot_config.h"
#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * 协议状态
 * ============================================================================ */
typedef enum {
    PROTO_STATE_IDLE,           /* 等待命令 */
    PROTO_STATE_WAIT_DATA,      /* 等待二进制数据 */
} ProtoState_t;

/* ============================================================================
 * 协议上下文
 * ============================================================================ */
typedef struct {
    ProtoState_t state;
    uint32_t write_addr;        /* 当前写入地址 */
    uint32_t write_len;         /* 期望数据长度 */
    uint32_t received_len;      /* 已接收长度 */
    uint8_t rx_buf[BOOT_RX_BUFFER_SIZE];
    uint16_t rx_pos;
    uint32_t last_activity;     /* 最后活动时间 (用于超时) */
} ProtoContext_t;

/* ============================================================================
 * API
 * ============================================================================ */

/**
 * @brief 初始化协议处理
 */
void BootProto_Init(void);

/**
 * @brief 处理接收到的数据
 * @param data 数据指针
 * @param len 数据长度
 */
void BootProto_ProcessData(const uint8_t *data, uint16_t len);

/**
 * @brief 发送响应
 * @param msg 消息字符串
 */
void BootProto_SendResponse(const char *msg);

/**
 * @brief 发送格式化响应
 * @param fmt 格式字符串
 */
void BootProto_SendResponsef(const char *fmt, ...);

/**
 * @brief 发送 ACK
 * @param status 状态码
 */
void BootProto_SendAck(BootStatus_t status);

/**
 * @brief 发送 Bootloader 就绪消息
 */
void BootProto_SendReady(void);

/**
 * @brief 检查超时
 * @param current_tick 当前 tick
 * @return true 超时
 */
bool BootProto_CheckTimeout(uint32_t current_tick);

/**
 * @brief 重置协议状态
 */
void BootProto_Reset(void);

#endif /* BOOT_PROTOCOL_H */
