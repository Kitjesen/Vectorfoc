/**
 * @file boot_protocol.c
 * @brief OTA 升级协议实现
 */
#include "boot_protocol.h"
#include "flash_ops.h"
#include "bootloader.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

/* ============================================================================
 * 私有变量
 * ============================================================================ */
static ProtoContext_t s_ctx;
static uint8_t s_tx_buf[256];

/* ============================================================================
 * 发送函数
 * ============================================================================ */
void BootProto_SendResponse(const char *msg)
{
    if (msg == NULL) return;
    
    uint16_t len = (uint16_t)strlen(msg);
    if (len + 1 > sizeof(s_tx_buf)) return;
    
    memcpy(s_tx_buf, msg, len);
    s_tx_buf[len] = '\n';
    
    CDC_Transmit_FS(s_tx_buf, len + 1);
}

void BootProto_SendResponsef(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf((char *)s_tx_buf, sizeof(s_tx_buf) - 1, fmt, args);
    va_end(args);
    
    if (n > 0) {
        s_tx_buf[n] = '\n';
        CDC_Transmit_FS(s_tx_buf, n + 1);
    }
}

void BootProto_SendAck(BootStatus_t status)
{
    BootProto_SendResponsef("boot_ack,%d", (int)status);
}

void BootProto_SendReady(void)
{
    BootProto_SendResponse("boot_ready");
}

/* ============================================================================
 * 初始化与重置
 * ============================================================================ */
void BootProto_Init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = PROTO_STATE_IDLE;
}

void BootProto_Reset(void)
{
    s_ctx.state = PROTO_STATE_IDLE;
    s_ctx.rx_pos = 0;
    s_ctx.write_addr = 0;
    s_ctx.write_len = 0;
    s_ctx.received_len = 0;
}

/* ============================================================================
 * 命令解析
 * ============================================================================ */
static void handle_command(const char *cmd)
{
    s_ctx.last_activity = HAL_GetTick();
    
    /* boot_erase - 擦除 App 区域 */
    if (strstr(cmd, "boot_erase")) {
        Flash_Unlock();
        BootStatus_t status = Flash_EraseAppArea();
        Flash_Lock();
        BootProto_SendAck(status);
        return;
    }
    
    /* boot_write,addr,len - 准备接收数据 */
    if (strncmp(cmd, "boot_write,", 11) == 0) {
        uint32_t addr = 0, len = 0;
        if (sscanf(cmd + 11, "%lx,%lu", &addr, &len) == 2) {
            if (!Flash_IsAddrInAppArea(addr, len)) {
                BootProto_SendAck(BOOT_ERR_INVALID_ADDR);
                return;
            }
            
            s_ctx.write_addr = addr;
            s_ctx.write_len = len;
            s_ctx.received_len = 0;
            s_ctx.state = PROTO_STATE_WAIT_DATA;
            
            /* 发送 ACK 表示准备好接收数据 */
            BootProto_SendAck(BOOT_OK);
        } else {
            BootProto_SendAck(BOOT_ERR_INVALID_CMD);
        }
        return;
    }
    
    /* boot_verify,crc - 校验 CRC */
    if (strncmp(cmd, "boot_verify,", 12) == 0) {
        uint32_t expected_crc = 0;
        uint32_t size = 0;
        
        if (sscanf(cmd + 12, "%lx,%lu", &expected_crc, &size) >= 1) {
            /* 如果没有提供 size，使用 App Header 中的 size */
            if (size == 0) {
                AppHeader_t *header = (AppHeader_t *)APP_HEADER_ADDR;
                if (header->magic == APP_MAGIC_NUMBER) {
                    size = header->size;
                }
            }
            
            if (size > 0) {
                uint32_t calc_crc = Flash_CalcFlashCRC32(
                    APP_ADDR_START + APP_HEADER_OFFSET + sizeof(AppHeader_t), 
                    size);
                
                if (calc_crc == expected_crc) {
                    BootProto_SendAck(BOOT_OK);
                } else {
                    BootProto_SendResponsef("boot_ack,%d,calc=%08lx,exp=%08lx",
                        BOOT_ERR_CRC_MISMATCH, calc_crc, expected_crc);
                }
            } else {
                BootProto_SendAck(BOOT_ERR_INVALID_CMD);
            }
        } else {
            BootProto_SendAck(BOOT_ERR_INVALID_CMD);
        }
        return;
    }
    
    /* boot_reboot - 重启到 App */
    if (strstr(cmd, "boot_reboot")) {
        BootProto_SendResponse("boot_ack,0,rebooting");
        HAL_Delay(100);  /* 等待发送完成 */
        Boot_JumpToApp();
        return;
    }
    
    /* boot_info - 获取 Bootloader 信息 */
    if (strstr(cmd, "boot_info")) {
        BootProto_SendResponsef("boot_info,app_start=%08lx,app_size=%lu",
            APP_ADDR_START, (uint32_t)APP_SIZE);
        return;
    }
    
    /* 未知命令 */
    BootProto_SendAck(BOOT_ERR_INVALID_CMD);
}

/* ============================================================================
 * 数据处理
 * ============================================================================ */
void BootProto_ProcessData(const uint8_t *data, uint16_t len)
{
    s_ctx.last_activity = HAL_GetTick();
    
    if (s_ctx.state == PROTO_STATE_WAIT_DATA) {
        /* 接收二进制数据 */
        uint32_t remaining = s_ctx.write_len - s_ctx.received_len;
        uint32_t to_copy = (len < remaining) ? len : remaining;
        
        /* 写入 Flash */
        Flash_Unlock();
        
        /* 确保 8 字节对齐写入 */
        uint32_t write_addr = s_ctx.write_addr + s_ctx.received_len;
        BootStatus_t status = Flash_WriteData(write_addr, data, to_copy);
        
        Flash_Lock();
        
        if (status != BOOT_OK) {
            BootProto_SendAck(status);
            BootProto_Reset();
            return;
        }
        
        s_ctx.received_len += to_copy;
        
        /* 检查是否接收完成 */
        if (s_ctx.received_len >= s_ctx.write_len) {
            BootProto_SendAck(BOOT_OK);
            s_ctx.state = PROTO_STATE_IDLE;
        }
        
        return;
    }
    
    /* 文本命令模式 - 按行解析 */
    for (uint16_t i = 0; i < len; i++) {
        uint8_t c = data[i];
        
        if (c == '\n' || c == '\r') {
            if (s_ctx.rx_pos > 0) {
                s_ctx.rx_buf[s_ctx.rx_pos] = '\0';
                handle_command((const char *)s_ctx.rx_buf);
                s_ctx.rx_pos = 0;
            }
        } else {
            if (s_ctx.rx_pos < sizeof(s_ctx.rx_buf) - 1) {
                s_ctx.rx_buf[s_ctx.rx_pos++] = c;
            }
        }
    }
}

/* ============================================================================
 * 超时检查
 * ============================================================================ */
bool BootProto_CheckTimeout(uint32_t current_tick)
{
    if (s_ctx.state == PROTO_STATE_WAIT_DATA) {
        if (current_tick - s_ctx.last_activity > BOOT_PROTOCOL_TIMEOUT_MS) {
            BootProto_Reset();
            return true;
        }
    }
    return false;
}
