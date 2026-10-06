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
 * @file bootloader.c
 * @brief OTA Bootloader 主逻辑实现
 *
 * 启动流程:
 *   1. 检查强制进入按键 → 按下则进入升级模式
 *   2. 检查升级标志 (RAM) → 有则进入升级模式
 *   3. 检查 App 有效性 (Magic + CRC) → 无效则进入升级模式
 *   4. 跳转到 App
 */
#include "bootloader.h"
#include "boot_protocol.h"
#include "flash_ops.h"
#include "stm32g4xx_hal.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* ============================================================================
 * 升级标志 (位于 RAM 末尾，复位不清除)
 * ============================================================================ */
/* 使用 __attribute__((section)) 将标志放在特定位置 */
static volatile BootFlag_t __attribute__((section(".noinit"))) s_boot_flag;

/* ============================================================================
 * App 有效性检查
 * ============================================================================ */
bool Boot_CheckAppValid(void)
{
    /* 检查 App 起始地址的栈指针是否有效 (STM32G431: 32KB RAM) */
    uint32_t app_stack = *(volatile uint32_t *)APP_ADDR_START;
    if (app_stack < 0x20000000 || app_stack > 0x20008000) {
        return false;
    }
    
    /* 检查 Reset Handler 地址是否有效 */
    uint32_t app_reset = *(volatile uint32_t *)(APP_ADDR_START + 4);
    if (app_reset < APP_ADDR_START || app_reset > APP_ADDR_END) {
        return false;
    }
    
    /* 检查 App Header */
    const AppHeader_t *header = Boot_GetAppHeader();
    if (header->magic != APP_MAGIC_NUMBER) {
        /* Magic 不匹配，可能是旧固件，仍然允许启动 */
        /* 但不进行 CRC 校验 */
        return true;
    }
    
    /* 有 Header，进行 CRC 校验 */
    if (header->size == 0 || header->size > APP_SIZE) {
        return false;
    }
    
    uint32_t calc_crc = Flash_CalcFlashCRC32(
        APP_HEADER_ADDR + sizeof(AppHeader_t),
        header->size);
    
    if (calc_crc != header->crc32) {
        return false;
    }
    
    return true;
}

/* ============================================================================
 * 升级标志操作
 * ============================================================================ */
bool Boot_CheckUpgradeFlag(void)
{
    if (s_boot_flag.magic == BOOT_FLAG_MAGIC &&
        s_boot_flag.request == BOOT_FLAG_UPGRADE) {
        return true;
    }
    return false;
}

void Boot_SetUpgradeFlag(void)
{
    s_boot_flag.magic = BOOT_FLAG_MAGIC;
    s_boot_flag.request = BOOT_FLAG_UPGRADE;
}

void Boot_ClearUpgradeFlag(void)
{
    s_boot_flag.magic = 0;
    s_boot_flag.request = 0;
}

/* ============================================================================
 * 强制进入按键检查
 * ============================================================================ */
bool Boot_CheckForceButton(void)
{
#ifdef BOOT_FORCE_GPIO_PORT
    /* 初始化 GPIO (如果还没初始化) */
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin = BOOT_FORCE_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = BOOT_FORCE_ACTIVE_LOW ? GPIO_PULLUP : GPIO_PULLDOWN;
    
    /* 使能 GPIO 时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_Init(BOOT_FORCE_GPIO_PORT, &gpio_init);
    
    /* 读取按键状态 */
    GPIO_PinState state = HAL_GPIO_ReadPin(BOOT_FORCE_GPIO_PORT, BOOT_FORCE_GPIO_PIN);
    
    if (BOOT_FORCE_ACTIVE_LOW) {
        return (state == GPIO_PIN_RESET);
    } else {
        return (state == GPIO_PIN_SET);
    }
#else
    return false;
#endif
}

/* ============================================================================
 * 跳转到 App
 * ============================================================================ */
typedef void (*pFunction)(void);

void Boot_JumpToApp(void)
{
    /* 清除升级标志 */
    Boot_ClearUpgradeFlag();
    
    /* 获取 App 的栈指针和 Reset Handler */
    uint32_t app_stack = *(volatile uint32_t *)APP_ADDR_START;
    uint32_t app_reset = *(volatile uint32_t *)(APP_ADDR_START + 4);
    
    /* 关闭所有中断 */
    __disable_irq();
    
    /* 关闭 SysTick */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    
    /* 清除所有挂起的中断 */
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
    
    /* 重新映射向量表到 App */
    SCB->VTOR = APP_ADDR_START;
    
    /* 设置栈指针 */
    __set_MSP(app_stack);
    
    /* 使能中断 */
    __enable_irq();
    
    /* 跳转到 App */
    pFunction jump_to_app = (pFunction)app_reset;
    jump_to_app();
    
    /* 不应该到达这里 */
    while (1);
}

/* ============================================================================
 * 升级模式
 * ============================================================================ */
void Boot_EnterUpgradeMode(void)
{
    /* 初始化协议 */
    BootProto_Init();
    
    /* 发送就绪消息 */
    BootProto_SendReady();
    
    /* 主循环 */
    while (1) {
        /* USB CDC 数据在中断中处理 */
        /* 这里只需要检查超时 */
        if (BootProto_CheckTimeout(HAL_GetTick())) {
            BootProto_SendResponse("boot_timeout");
        }
        
        HAL_Delay(10);
    }
}

/* ============================================================================
 * Bootloader 主入口
 * ============================================================================ */
void Boot_Main(void)
{
    /* 1. 检查强制进入按键 */
    if (Boot_CheckForceButton()) {
        Boot_EnterUpgradeMode();
        /* 不返回 */
    }
    
    /* 2. 检查升级标志 */
    if (Boot_CheckUpgradeFlag()) {
        Boot_ClearUpgradeFlag();
        Boot_EnterUpgradeMode();
        /* 不返回 */
    }
    
    /* 3. 检查 App 有效性 */
    if (!Boot_CheckAppValid()) {
        Boot_EnterUpgradeMode();
        /* 不返回 */
    }
    
    /* 4. 跳转到 App */
    Boot_JumpToApp();
    
    /* 不应该到达这里 */
    while (1);
}

/* ============================================================================
 * App Header 获取
 * ============================================================================ */
const AppHeader_t* Boot_GetAppHeader(void)
{
    return (const AppHeader_t *)APP_HEADER_ADDR;
}

/* ============================================================================
 * App 端调用: 请求升级
 * ============================================================================ */
void Boot_RequestUpgrade(void)
{
    Boot_SetUpgradeFlag();
    
    /* 执行软复位 */
    __disable_irq();
    NVIC_SystemReset();
    
    /* 不应该到达这里 */
    while (1);
}
