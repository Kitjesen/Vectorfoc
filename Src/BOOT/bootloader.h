/**
 * @file bootloader.h
 * @brief OTA Bootloader 主逻辑接口
 */
#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include "boot_config.h"
#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * Bootloader 状态
 * ============================================================================ */
typedef enum {
    BOOT_MODE_CHECK,            /* 检查模式 */
    BOOT_MODE_UPGRADE,          /* 升级模式 */
    BOOT_MODE_JUMP_APP,         /* 跳转到 App */
} BootMode_t;

/* ============================================================================
 * API
 * ============================================================================ */

/**
 * @brief Bootloader 主入口
 * @note 此函数不返回 (跳转到 App 或进入升级模式)
 */
void Boot_Main(void);

/**
 * @brief 检查 App 有效性
 * @return true App 有效
 */
bool Boot_CheckAppValid(void);

/**
 * @brief 检查升级标志
 * @return true 有升级请求
 */
bool Boot_CheckUpgradeFlag(void);

/**
 * @brief 设置升级标志
 */
void Boot_SetUpgradeFlag(void);

/**
 * @brief 清除升级标志
 */
void Boot_ClearUpgradeFlag(void);

/**
 * @brief 检查强制进入按键
 * @return true 按键按下
 */
bool Boot_CheckForceButton(void);

/**
 * @brief 跳转到 App
 * @note 此函数不返回
 */
void Boot_JumpToApp(void);

/**
 * @brief 进入升级模式
 * @note 此函数不返回
 */
void Boot_EnterUpgradeMode(void);

/**
 * @brief 获取 App Header
 * @return App Header 指针 (可能无效)
 */
const AppHeader_t* Boot_GetAppHeader(void);

/* ============================================================================
 * App 端调用的函数 (用于请求升级)
 * ============================================================================ */

/**
 * @brief 请求进入 Bootloader (App 端调用)
 * @note 设置升级标志后执行软复位
 */
void Boot_RequestUpgrade(void);

#endif /* BOOTLOADER_H */
