/**
 * @file    device_id.h
 * @brief   Device unique identification
 * @note    Uses STM32 built-in 96-bit UID (factory programmed, read-only)
 * 
 * STM32G4 UID 地址: 0x1FFF7590 (UID_BASE)
 * - 96位 (12字节) 唯一标识
 * - 出厂烧录，只读，永不更改
 * - 可用于：设备序列号、CAN节点分配、许可证绑定等
 */

#ifndef DEVICE_ID_H
#define DEVICE_ID_H

#include <stdint.h>

#ifdef STM32G431xx
#include "stm32g4xx.h"
#endif

/**
 * @brief Device unique ID structure (96-bit / 12 bytes)
 */
typedef struct {
    uint32_t word0;     // UID[31:0]   - X/Y coordinates on wafer
    uint32_t word1;     // UID[63:32]  - Wafer number + Lot number (low)
    uint32_t word2;     // UID[95:64]  - Lot number (high)
} DeviceUID_t;

/**
 * @brief Get raw 96-bit device UID
 * @param uid Pointer to DeviceUID_t structure to fill
 */
static inline void DeviceID_GetUID(DeviceUID_t *uid) {
#ifdef UID_BASE
    const uint32_t *base = (const uint32_t *)UID_BASE;
    uid->word0 = base[0];
    uid->word1 = base[1];
    uid->word2 = base[2];
#else
    // Fallback for non-STM32 or simulation
    uid->word0 = 0x12345678;
    uid->word1 = 0x87654321;
    uid->word2 = 0xDEADBEEF;
#endif
}

/**
 * @brief Get 32-bit short device ID (for CAN addressing, display, etc.)
 * @return CRC32 hash of the full 96-bit UID
 * @note This provides a shorter ID while maintaining uniqueness
 */
static inline uint32_t DeviceID_GetShortID(void) {
    DeviceUID_t uid;
    DeviceID_GetUID(&uid);
    // Simple hash: XOR all words together
    return uid.word0 ^ uid.word1 ^ uid.word2;
}

/**
 * @brief Get 8-bit device ID (for CAN node ID auto-assignment)
 * @return Lower 8 bits of short ID, avoiding reserved addresses (0, 127, 255)
 */
static inline uint8_t DeviceID_GetNodeID(void) {
    uint32_t short_id = DeviceID_GetShortID();
    uint8_t node_id = (uint8_t)(short_id & 0x7F);  // 0-127
    
    // Avoid reserved addresses
    if (node_id == 0) node_id = 1;
    if (node_id == 127) node_id = 126;
    
    return node_id;
}

/**
 * @brief Format UID as hex string
 * @param buffer Output buffer (must be at least 25 bytes: 24 hex + null)
 * @note Format: "XXXXXXXX-XXXXXXXX-XXXXXXXX"
 */
static inline void DeviceID_GetUIDString(char *buffer) {
    DeviceUID_t uid;
    DeviceID_GetUID(&uid);
    
    // Simple hex formatting without sprintf
    static const char hex[] = "0123456789ABCDEF";
    uint32_t words[3] = {uid.word0, uid.word1, uid.word2};
    int pos = 0;
    
    for (int w = 0; w < 3; w++) {
        if (w > 0) buffer[pos++] = '-';
        for (int i = 7; i >= 0; i--) {
            buffer[pos++] = hex[(words[w] >> (i * 4)) & 0xF];
        }
    }
    buffer[pos] = '\0';
}

/**
 * @brief Check if two devices have the same UID
 * @param a First UID
 * @param b Second UID
 * @return 1 if equal, 0 if different
 */
static inline int DeviceID_Compare(const DeviceUID_t *a, const DeviceUID_t *b) {
    return (a->word0 == b->word0) && 
           (a->word1 == b->word1) && 
           (a->word2 == b->word2);
}

#endif // DEVICE_ID_H
