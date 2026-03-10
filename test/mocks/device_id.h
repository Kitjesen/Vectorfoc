#ifndef TEST_MOCK_DEVICE_ID_H
#define TEST_MOCK_DEVICE_ID_H

#include <stdint.h>

typedef struct {
  uint32_t word0;
  uint32_t word1;
  uint32_t word2;
} DeviceUID_t;

static inline void DeviceID_GetUID(DeviceUID_t *uid) {
  uid->word0 = 0x11223344U;
  uid->word1 = 0x55667788U;
  uid->word2 = 0x99AABBCCU;
}

#endif
