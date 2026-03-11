#ifndef TEST_MOCK_VERSION_H
#define TEST_MOCK_VERSION_H

#include <stdint.h>

typedef struct {
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
  uint8_t dirty;
  char git_hash[8];
} FirmwareVersion_t;

static inline const FirmwareVersion_t *FW_GetVersion(void) {
  static const FirmwareVersion_t version = {
      .major = 9,
      .minor = 8,
      .patch = 7,
      .dirty = 1,
      .git_hash = "abcd123",
  };

  return &version;
}

#endif
