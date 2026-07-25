#include "encoder_position_tracker.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);              \
      return 1;                                                                \
    }                                                                          \
  } while (0)

int main(void) {
  bool initialized = false;
  int32_t last_count = 0;
  int64_t shadow_count = 1234;
  const int32_t cpr = 16384;

  CHECK(EncoderPosition_Update(12288, cpr, &initialized, &last_count,
                               &shadow_count) == 0);
  CHECK(initialized);
  CHECK(last_count == 12288);
  CHECK(shadow_count == 0);

  CHECK(EncoderPosition_Update(16380, cpr, &initialized, &last_count,
                               &shadow_count) == 4092);
  CHECK(EncoderPosition_Update(4, cpr, &initialized, &last_count,
                               &shadow_count) == 8);
  CHECK(shadow_count == 4100);

  CHECK(EncoderPosition_Update(16380, cpr, &initialized, &last_count,
                               &shadow_count) == -8);
  CHECK(shadow_count == 4092);

  initialized = false;
  CHECK(EncoderPosition_Update(3000, cpr, &initialized, &last_count,
                               &shadow_count) == 0);
  CHECK(shadow_count == 0);

  puts("Encoder position tracker tests PASSED");
  return 0;
}
