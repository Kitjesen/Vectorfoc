#include "encoder_position_tracker.h"

#include <assert.h>

static void first_sample_establishes_boot_relative_origin(void) {
  bool initialized = false;
  int32_t last_count = 0;
  int64_t shadow_count = 1234;

  assert(EncoderPosition_Update(12288, 16384, &initialized, &last_count,
                                &shadow_count) == 0);
  assert(initialized);
  assert(last_count == 12288);
  assert(shadow_count == 0);
}

static void forward_and_reverse_wrap_use_shortest_delta(void) {
  bool initialized = false;
  int32_t last_count = 0;
  int64_t shadow_count = 0;

  assert(EncoderPosition_Update(16380, 16384, &initialized, &last_count,
                                &shadow_count) == 0);
  assert(EncoderPosition_Update(4, 16384, &initialized, &last_count,
                                &shadow_count) == 8);
  assert(shadow_count == 8);
  assert(EncoderPosition_Update(16380, 16384, &initialized, &last_count,
                                &shadow_count) == -8);
  assert(shadow_count == 0);
}

int main(void) {
  first_sample_establishes_boot_relative_origin();
  forward_and_reverse_wrap_use_shortest_delta();
  return 0;
}
