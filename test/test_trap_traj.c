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

#include "trap_traj.h"
#include <math.h>
#include <stdio.h>

#define ASSERT_NEAR(a, b, epsilon)                                             \
  if (fabs((a) - (b)) > (epsilon)) {                                           \
    printf("FAIL: %s l:%d | %.5f != %.5f\n", __func__, __LINE__, (float)(a),   \
           (float)(b));                                                        \
    return 0;                                                                  \
  }

#define TEST_PASS 1
#define TEST_FAIL 0

int Test_Traj_Basic(void) {
  TrajTypeDef traj = {0};
  TRAJ_plan(&traj, 10.0f, 0.0f, 0.0f, 5.0f, 10.0f, 10.0f);

  TRAJ_eval(&traj, 0.0f);
  ASSERT_NEAR(traj.Y, 0.0f, 1e-4f);
  ASSERT_NEAR(traj.Yd, 0.0f, 1e-4f);

  TRAJ_eval(&traj, traj.Tf_);
  ASSERT_NEAR(traj.Y, 10.0f, 1e-3f);
  ASSERT_NEAR(traj.Yd, 0.0f, 1e-2f);

  printf("Trap Traj Basic: PASS\n");
  return TEST_PASS;
}

int main(void) {
  int pass = 1;
  pass &= Test_Traj_Basic();

  if (pass) {
    printf("All Trap Traj tests passed.\n");
    return 0;
  }

  printf("Trap Traj tests failed.\n");
  return 1;
}
