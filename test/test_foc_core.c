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

#include "foc/clarke.h"
#include "foc/foc_algorithm.h"
#include "foc/park.h"
#include "foc/svpwm.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>


// Simple Test Framework
#define ASSERT_NEAR(a, b, epsilon)                                             \
  if (fabs((a) - (b)) > (epsilon)) {                                           \
    printf("FAIL: %s l:%d | %.5f != %.5f\n", __func__, __LINE__, (float)(a),   \
           (float)(b));                                                        \
    return 0;                                                                  \
  }

#define TEST_PASS 1
#define TEST_FAIL 0

int Test_Clarke() {
  float Ia = 1.0f, Ib = -0.5f, Ic = -0.5f;
  float Ialpha, Ibeta;

  // Balanced 3-phase: 1, -0.5, -0.5 -> Alpha should be 1
  Clarke_Transform(Ia, Ib, Ic, &Ialpha, &Ibeta);

  ASSERT_NEAR(Ialpha, 1.0f, 0.001f);
  ASSERT_NEAR(Ibeta, 0.0f, 0.001f); // Beta should be 0 since Ib=Ic

  printf("Clarke Test: PASS\n");
  return TEST_PASS;
}

int Test_Park() {
  // 1. Align alpha with d-axis (theta = 0)
  float Ialpha = 1.0f, Ibeta = 0.0f, theta = 0.0f;
  float Id, Iq;

  Park_Transform(Ialpha, Ibeta, theta, &Id, &Iq);
  ASSERT_NEAR(Id, 1.0f, 0.001f);
  ASSERT_NEAR(Iq, 0.0f, 0.001f);

  // 2. Rotate 90 deg (theta = PI/2) -> Alpha aligns with -Q (Standard Park def?
  // OR d aligns with alpha?) Standard Park: d = alpha*cos + beta*sin, q =
  // -alpha*sin + beta*cos
  theta = 1.570796f;
  Park_Transform(Ialpha, Ibeta, theta, &Id, &Iq);

  // cos(90) = 0, sin(90) = 1
  // d = 0 + 0 = 0
  // q = -1 + 0 = -1
  ASSERT_NEAR(Id, 0.0f, 0.01f);
  ASSERT_NEAR(Iq, -1.0f, 0.01f);

  printf("Park Test: PASS\n");
  return TEST_PASS;
}

int Test_SVPWM() {
  float Valpha = 0.0f;
  float Vbeta = 1.0f; // Pure voltage in beta
  float v_bus = 12.0f;
  float t_a, t_b, t_c;

  SVPWM_Modulate(Valpha, Vbeta, v_bus, &t_a, &t_b, &t_c);

  // Just sanity check they are within 0-1
  if (t_a < 0.0f || t_a > 1.0f)
    return TEST_FAIL;
  if (t_b < 0.0f || t_b > 1.0f)
    return TEST_FAIL;
  if (t_c < 0.0f || t_c > 1.0f)
    return TEST_FAIL;

  printf("SVPWM Test: PASS\n");
  return TEST_PASS;
}

int Test_SVPWM_ReportsOvermodulation() {
  float t_a, t_b, t_c;
  int status = SVPWM_Modulate(20.0f, 0.0f, 12.0f, &t_a, &t_b, &t_c);

  if (status != SVPWM_STATUS_OVERMODULATION)
    return TEST_FAIL;
  if (t_a < 0.0f || t_a > 1.0f || t_b < 0.0f || t_b > 1.0f ||
      t_c < 0.0f || t_c > 1.0f)
    return TEST_FAIL;

  printf("SVPWM Overmodulation Status Test: PASS\n");
  return TEST_PASS;
}

int Test_SVPWM_InvalidBusIsNotOvermodulation() {
  float t_a, t_b, t_c;
  int status = SVPWM_Modulate(1.0f, 0.0f, 0.0f, &t_a, &t_b, &t_c);

  if (status != SVPWM_STATUS_INVALID_INPUT)
    return TEST_FAIL;
  ASSERT_NEAR(t_a, 0.5f, 0.0001f);
  ASSERT_NEAR(t_b, 0.5f, 0.0001f);
  ASSERT_NEAR(t_c, 0.5f, 0.0001f);

  printf("SVPWM Invalid Bus Status Test: PASS\n");
  return TEST_PASS;
}

int Test_SVPWM_LinearBoundaryIsNotScaled() {
  const float v_bus = 12.0f;
  const float magnitude = v_bus / sqrtf(3.0f);
  const float angle = 3.14159265358979323846f / 6.0f;
  float t_a, t_b, t_c;

  int status = SVPWM_Modulate(magnitude * cosf(angle),
                              magnitude * sinf(angle), v_bus,
                              &t_a, &t_b, &t_c);

  if (status != SVPWM_STATUS_OK)
    return TEST_FAIL;
  ASSERT_NEAR(t_a, 1.0f, 0.001f);
  ASSERT_NEAR(t_b, 0.5f, 0.001f);
  ASSERT_NEAR(t_c, 0.0f, 0.001f);

  printf("SVPWM Linear Boundary Test: PASS\n");
  return TEST_PASS;
}

int Test_FOC_UsesConfiguredVoltageLimit() {
  FOC_AlgorithmInput_t input = {0};
  FOC_AlgorithmConfig_t config = {0};
  FOC_AlgorithmState_t state = {0};
  FOC_AlgorithmOutput_t output = {0};

  input.enabled = true;
  input.Vbus = 48.0f;
  input.Iq_ref = 10.0f;

  config.Kp_current_d = 10.0f;
  config.Kp_current_q = 10.0f;
  config.Ts_current = 0.00005f;
  config.current_limit = 100.0f;
  config.voltage_limit = 2.0f;

  FOC_Algorithm_CurrentLoop(&input, &config, &state, &output);

  float voltage_magnitude = sqrtf(output.Vd * output.Vd + output.Vq * output.Vq);
  ASSERT_NEAR(voltage_magnitude, 2.0f, 0.001f);
  if (!output.voltage_saturated)
    return TEST_FAIL;

  printf("FOC Configured Voltage Limit Test: PASS\n");
  return TEST_PASS;
}

int main() {
  int passed = 0;
  int total = 0;

  total++;
  passed += Test_Clarke();
  total++;
  passed += Test_Park();
  total++;
  passed += Test_SVPWM();
  total++;
  passed += Test_SVPWM_ReportsOvermodulation();
  total++;
  passed += Test_SVPWM_InvalidBusIsNotOvermodulation();
  total++;
  passed += Test_SVPWM_LinearBoundaryIsNotScaled();
  total++;
  passed += Test_FOC_UsesConfiguredVoltageLimit();

  printf("=====================\n");
  printf("Total: %d, Passed: %d\n", total, passed);
  printf("=====================\n");

  return (passed == total) ? 0 : 1;
}
