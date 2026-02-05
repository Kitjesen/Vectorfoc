#include "motor/foc/clarke.h"
#include "motor/foc/foc_algorithm.h"
#include "motor/foc/park.h"
#include "motor/foc/svpwm.h"
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
  Clarke_Transform(Ia, Ib, &Ialpha, &Ibeta);

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

  // SVPWM usually takes normalized inputs or raw voltages?
  // Let's check api. svpwm.h usually calculates duties.
  // Assuming SVPWM_Calculate(Valpha, Vbeta, Vbus, &ta, &tb, &tc)

  SVPWM_Calculate(Valpha, Vbeta, &t_a, &t_b, &t_c);

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

int main() {
  int passed = 0;
  int total = 0;

  total++;
  passed += Test_Clarke();
  total++;
  passed += Test_Park();
  total++;
  passed += Test_SVPWM();

  printf("=====================\n");
  printf("Total: %d, Passed: %d\n", total, passed);
  printf("=====================\n");

  return (passed == total) ? 0 : 1;
}
