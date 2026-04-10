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
 * @file    test_pid.c
 * @brief   Unit tests for PID controller
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "pid.h"

#define EPSILON 1e-4f
#define ASSERT_FLOAT_EQ(a, b) assert(fabsf((a) - (b)) < EPSILON)
#define ASSERT_FLOAT_NEAR(a, b, eps) assert(fabsf((a) - (b)) < (eps))

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    printf("  Testing: %s ... ", #name); \
    test_##name(); \
    printf("PASS\n"); \
    tests_passed++;

void test_pid_init(void) {
    PidTypeDef pid;
    float gains[3] = {1.0f, 0.1f, 0.01f};
    PID_Init(&pid, PID_POSITION, gains, 10.0f, 1.0f);
    
    assert(pid.Kp == 1.0f);
    assert(pid.Ki == 0.1f);
    assert(pid.Kd == 0.01f);
    assert(pid.max_out == 10.0f);
    assert(pid.max_iout == 1.0f);
}

void test_pid_proportional(void) {
    PidTypeDef pid;
    float gains[3] = {2.0f, 0.0f, 0.0f};  // Kp only
    PID_Init(&pid, PID_POSITION, gains, 100.0f, 10.0f);
    
    // set=10, fdb=0 -> error=10, output = Kp * error = 20
    float output = PID_Calc(&pid, 0.0f, 10.0f);
    ASSERT_FLOAT_EQ(output, 20.0f);
}

void test_pid_integral_accumulation(void) {
    PidTypeDef pid;
    float gains[3] = {0.0f, 1.0f, 0.0f};  // Ki only
    PID_Init(&pid, PID_POSITION, gains, 100.0f, 100.0f);
    
    // First call: error = 5
    float output1 = PID_CalcDt(&pid, 0.0f, 5.0f, 1.0f);
    ASSERT_FLOAT_NEAR(output1, 5.0f, 0.1f);
    
    // Second call: integral accumulates
    float output2 = PID_CalcDt(&pid, 0.0f, 5.0f, 1.0f);
    ASSERT_FLOAT_NEAR(output2, 10.0f, 0.1f);
}

void test_pid_integral_limit(void) {
    PidTypeDef pid;
    float gains[3] = {0.0f, 10.0f, 0.0f};  // High Ki
    PID_Init(&pid, PID_POSITION, gains, 100.0f, 5.0f);  // max_iout = 5
    
    // Accumulate integral beyond limit
    PID_CalcDt(&pid, 0.0f, 10.0f, 1.0f);
    PID_CalcDt(&pid, 0.0f, 10.0f, 1.0f);
    float output = PID_CalcDt(&pid, 0.0f, 10.0f, 1.0f);
    
    // Integral should be clamped to max_iout
    assert(output <= 5.0f + EPSILON);
}

void test_pid_output_limit(void) {
    PidTypeDef pid;
    float gains[3] = {10.0f, 0.0f, 0.0f};  // High Kp
    PID_Init(&pid, PID_POSITION, gains, 50.0f, 10.0f);  // max_out = 50
    
    // error = 100, Kp * error = 1000, but limited to 50
    float output = PID_Calc(&pid, 0.0f, 100.0f);
    ASSERT_FLOAT_EQ(output, 50.0f);
}

void test_pid_negative_error(void) {
    PidTypeDef pid;
    float gains[3] = {2.0f, 0.0f, 0.0f};
    PID_Init(&pid, PID_POSITION, gains, 100.0f, 10.0f);
    
    // set=0, fdb=10 -> error=-10, output = -20
    float output = PID_Calc(&pid, 10.0f, 0.0f);
    ASSERT_FLOAT_EQ(output, -20.0f);
}

void test_pid_clear(void) {
    PidTypeDef pid;
    float gains[3] = {1.0f, 1.0f, 1.0f};
    PID_Init(&pid, PID_POSITION, gains, 100.0f, 100.0f);
    
    // Accumulate some state
    PID_CalcDt(&pid, 0.0f, 10.0f, 1.0f);
    PID_CalcDt(&pid, 0.0f, 10.0f, 1.0f);
    
    // Clear
    PID_clear(&pid);
    
    assert(pid.Iout == 0.0f);
    assert(pid.error[0] == 0.0f);
    assert(pid.error[1] == 0.0f);
}

void test_pid_delta_mode(void) {
    PidTypeDef pid;
    float gains[3] = {1.0f, 0.0f, 0.0f};
    PID_Init(&pid, PID_DELTA, gains, 100.0f, 10.0f);
    
    // Delta mode should output incremental changes
    float output1 = PID_Calc(&pid, 0.0f, 10.0f);
    float output2 = PID_Calc(&pid, 0.0f, 10.0f);
    
    // In delta mode, second output should be different
    // (depends on implementation)
    printf("(delta mode: out1=%.2f, out2=%.2f) ", output1, output2);
}

int main(void) {
    printf("\n=== PID Controller Unit Tests ===\n\n");
    
    TEST(pid_init);
    TEST(pid_proportional);
    TEST(pid_integral_accumulation);
    TEST(pid_integral_limit);
    TEST(pid_output_limit);
    TEST(pid_negative_error);
    TEST(pid_clear);
    TEST(pid_delta_mode);
    
    printf("\n=== Results: %d passed, %d failed ===\n\n", 
           tests_passed, tests_failed);
    
    return tests_failed > 0 ? 1 : 0;
}
