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
 * @file    test_ladrc.c
 * @brief   Unit tests for LADRC controller
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "ladrc.h"

#define EPSILON 1e-4f
#define ASSERT_FLOAT_NEAR(a, b, eps) assert(fabsf((a) - (b)) < (eps))

static int tests_passed = 0;

#define TEST(name) \
    printf("  Testing: %s ... ", #name); \
    test_##name(); \
    printf("PASS\n"); \
    tests_passed++;

void test_ladrc_init(void) {
    LADRC_State_t state;
    LADRC_Config_t config = {
        .omega_o = 300.0f,    // Observer bandwidth
        .omega_c = 100.0f,    // Controller bandwidth
        .b0 = 50.0f,          // Control gain
        .max_output = 10.0f   // Max Iq
    };
    
    LADRC_Init(&state, &config);
    
    // initialized is false until first Calc
    assert(state.initialized == false);
    assert(state.z1 == 0.0f);
    assert(state.z2 == 0.0f);
    // beta1 = 2 * omega_o = 600
    ASSERT_FLOAT_NEAR(state.beta1, 600.0f, 1.0f);
    // beta2 = omega_o^2 = 90000
    ASSERT_FLOAT_NEAR(state.beta2, 90000.0f, 100.0f);
    // kp = omega_c = 100
    ASSERT_FLOAT_NEAR(state.kp, 100.0f, 0.1f);
}

void test_ladrc_zero_error(void) {
    LADRC_State_t state;
    LADRC_Config_t config = {
        .omega_o = 300.0f,
        .omega_c = 100.0f,
        .b0 = 50.0f,
        .max_output = 10.0f
    };
    
    LADRC_Init(&state, &config);
    
    // Zero error should produce near-zero output (after settling)
    float output = 0.0f;
    for (int i = 0; i < 100; i++) {
        output = LADRC_Calc(&state, &config, 0.0f, 0.0f, output, 0.001f);
    }
    
    ASSERT_FLOAT_NEAR(output, 0.0f, 0.1f);
}

void test_ladrc_step_response(void) {
    LADRC_State_t state;
    LADRC_Config_t config = {
        .omega_o = 300.0f,
        .omega_c = 100.0f,
        .b0 = 50.0f,
        .max_output = 10.0f
    };
    
    LADRC_Init(&state, &config);
    
    // Step input: vel_ref = 1.0, vel_fdb = 0
    float output = 0.0f;
    float vel_fdb = 0.0f;
    
    // Simulate a few steps
    for (int i = 0; i < 10; i++) {
        output = LADRC_Calc(&state, &config, 1.0f, vel_fdb, output, 0.001f);
        // Simple plant model: vel changes based on output
        vel_fdb += output * config.b0 * 0.001f;
    }
    
    // Output should be positive (trying to increase velocity)
    assert(output > 0.0f);
    printf("(output=%.3f, vel=%.3f) ", output, vel_fdb);
}

void test_ladrc_output_limit(void) {
    LADRC_State_t state;
    LADRC_Config_t config = {
        .omega_o = 300.0f,
        .omega_c = 100.0f,
        .b0 = 50.0f,
        .max_output = 5.0f  // Low limit
    };
    
    LADRC_Init(&state, &config);
    
    // Large error should hit output limit
    float output = LADRC_Calc(&state, &config, 100.0f, 0.0f, 0.0f, 0.001f);
    
    assert(output <= config.max_output);
    assert(output >= -config.max_output);
}

void test_ladrc_disturbance_rejection(void) {
    LADRC_State_t state;
    LADRC_Config_t config = {
        .omega_o = 500.0f,   // High observer bandwidth
        .omega_c = 100.0f,
        .b0 = 50.0f,
        .max_output = 20.0f
    };
    
    LADRC_Init(&state, &config);
    
    float output = 0.0f;
    float vel_fdb = 0.0f;
    float disturbance = 0.5f;  // External disturbance
    
    // Run until settled
    for (int i = 0; i < 500; i++) {
        output = LADRC_Calc(&state, &config, 1.0f, vel_fdb, output, 0.001f);
        // Plant with disturbance
        vel_fdb += (output * config.b0 - disturbance) * 0.001f;
    }
    
    // Should track reference despite disturbance
    ASSERT_FLOAT_NEAR(vel_fdb, 1.0f, 0.2f);
    printf("(vel_fdb=%.3f) ", vel_fdb);
}

void test_ladrc_reset(void) {
    LADRC_State_t state;
    LADRC_Config_t config = {
        .omega_o = 300.0f,
        .omega_c = 100.0f,
        .b0 = 50.0f,
        .max_output = 10.0f
    };
    
    LADRC_Init(&state, &config);
    
    // Run some calculations
    LADRC_Calc(&state, &config, 1.0f, 0.0f, 0.0f, 0.001f);
    LADRC_Calc(&state, &config, 1.0f, 0.5f, 5.0f, 0.001f);
    
    // Reset
    LADRC_Reset(&state);
    
    assert(state.z1 == 0.0f);
    assert(state.z2 == 0.0f);
    assert(state.output == 0.0f);
}

int main(void) {
    printf("\n=== LADRC Controller Unit Tests ===\n\n");
    
    TEST(ladrc_init);
    TEST(ladrc_zero_error);
    TEST(ladrc_step_response);
    TEST(ladrc_output_limit);
    TEST(ladrc_disturbance_rejection);
    TEST(ladrc_reset);
    
    printf("\n=== Results: %d passed ===\n\n", tests_passed);
    
    return 0;
}
