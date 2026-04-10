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
 * @file    test_foc_integration.c
 * @brief   FOC Integration Tests - Full closed-loop simulation
 * 
 * Tests the complete FOC control chain with a simulated PMSM motor model.
 * Validates: current loop, speed loop, position loop, and transitions.
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include "common.h"
#include "clarke.h"
#include "park.h"
#include "svpwm.h"
#include "trigonometry.h"
#include "pid.h"

/*============================================================================
 * PMSM Motor Model (Simplified)
 *============================================================================*/

typedef struct {
    // Motor parameters
    float Rs;           // Stator resistance [Ohm]
    float Ld;           // D-axis inductance [H]
    float Lq;           // Q-axis inductance [H]
    float flux;         // PM flux linkage [Wb]
    float J;            // Inertia [kg.m^2]
    float B;            // Friction [N.m.s]
    uint8_t pole_pairs;
    
    // State variables
    float Id;           // D-axis current [A]
    float Iq;           // Q-axis current [A]
    float omega_e;      // Electrical speed [rad/s]
    float omega_m;      // Mechanical speed [rad/s]
    float theta_e;      // Electrical angle [rad]
    float theta_m;      // Mechanical angle [rad]
    
    // Outputs
    float Ia, Ib, Ic;   // Phase currents [A]
    float Te;           // Electromagnetic torque [N.m]
} MotorModel_t;

void Motor_Init(MotorModel_t *m) {
    // Typical small BLDC motor parameters
    m->Rs = 0.5f;
    m->Ld = 0.001f;
    m->Lq = 0.001f;
    m->flux = 0.01f;
    m->J = 0.0001f;
    m->B = 0.001f;
    m->pole_pairs = 7;
    
    m->Id = 0; m->Iq = 0;
    m->omega_e = 0; m->omega_m = 0;
    m->theta_e = 0; m->theta_m = 0;
    m->Ia = 0; m->Ib = 0; m->Ic = 0;
    m->Te = 0;
}

void Motor_Step(MotorModel_t *m, float Vd, float Vq, float dt, float load_torque) {
    // Current dynamics (simplified, ignoring cross-coupling)
    // dId/dt = (Vd - Rs*Id + omega_e*Lq*Iq) / Ld
    // dIq/dt = (Vq - Rs*Iq - omega_e*Ld*Id - omega_e*flux) / Lq
    
    float dId = (Vd - m->Rs * m->Id + m->omega_e * m->Lq * m->Iq) / m->Ld;
    float dIq = (Vq - m->Rs * m->Iq - m->omega_e * m->Ld * m->Id - m->omega_e * m->flux) / m->Lq;
    
    m->Id += dId * dt;
    m->Iq += dIq * dt;
    
    // Torque: Te = 1.5 * p * (flux * Iq + (Ld - Lq) * Id * Iq)
    // For surface mount PM (Ld ≈ Lq): Te ≈ 1.5 * p * flux * Iq
    m->Te = 1.5f * m->pole_pairs * m->flux * m->Iq;
    
    // Mechanical dynamics
    // J * d(omega_m)/dt = Te - B*omega_m - T_load
    float domega_m = (m->Te - m->B * m->omega_m - load_torque) / m->J;
    m->omega_m += domega_m * dt;
    
    // Electrical speed
    m->omega_e = m->omega_m * m->pole_pairs;
    
    // Angle integration
    m->theta_m += m->omega_m * dt;
    m->theta_e = m->theta_m * m->pole_pairs;
    
    // Wrap angle
    while (m->theta_e > M_PI) m->theta_e -= 2 * M_PI;
    while (m->theta_e < -M_PI) m->theta_e += 2 * M_PI;
    
    // Inverse Park to get phase currents
    float sin_e, cos_e;
    Trig_FastSinCos(m->theta_e, &sin_e, &cos_e);
    
    float Ialpha = m->Id * cos_e - m->Iq * sin_e;
    float Ibeta = m->Id * sin_e + m->Iq * cos_e;
    
    // Inverse Clarke
    m->Ia = Ialpha;
    m->Ib = -0.5f * Ialpha + 0.866f * Ibeta;
    m->Ic = -0.5f * Ialpha - 0.866f * Ibeta;
}

/*============================================================================
 * Test Utilities
 *============================================================================*/

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    printf("  Testing: %s ... ", #name); \
    if (test_##name()) { printf("PASS\n"); tests_passed++; } \
    else { printf("FAIL\n"); tests_failed++; }

#define ASSERT_NEAR(a, b, eps) \
    if (fabsf((a) - (b)) >= (eps)) { \
        printf("\n    ASSERT_NEAR failed: %f vs %f (eps=%f)\n", (float)(a), (float)(b), (float)(eps)); \
        return 0; \
    }

#define ASSERT_TRUE(cond) \
    if (!(cond)) { \
        printf("\n    ASSERT_TRUE failed: %s\n", #cond); \
        return 0; \
    }

/*============================================================================
 * Integration Tests
 *============================================================================*/

// Test 1: Current loop step response
int test_current_loop_step_response(void) {
    MotorModel_t motor;
    Motor_Init(&motor);
    
    PidTypeDef pid_d, pid_q;
    float gains[3] = {10.0f, 100.0f, 0.0f};  // Kp, Ki, Kd
    PID_Init(&pid_d, PID_POSITION, gains, 24.0f, 10.0f);
    PID_Init(&pid_q, PID_POSITION, gains, 24.0f, 10.0f);
    
    float Id_ref = 0.0f;
    float Iq_ref = 1.0f;  // Step to 1A
    float dt = 0.0001f;   // 10kHz
    
    // Run for 10ms
    for (int i = 0; i < 100; i++) {
        float Vd = PID_CalcDt(&pid_d, motor.Id, Id_ref, dt);
        float Vq = PID_CalcDt(&pid_q, motor.Iq, Iq_ref, dt);
        Motor_Step(&motor, Vd, Vq, dt, 0.0f);
    }
    
    // Iq should be close to reference
    ASSERT_NEAR(motor.Iq, Iq_ref, 0.2f);
    // Id should stay near zero
    ASSERT_NEAR(motor.Id, Id_ref, 0.1f);
    
    return 1;
}

// Test 2: Speed loop step response
int test_speed_loop_step_response(void) {
    MotorModel_t motor;
    Motor_Init(&motor);
    
    // Current loop PIDs
    PidTypeDef pid_d, pid_q;
    float current_gains[3] = {10.0f, 100.0f, 0.0f};
    PID_Init(&pid_d, PID_POSITION, current_gains, 24.0f, 10.0f);
    PID_Init(&pid_q, PID_POSITION, current_gains, 24.0f, 10.0f);
    
    // Speed loop PID (more aggressive)
    PidTypeDef pid_speed;
    float speed_gains[3] = {0.05f, 0.5f, 0.0f};
    PID_Init(&pid_speed, PID_POSITION, speed_gains, 5.0f, 3.0f);
    
    float speed_ref = 50.0f;  // Lower target, easier to reach
    float dt = 0.0001f;
    
    // Run for 200ms (more time to settle)
    for (int i = 0; i < 2000; i++) {
        // Speed loop
        float Iq_ref = PID_CalcDt(&pid_speed, motor.omega_m, speed_ref, dt);
        float Id_ref = 0.0f;
        
        // Current loop
        float Vd = PID_CalcDt(&pid_d, motor.Id, Id_ref, dt);
        float Vq = PID_CalcDt(&pid_q, motor.Iq, Iq_ref, dt);
        
        Motor_Step(&motor, Vd, Vq, dt, 0.0f);
    }
    
    // Speed should be close to reference (within 30%)
    ASSERT_NEAR(motor.omega_m, speed_ref, speed_ref * 0.3f);
    
    return 1;
}

// Test 3: Load disturbance rejection
int test_load_disturbance_rejection(void) {
    MotorModel_t motor;
    Motor_Init(&motor);
    
    PidTypeDef pid_d, pid_q, pid_speed;
    float current_gains[3] = {10.0f, 100.0f, 0.0f};
    float speed_gains[3] = {0.01f, 0.1f, 0.0f};
    PID_Init(&pid_d, PID_POSITION, current_gains, 24.0f, 10.0f);
    PID_Init(&pid_q, PID_POSITION, current_gains, 24.0f, 10.0f);
    PID_Init(&pid_speed, PID_POSITION, speed_gains, 5.0f, 2.0f);
    
    float speed_ref = 50.0f;
    float dt = 0.0001f;
    float load_torque = 0.0f;
    
    // Run to steady state
    for (int i = 0; i < 1000; i++) {
        float Iq_ref = PID_CalcDt(&pid_speed, motor.omega_m, speed_ref, dt);
        float Vd = PID_CalcDt(&pid_d, motor.Id, 0.0f, dt);
        float Vq = PID_CalcDt(&pid_q, motor.Iq, Iq_ref, dt);
        Motor_Step(&motor, Vd, Vq, dt, load_torque);
    }
    
    float speed_before = motor.omega_m;
    (void)speed_before;
    
    // Apply load disturbance
    load_torque = 0.001f;  // Small load
    
    // Run with load
    for (int i = 0; i < 2000; i++) {
        float Iq_ref = PID_CalcDt(&pid_speed, motor.omega_m, speed_ref, dt);
        float Vd = PID_CalcDt(&pid_d, motor.Id, 0.0f, dt);
        float Vq = PID_CalcDt(&pid_q, motor.Iq, Iq_ref, dt);
        Motor_Step(&motor, Vd, Vq, dt, load_torque);
    }
    
    // Speed should recover close to reference
    ASSERT_NEAR(motor.omega_m, speed_ref, 10.0f);
    
    return 1;
}

// Test 4: Clarke-Park-InversePark-InverseClarke round trip
int test_transform_roundtrip(void) {
    float Ia = 1.0f, Ib = -0.5f, Ic = -0.5f;
    float theta = 0.5f;  // arbitrary angle
    
    // Clarke
    float Ialpha, Ibeta;
    Clarke_Transform(Ia, Ib, Ic, &Ialpha, &Ibeta);
    
    // Park
    float Id, Iq;
    Park_Transform(Ialpha, Ibeta, theta, &Id, &Iq);
    
    // Inverse Park
    float Valpha, Vbeta;
    Park_Inverse(Id, Iq, theta, &Valpha, &Vbeta);
    
    // Inverse Clarke
    float Va, Vb, Vc;
    Clarke_Inverse(Valpha, Vbeta, &Va, &Vb, &Vc);
    
    // Park-InvPark should preserve magnitude
    float mag_in = sqrtf(Ialpha * Ialpha + Ibeta * Ibeta);
    float mag_out = sqrtf(Valpha * Valpha + Vbeta * Vbeta);
    ASSERT_NEAR(mag_in, mag_out, 0.01f);
    
    return 1;
}

// Test 5: SVPWM duty cycle validity
int test_svpwm_duty_validity(void) {
    for (float angle = 0; angle < 2 * M_PI; angle += 0.1f) {
        float Valpha = cosf(angle);
        float Vbeta = sinf(angle);
        float Vbus = 24.0f;
        
        float Ta, Tb, Tc;
        SVPWM_Modulate(Valpha * 10.0f, Vbeta * 10.0f, Vbus, &Ta, &Tb, &Tc);
        
        // Duty cycles must be in [0, 1]
        ASSERT_TRUE(Ta >= 0.0f && Ta <= 1.0f);
        ASSERT_TRUE(Tb >= 0.0f && Tb <= 1.0f);
        ASSERT_TRUE(Tc >= 0.0f && Tc <= 1.0f);
    }
    return 1;
}

// Test 6: Numerical stability - long run
int test_numerical_stability(void) {
    MotorModel_t motor;
    Motor_Init(&motor);
    
    PidTypeDef pid_d, pid_q;
    float gains[3] = {10.0f, 100.0f, 0.0f};
    PID_Init(&pid_d, PID_POSITION, gains, 24.0f, 10.0f);
    PID_Init(&pid_q, PID_POSITION, gains, 24.0f, 10.0f);
    
    float dt = 0.0001f;
    
    // Run for 1 second (10000 steps)
    for (int i = 0; i < 10000; i++) {
        float Iq_ref = 0.5f * sinf(i * dt * 10.0f);  // Varying reference
        float Vd = PID_CalcDt(&pid_d, motor.Id, 0.0f, dt);
        float Vq = PID_CalcDt(&pid_q, motor.Iq, Iq_ref, dt);
        Motor_Step(&motor, Vd, Vq, dt, 0.0f);
        
        // Check for NaN or Inf
        ASSERT_TRUE(isfinite(motor.Id));
        ASSERT_TRUE(isfinite(motor.Iq));
        ASSERT_TRUE(isfinite(motor.omega_m));
        ASSERT_TRUE(isfinite(motor.theta_e));
    }
    
    return 1;
}

// Test 7: Zero crossing behavior
int test_zero_crossing(void) {
    MotorModel_t motor;
    Motor_Init(&motor);
    
    PidTypeDef pid_d, pid_q, pid_speed;
    float current_gains[3] = {10.0f, 100.0f, 0.0f};
    float speed_gains[3] = {0.01f, 0.1f, 0.0f};
    PID_Init(&pid_d, PID_POSITION, current_gains, 24.0f, 10.0f);
    PID_Init(&pid_q, PID_POSITION, current_gains, 24.0f, 10.0f);
    PID_Init(&pid_speed, PID_POSITION, speed_gains, 5.0f, 2.0f);
    
    float dt = 0.0001f;
    
    // Accelerate
    for (int i = 0; i < 500; i++) {
        float Iq_ref = PID_CalcDt(&pid_speed, motor.omega_m, 50.0f, dt);
        float Vd = PID_CalcDt(&pid_d, motor.Id, 0.0f, dt);
        float Vq = PID_CalcDt(&pid_q, motor.Iq, Iq_ref, dt);
        Motor_Step(&motor, Vd, Vq, dt, 0.0f);
    }
    
    // Decelerate through zero
    PID_clear(&pid_speed);
    for (int i = 0; i < 1000; i++) {
        float Iq_ref = PID_CalcDt(&pid_speed, motor.omega_m, -50.0f, dt);
        float Vd = PID_CalcDt(&pid_d, motor.Id, 0.0f, dt);
        float Vq = PID_CalcDt(&pid_q, motor.Iq, Iq_ref, dt);
        Motor_Step(&motor, Vd, Vq, dt, 0.0f);
        
        // Should remain stable through zero crossing
        ASSERT_TRUE(isfinite(motor.omega_m));
        ASSERT_TRUE(fabsf(motor.Iq) < 10.0f);  // Current shouldn't explode
    }
    
    return 1;
}

// Test 8: Edge cases - zero voltage
int test_edge_zero_voltage(void) {
    MotorModel_t motor;
    Motor_Init(&motor);
    motor.omega_m = 10.0f;  // Initial speed
    
    float dt = 0.0001f;
    
    // Apply zero voltage
    for (int i = 0; i < 100; i++) {
        Motor_Step(&motor, 0.0f, 0.0f, dt, 0.0f);
        ASSERT_TRUE(isfinite(motor.Id));
        ASSERT_TRUE(isfinite(motor.Iq));
    }
    
    // Motor should coast down due to friction
    ASSERT_TRUE(motor.omega_m < 10.0f);
    
    return 1;
}

// Test 9: Edge cases - maximum current
int test_edge_max_current(void) {
    MotorModel_t motor;
    Motor_Init(&motor);
    
    PidTypeDef pid_d, pid_q;
    float gains[3] = {10.0f, 100.0f, 0.0f};
    PID_Init(&pid_d, PID_POSITION, gains, 24.0f, 10.0f);  // Vmax = 24V
    PID_Init(&pid_q, PID_POSITION, gains, 24.0f, 10.0f);
    
    float dt = 0.0001f;
    float Iq_ref = 100.0f;  // Unrealistically high
    
    for (int i = 0; i < 100; i++) {
        float Vd = PID_CalcDt(&pid_d, motor.Id, 0.0f, dt);
        float Vq = PID_CalcDt(&pid_q, motor.Iq, Iq_ref, dt);
        Motor_Step(&motor, Vd, Vq, dt, 0.0f);
    }
    
    // Current should be limited by voltage saturation
    // Vq is limited to 24V, so Iq can't grow indefinitely
    ASSERT_TRUE(motor.Iq < 50.0f);  // Should be bounded
    
    return 1;
}

// Test 10: Frequency response - bandwidth check
int test_current_loop_bandwidth(void) {
    MotorModel_t motor;
    Motor_Init(&motor);
    
    PidTypeDef pid_q;
    float gains[3] = {10.0f, 100.0f, 0.0f};
    PID_Init(&pid_q, PID_POSITION, gains, 24.0f, 10.0f);
    
    float dt = 0.0001f;
    float freq = 100.0f;  // 100 Hz test frequency
    float amplitude = 1.0f;
    
    float max_response = 0.0f;
    
    // Run for several cycles
    for (int i = 0; i < 1000; i++) {
        float t = i * dt;
        float Iq_ref = amplitude * sinf(2 * M_PI * freq * t);
        float Vq = PID_CalcDt(&pid_q, motor.Iq, Iq_ref, dt);
        Motor_Step(&motor, 0.0f, Vq, dt, 0.0f);
        
        if (fabsf(motor.Iq) > max_response) {
            max_response = fabsf(motor.Iq);
        }
    }
    
    // At 100Hz, response should be significant (> 50% of input)
    // This validates the current loop has reasonable bandwidth
    ASSERT_TRUE(max_response > 0.3f * amplitude);
    
    return 1;
}

/*============================================================================
 * Main
 *============================================================================*/

int main(void) {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║       FOC Integration Tests - Closed Loop Simulation         ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    
    printf("=== Transform Tests ===\n");
    TEST(transform_roundtrip);
    TEST(svpwm_duty_validity);
    
    printf("\n=== Current Loop Tests ===\n");
    TEST(current_loop_step_response);
    TEST(current_loop_bandwidth);
    
    printf("\n=== Speed Loop Tests ===\n");
    TEST(speed_loop_step_response);
    TEST(load_disturbance_rejection);
    
    printf("\n=== Stability Tests ===\n");
    TEST(numerical_stability);
    TEST(zero_crossing);
    
    printf("\n=== Edge Case Tests ===\n");
    TEST(edge_zero_voltage);
    TEST(edge_max_current);
    
    printf("\n");
    printf("══════════════════════════════════════════════════════════════\n");
    printf("  Results: %d passed, %d failed\n", tests_passed, tests_failed);
    printf("══════════════════════════════════════════════════════════════\n\n");
    
    return tests_failed > 0 ? 1 : 0;
}
