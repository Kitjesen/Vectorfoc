/**
 * @file    test_trigonometry.c
 * @brief   Unit tests for fast trigonometry functions
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "common.h"
#include "trigonometry.h"

#define EPSILON 0.01f  // 1% error tolerance for fast trig
#define ASSERT_FLOAT_NEAR(a, b, eps) assert(fabsf((a) - (b)) < (eps))

static int tests_passed = 0;

#define TEST(name) \
    printf("  Testing: %s ... ", #name); \
    test_##name(); \
    printf("PASS\n"); \
    tests_passed++;

void test_sincos_zero(void) {
    float s, c;
    Trig_FastSinCos(0.0f, &s, &c);
    ASSERT_FLOAT_NEAR(s, 0.0f, EPSILON);
    ASSERT_FLOAT_NEAR(c, 1.0f, EPSILON);
}

void test_sincos_pi_2(void) {
    float s, c;
    Trig_FastSinCos((float)(M_PI / 2.0), &s, &c);
    ASSERT_FLOAT_NEAR(s, 1.0f, EPSILON);
    ASSERT_FLOAT_NEAR(c, 0.0f, EPSILON);
}

void test_sincos_pi(void) {
    float s, c;
    Trig_FastSinCos((float)M_PI, &s, &c);
    ASSERT_FLOAT_NEAR(s, 0.0f, EPSILON);
    ASSERT_FLOAT_NEAR(c, -1.0f, EPSILON);
}

void test_sincos_3pi_2(void) {
    float s, c;
    Trig_FastSinCos((float)(3.0 * M_PI / 2.0), &s, &c);
    ASSERT_FLOAT_NEAR(s, -1.0f, EPSILON);
    ASSERT_FLOAT_NEAR(c, 0.0f, EPSILON);
}

void test_sincos_negative(void) {
    float s, c;
    Trig_FastSinCos((float)(-M_PI / 2.0), &s, &c);
    ASSERT_FLOAT_NEAR(s, -1.0f, EPSILON);
    ASSERT_FLOAT_NEAR(c, 0.0f, EPSILON);
}

void test_sin_cos_identity(void) {
    // sin^2 + cos^2 = 1
    for (float angle = 0; angle < 2 * M_PI; angle += 0.1f) {
        float s, c;
        Trig_FastSinCos(angle, &s, &c);
        float sum = s * s + c * c;
        ASSERT_FLOAT_NEAR(sum, 1.0f, 0.02f);
    }
}

void test_accuracy_vs_stdlib(void) {
    float max_sin_err = 0.0f;
    float max_cos_err = 0.0f;
    
    for (float angle = -2 * M_PI; angle < 2 * M_PI; angle += 0.01f) {
        float s, c;
        Trig_FastSinCos(angle, &s, &c);
        
        float sin_err = fabsf(s - sinf(angle));
        float cos_err = fabsf(c - cosf(angle));
        
        if (sin_err > max_sin_err) max_sin_err = sin_err;
        if (cos_err > max_cos_err) max_cos_err = cos_err;
    }
    
    printf("(max_sin_err=%.4f, max_cos_err=%.4f) ", max_sin_err, max_cos_err);
    
    // Should be within 1% for FOC applications
    assert(max_sin_err < 0.02f);
    assert(max_cos_err < 0.02f);
}

void test_large_angles(void) {
    // Test angle wrapping for large angles
    float s1, c1, s2, c2;
    
    Trig_FastSinCos(0.5f, &s1, &c1);
    Trig_FastSinCos(0.5f + 2 * M_PI, &s2, &c2);
    
    ASSERT_FLOAT_NEAR(s1, s2, 0.02f);
    ASSERT_FLOAT_NEAR(c1, c2, 0.02f);
}

int main(void) {
    printf("\n=== Fast Trigonometry Unit Tests ===\n\n");
    
    TEST(sincos_zero);
    TEST(sincos_pi_2);
    TEST(sincos_pi);
    TEST(sincos_3pi_2);
    TEST(sincos_negative);
    TEST(sin_cos_identity);
    TEST(accuracy_vs_stdlib);
    TEST(large_angles);
    
    printf("\n=== Results: %d passed ===\n\n", tests_passed);
    
    return 0;
}
