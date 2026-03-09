/**
 * @file    test_rate_limiter.c
 * @brief   Unit tests for rate limiter
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "rate_limiter.h"

#define EPSILON 1e-5f
#define ASSERT_FLOAT_EQ(a, b) assert(fabsf((a) - (b)) < EPSILON)
#define ASSERT_FLOAT_NEAR(a, b, eps) assert(fabsf((a) - (b)) < (eps))

static int tests_passed = 0;

#define TEST(name) \
    printf("  Testing: %s ... ", #name); \
    test_##name(); \
    printf("PASS\n"); \
    tests_passed++;

void test_rate_limiter_init(void) {
    RateLimiterTypeDef limiter;
    RateLimiter_Init(&limiter, 100.0f);
    
    assert(limiter.max_rate == 100.0f);
    assert(limiter.initialized == false);  // Not initialized until first call
}

void test_rate_limiter_first_call(void) {
    RateLimiterTypeDef limiter;
    RateLimiter_Init(&limiter, 100.0f);
    
    // First call should return input directly
    float output = RateLimiter_Apply(&limiter, 50.0f, 0.01f);
    ASSERT_FLOAT_EQ(output, 50.0f);
    assert(limiter.initialized == true);
}

void test_rate_limiter_within_limit(void) {
    RateLimiterTypeDef limiter;
    RateLimiter_Init(&limiter, 100.0f);  // 100 units/s
    
    RateLimiter_Apply(&limiter, 0.0f, 0.01f);  // Initialize at 0
    
    // Request 0.5 change in 0.01s = 50 units/s (within limit)
    float output = RateLimiter_Apply(&limiter, 0.5f, 0.01f);
    ASSERT_FLOAT_EQ(output, 0.5f);
}

void test_rate_limiter_exceeds_limit(void) {
    RateLimiterTypeDef limiter;
    RateLimiter_Init(&limiter, 100.0f);  // 100 units/s
    
    RateLimiter_Apply(&limiter, 0.0f, 0.01f);  // Initialize at 0
    
    // Request 10.0 change in 0.01s = 1000 units/s (exceeds limit)
    // Should be limited to 100 * 0.01 = 1.0
    float output = RateLimiter_Apply(&limiter, 10.0f, 0.01f);
    ASSERT_FLOAT_EQ(output, 1.0f);
}

void test_rate_limiter_negative_direction(void) {
    RateLimiterTypeDef limiter;
    RateLimiter_Init(&limiter, 100.0f);
    
    RateLimiter_Apply(&limiter, 10.0f, 0.01f);  // Initialize at 10
    
    // Request -10 (from 10 to 0) in 0.01s = -1000 units/s
    // Should be limited to -100 * 0.01 = -1.0 change
    float output = RateLimiter_Apply(&limiter, 0.0f, 0.01f);
    ASSERT_FLOAT_EQ(output, 9.0f);  // 10 - 1 = 9
}

void test_rate_limiter_reset(void) {
    RateLimiterTypeDef limiter;
    RateLimiter_Init(&limiter, 100.0f);
    
    RateLimiter_Apply(&limiter, 50.0f, 0.01f);
    
    // Reset to new value
    RateLimiter_Reset(&limiter, 100.0f);
    
    // Next call should start from 100
    float output = RateLimiter_Apply(&limiter, 100.5f, 0.01f);
    ASSERT_FLOAT_EQ(output, 100.5f);  // Within limit
}

void test_rate_limiter_ramp_up(void) {
    RateLimiterTypeDef limiter;
    RateLimiter_Init(&limiter, 10.0f);  // 10 units/s
    
    RateLimiter_Apply(&limiter, 0.0f, 0.1f);  // Start at 0
    
    // Ramp up to 100 with rate limit
    float output = 0.0f;
    for (int i = 0; i < 10; i++) {
        output = RateLimiter_Apply(&limiter, 100.0f, 0.1f);
    }
    
    // After 10 steps of 0.1s each, should have ramped up by 10 * 10 * 0.1 = 10
    ASSERT_FLOAT_NEAR(output, 10.0f, 0.1f);
}

int main(void) {
    printf("\n=== Rate Limiter Unit Tests ===\n\n");
    
    TEST(rate_limiter_init);
    TEST(rate_limiter_first_call);
    TEST(rate_limiter_within_limit);
    TEST(rate_limiter_exceeds_limit);
    TEST(rate_limiter_negative_direction);
    TEST(rate_limiter_reset);
    TEST(rate_limiter_ramp_up);
    
    printf("\n=== Results: %d passed ===\n\n", tests_passed);
    
    return 0;
}
