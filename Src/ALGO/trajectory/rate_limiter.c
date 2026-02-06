#include "rate_limiter.h"
#include "common.h"
#include <math.h>

// 初始化限速器
void RateLimiter_Init(RateLimiterTypeDef *limiter, float max_rate) {
  if (limiter == NULL)
    return;

  limiter->last_value = 0.0f;
  limiter->max_rate = fabsf(max_rate);
  limiter->initialized = false;
}

// 重置限速器
void RateLimiter_Reset(RateLimiterTypeDef *limiter, float initial_value) {
  if (limiter == NULL)
    return;

  limiter->last_value = initial_value;
  limiter->initialized = true;
}

// 应用限速器
float RateLimiter_Apply(RateLimiterTypeDef *limiter, float input, float dt) {
  if (limiter == NULL)
    return input;

  if (dt < 0.000001f || dt > 1.0f)
    return limiter->last_value;

  // First call: pass through
  if (!limiter->initialized) {
    limiter->last_value = input;
    limiter->initialized = true;
    return input;
  }
  // 计算最大变化量
  float max_delta = limiter->max_rate * dt;
  // 计算实际变化量
  float delta = input - limiter->last_value;
  // 限制变化量
  float limited_delta = CLAMP(delta, -max_delta, max_delta);
  // 更新 last_value
  limiter->last_value += limited_delta;
  return limiter->last_value;
}

// 设置最大变化率
void RateLimiter_SetMaxRate(RateLimiterTypeDef *limiter, float max_rate) {
  if (limiter == NULL)
    return;

  limiter->max_rate = fabsf(max_rate);
}

// 获取限速器当前值
float RateLimiter_GetValue(const RateLimiterTypeDef *limiter) {
  if (limiter == NULL)
    return 0.0f;

  return limiter->last_value;
}
