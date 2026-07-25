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
 * @file    platform.h
 * @brief   Platform utility macros and helpers
 * @note    Critical sections, compiler hints, common macros
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stddef.h>

/* Critical-section intrinsics are a platform concern, not a transitive
 * dependency of whichever module happens to include this header. */
#if !defined(TEST_ENV)
#include "stm32g4xx.h"
#endif

/* ==========================================================================
   Critical Section (interrupt-safe access)
   
   Usage:
     CRITICAL_SECTION_BEGIN();
     shared_variable = new_value;
     CRITICAL_SECTION_END();
   ========================================================================== */
#if defined(TEST_ENV)
#define CRITICAL_SECTION_BEGIN() do { } while (0)
#define CRITICAL_SECTION_END()   do { } while (0)
#else
#define CRITICAL_SECTION_BEGIN()                  \
    do {                                          \
        uint32_t __primask = __get_PRIMASK();     \
        __disable_irq()

#define CRITICAL_SECTION_END()                    \
        __set_PRIMASK(__primask);                 \
    } while (0)
#endif

/* ==========================================================================
   Array utilities
   ========================================================================== */
/** Get number of elements in a static array */
#define ARRAY_SIZE(arr)     (sizeof(arr) / sizeof((arr)[0]))

/** Zero-fill a struct */
#define ZERO_STRUCT(ptr)    memset((ptr), 0, sizeof(*(ptr)))

/* ==========================================================================
   Compiler hints
   ========================================================================== */
#ifndef UNUSED
#define UNUSED(x)           ((void)(x))
#endif

#ifndef STATIC_ASSERT
#define STATIC_ASSERT(cond, msg)    _Static_assert(cond, msg)
#endif

/** Mark a variable as potentially unused (suppress warnings) */
#ifdef __GNUC__
  #define ATTR_UNUSED       __attribute__((unused))
  #define ATTR_PACKED       __attribute__((packed))
  #define ATTR_ALIGNED(n)   __attribute__((aligned(n)))
  #define ATTR_WEAK         __attribute__((weak))
  #define LIKELY(x)         __builtin_expect(!!(x), 1)
  #define UNLIKELY(x)       __builtin_expect(!!(x), 0)
#else
  #define ATTR_UNUSED
  #define ATTR_PACKED
  #define ATTR_ALIGNED(n)
  #define ATTR_WEAK
  #define LIKELY(x)         (x)
  #define UNLIKELY(x)       (x)
#endif

/* ==========================================================================
   Numeric helpers
   ========================================================================== */
#ifndef CLAMP
#define CLAMP(val, lo, hi)  ((val) < (lo) ? (lo) : ((val) > (hi) ? (hi) : (val)))
#endif
#ifndef MIN
#define MIN(a, b)           ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b)           ((a) > (b) ? (a) : (b))
#endif
#ifndef ABS
#define ABS(x)              ((x) < 0 ? -(x) : (x))
#endif

/* ==========================================================================
   Bit manipulation
   ========================================================================== */
#define BIT(n)              (1UL << (n))
#define BIT_SET(reg, bit)   ((reg) |= (bit))
#define BIT_CLR(reg, bit)   ((reg) &= ~(bit))
#define BIT_IS_SET(reg, bit) (((reg) & (bit)) != 0)

#endif /* PLATFORM_H */
