#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

#include <stdint.h>

/* Type definitions. */
#define portCHAR char
#define portFLOAT float
#define portDOUBLE double
#define portLONG long
#define portSHORT short
#define portSTACK_TYPE uint32_t
#define portBASE_TYPE long

typedef portSTACK_TYPE StackType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;

#if (configUSE_16_BIT_TICKS == 1)
typedef uint16_t TickType_t;
#define portMAX_DELAY (TickType_t)0xffff
#else
typedef uint32_t TickType_t;
#define portMAX_DELAY (TickType_t)0xffffffffUL

/* 32-bit tick type on a 32-bit architecture, so reads of the tick count do
not need to be guarded with a critical section. */
#define portTICK_TYPE_IS_ATOMIC 1
#endif
/*-----------------------------------------------------------*/

/* Architecture specifics. */
#define portSTACK_GROWTH (-1)
#define portTICK_PERIOD_MS ((TickType_t)1000 / configTICK_RATE_HZ)
#define portBYTE_ALIGNMENT 8
/*-----------------------------------------------------------*/

/* Scheduler utilities. */
#define portYIELD()                                                            \
  {                                                                            \
    /* Set a PendSV to request a context switch. */                            \
    portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;                            \
                                                                               \
    /* Barriers are normally not required but do ensure the code is completely \
    within the specified behaviour for the architecture. */                    \
    __asm volatile("dsb" ::: "memory");                                        \
    __asm volatile("isb");                                                     \
  }

#define portNVIC_INT_CTRL_REG (*((volatile uint32_t *)0xe000ed04))
#define portNVIC_PENDSVSET_BIT (1UL << 28UL)
#define portEND_SWITCHING_ISR(xSwitchRequired)                                 \
  if (xSwitchRequired != pdFALSE)                                              \
  portYIELD()
#define portYIELD_FROM_ISR(x) portEND_SWITCHING_ISR(x)
/*-----------------------------------------------------------*/

/* Critical section management. */
extern void vPortEnterCritical(void);
extern void vPortExitCritical(void);
#define portENTER_CRITICAL() vPortEnterCritical()
#define portEXIT_CRITICAL() vPortExitCritical()
/*-----------------------------------------------------------*/

/* Tickless idle/low power functionality. */
#ifndef portSUPPRESS_TICKS_AND_SLEEP
extern void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime);
#define portSUPPRESS_TICKS_AND_SLEEP(xExpectedIdleTime)                        \
  vPortSuppressTicksAndSleep(xExpectedIdleTime)
#endif
/*-----------------------------------------------------------*/

/* Architecture specific optimisations. */
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#endif

#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1

/* Generic helper function. */
__attribute__((always_inline)) static inline uint8_t
ucPortCountLeadingZeros(uint32_t ulBitmap) {
  uint8_t ucReturn;

  __asm volatile("clz %0, %1" : "=r"(ucReturn) : "r"(ulBitmap) : "memory");
  return ucReturn;
}

/* Check the configuration. */
#if (configMAX_PRIORITIES > 32)
#error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice.
#endif

/* Store/clear the ready priorities in a bit map. */
#define portRECORD_READY_PRIORITY(uxPriority, uxReadyPriorities)               \
  (uxReadyPriorities) |= (1UL << (uxPriority))
#define portRESET_READY_PRIORITY(uxPriority, uxReadyPriorities)                \
  (uxReadyPriorities) &= ~(1UL << (uxPriority))

/*-----------------------------------------------------------*/

#define portGET_HIGHEST_PRIORITY(uxTopPriority, uxReadyPriorities)             \
  uxTopPriority =                                                              \
      (31UL - (uint32_t)ucPortCountLeadingZeros((uxReadyPriorities)))

#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

/*-----------------------------------------------------------*/

#ifdef configASSERT
void vPortValidateInterruptPriority(void);
#define portASSERT_IF_INTERRUPT_PRIORITY_INVALID()                             \
  vPortValidateInterruptPriority()
#endif

/* portNOP() is not required by this port. */
#define portNOP()

#define portINLINE __inline

#ifndef portFORCE_INLINE
#define portFORCE_INLINE inline __attribute__((always_inline))
#endif

/*-----------------------------------------------------------*/

static portFORCE_INLINE void vPortSetBASEPRI(uint32_t ulBASEPRI) {
  __asm volatile("msr basepri, %0" ::"r"(ulBASEPRI) : "memory");
}

/*-----------------------------------------------------------*/

static portFORCE_INLINE void vPortRaiseBASEPRI(void) {
  uint32_t ulNewBASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY;

  __asm volatile("msr basepri, %0" ::"r"(ulNewBASEPRI) : "memory");
  __asm volatile("dsb" ::: "memory");
  __asm volatile("isb");
}

/*-----------------------------------------------------------*/

static portFORCE_INLINE void vPortClearBASEPRIFromISR(void) {
  __asm volatile("msr basepri, %0" ::"r"(0) : "memory");
}

/*-----------------------------------------------------------*/

static portFORCE_INLINE uint32_t ulPortRaiseBASEPRI(void) {
  uint32_t ulReturn, ulNewBASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY;

  __asm volatile("mrs %0, basepri" : "=r"(ulReturn)::"memory");
  __asm volatile("msr basepri, %0" ::"r"(ulNewBASEPRI) : "memory");
  __asm volatile("dsb" ::: "memory");
  __asm volatile("isb");

  return ulReturn;
}

/*-----------------------------------------------------------*/

static portFORCE_INLINE void vPortSetInterruptMask(uint32_t ulNewMaskValue) {
  __asm volatile("msr basepri, %0" ::"r"(ulNewMaskValue) : "memory");
}

/*-----------------------------------------------------------*/

#define portDISABLE_INTERRUPTS() vPortRaiseBASEPRI()
#define portENABLE_INTERRUPTS() vPortSetBASEPRI(0)
#define portSET_INTERRUPT_MASK_FROM_ISR() ulPortRaiseBASEPRI()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x) vPortSetInterruptMask(x)

/*-----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#ifndef portTASK_FUNCTION_PROTO
#define portTASK_FUNCTION_PROTO(vFunction, pvParameters)                       \
  void vFunction(void *pvParameters)
#endif

#ifndef portTASK_FUNCTION
#define portTASK_FUNCTION(vFunction, pvParameters)                             \
  void vFunction(void *pvParameters)
#endif

#endif /* PORTMACRO_H */
