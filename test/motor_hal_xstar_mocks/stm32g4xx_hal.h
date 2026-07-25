#ifndef MOTOR_HAL_XSTAR_MOCK_STM32G4XX_HAL_H
#define MOTOR_HAL_XSTAR_MOCK_STM32G4XX_HAL_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
  HAL_OK = 0,
  HAL_ERROR = 1,
} HAL_StatusTypeDef;

typedef struct {
  uint32_t ARR;
  uint32_t CCR1;
  uint32_t CCR2;
  uint32_t CCR3;
  uint32_t CCR4;
} TIM_TypeDef;

typedef struct {
  TIM_TypeDef *Instance;
} TIM_HandleTypeDef;

typedef struct {
  uint32_t JDR1;
  uint32_t JDR2;
  uint32_t JDR3;
  uint32_t JDR4;
  uint32_t IER;
  uint32_t ISR;
} ADC_TypeDef;

typedef struct {
  ADC_TypeDef *Instance;
} ADC_HandleTypeDef;

typedef struct {
  uint32_t CYCCNT;
} DWT_Type;

extern DWT_Type mock_dwt;
#define DWT (&mock_dwt)

typedef struct { int unused; } FDCAN_HandleTypeDef;
typedef struct { int unused; } UART_HandleTypeDef;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern UART_HandleTypeDef huart2;

#define TIM1 ((TIM_TypeDef *)0x40012c00u)
#define TIM_CHANNEL_1 1u
#define TIM_CHANNEL_2 2u
#define TIM_CHANNEL_3 3u
#define TIM_CHANNEL_4 4u
#define TIM_BREAKPOLARITY_HIGH 0u
#define TIM_BREAK2POLARITY_HIGH 0u

#define GPIO_PIN_3  (1u << 3)
#define GPIO_PIN_4  (1u << 4)
#define GPIO_PIN_5  (1u << 5)
#define GPIO_PIN_6  (1u << 6)
#define GPIO_PIN_7  (1u << 7)
#define GPIO_PIN_8  (1u << 8)
#define GPIO_PIN_9  (1u << 9)
#define GPIO_PIN_10 (1u << 10)
#define GPIO_PIN_11 (1u << 11)
#define GPIO_PIN_12 (1u << 12)
#define GPIO_PIN_13 (1u << 13)
#define GPIO_PIN_15 (1u << 15)
#define GPIOA ((void *)0x48000000u)
#define GPIOB ((void *)0x48000400u)
#define GPIOC ((void *)0x48000800u)

#define ADC1 ((ADC_TypeDef *)0x50000000u)
#define ADC2 ((ADC_TypeDef *)0x50000100u)
#define ADC_CHANNEL_3 3u
#define ADC_CHANNEL_11 11u
#define ADC_CHANNEL_12 12u
#define ADC_CHANNEL_14 14u
#define ADC_FLAG_JEOC (1u << 2)
#define ADC_FLAG_JEOS (1u << 3)
#define ADC_IT_JEOC   (1u << 6)
#define ADC_IT_JEOS   (1u << 7)

#define FDCAN1 ((void *)0x40006400u)
#define USART2 ((void *)0x40004400u)

#define READ_BIT(REG, MASK) ((REG) & (MASK))
#define __DSB() ((void)0)

#define __HAL_TIM_GET_AUTORELOAD(HANDLE) ((HANDLE)->Instance->ARR)
#define __HAL_TIM_SET_COMPARE(HANDLE, CHANNEL, VALUE) \
  mock_hal_tim_set_compare((HANDLE), (CHANNEL), (VALUE))
#define __HAL_ADC_CLEAR_FLAG(HANDLE, FLAG) \
  mock_hal_adc_clear_flag((HANDLE), (FLAG))
#define __HAL_ADC_GET_FLAG(HANDLE, FLAG) \
  mock_hal_adc_get_flag((HANDLE), (FLAG))
#define __HAL_ADC_DISABLE_IT(HANDLE, MASK) ((HANDLE)->Instance->IER &= ~(MASK))
#define __HAL_ADC_ENABLE_IT(HANDLE, MASK) ((HANDLE)->Instance->IER |= (MASK))

void mock_hal_tim_set_compare(TIM_HandleTypeDef *handle, uint32_t channel,
                              uint32_t value);
void mock_hal_adc_clear_flag(ADC_HandleTypeDef *handle, uint32_t flag);
uint32_t mock_hal_adc_get_flag(ADC_HandleTypeDef *handle, uint32_t flag);

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *handle,
                                    uint32_t channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *handle,
                                       uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *handle,
                                   uint32_t channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *handle,
                                      uint32_t channel);
void HAL_WatchdogFeed(void);

#endif
