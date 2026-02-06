#include "motor_adc.h"
#include "config.h"
#include "motor_hal_api.h"
#include "mt6816_encoder.h"
#include "hal_abstraction.h" // For HAL_GetTemperature()
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern MT6816_Handle_t encoder_data;
extern CURRENT_DATA current_data;

/* ============================================================================
 * PWM Interface Implementation
 * ============================================================================
 */

static void G431_PWM_SetDuty(float dtc_a, float dtc_b, float dtc_c) {
  // 直接写 TIM 比较寄存器，死区补偿在 inner.c 中完成
  uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)(dtc_a * arr));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)(dtc_b * arr));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(dtc_c * arr));
}

static void G431_PWM_Enable(void) {
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

static void G431_PWM_Disable(void) {
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

static void G431_PWM_Brake(void) {
  // Low side ON, High side OFF
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  // Ensure outputs are enabled
  G431_PWM_Enable();
}

static const Motor_HAL_PwmInterface_t g431_pwm = {.set_duty = G431_PWM_SetDuty,
                                                  .enable = G431_PWM_Enable,
                                                  .disable = G431_PWM_Disable,
                                                  .brake = G431_PWM_Brake};

/* ============================================================================
 * ADC Interface Implementation
 * ============================================================================
 */

// 温度读取优化参数
#define TEMP_UPDATE_INTERVAL_MS 20    // 50Hz 温度更新频率
#define TEMP_LPF_ALPHA 0.1f           // 一阶低通滤波系数 (0-1，越小越平滑)
#define TEMP_ADC_MIN_VALID 100        // 最小有效 ADC 值（避免短路）
#define TEMP_ADC_MAX_VALID 4000       // 最大有效 ADC 值（避免开路）
#define TEMP_DEFAULT 25.0f            // 默认温度（传感器故障时）

static uint32_t s_last_temp_update = 0;
static float s_temp_filtered = TEMP_DEFAULT;
static bool s_temp_sensor_ok = true;

/**
 * @brief 读取并处理温度（带降频、滤波和错误检查）
 * @return 处理后的温度值
 */
static float G431_ReadTemperature(void) {
  // 1. 降频：检查是否到了更新时间
  uint32_t now = HAL_GetSystemTick();
  if (now - s_last_temp_update < TEMP_UPDATE_INTERVAL_MS) {
    return s_temp_filtered;  // 返回缓存的滤波值
  }
  s_last_temp_update = now;

  // 2. 读取原始 ADC 值（直接读取，避免重复滤波）
  // 注意：这里读取的是 DMA 数组的最新值
  uint16_t adc_raw = adc2_dma_value[0][adc2_ch12];

  // 3. 错误检查：ADC 值是否在有效范围
  if (adc_raw < TEMP_ADC_MIN_VALID || adc_raw > TEMP_ADC_MAX_VALID) {
    if (s_temp_sensor_ok) {
      // 首次检测到故障，记录错误
      s_temp_sensor_ok = false;
      // 可选：ERROR_REPORT(ERROR_SENSOR_TEMP, "Temp sensor out of range");
    }
    // 保持上次的滤波值，不更新
    return s_temp_filtered;
  }

  // 4. 查表转换为温度
  float temp;
  GetTempNtc(adc_raw, &temp);

  // 5. 一阶低通滤波
  s_temp_filtered += TEMP_LPF_ALPHA * (temp - s_temp_filtered);

  s_temp_sensor_ok = true;
  return s_temp_filtered;
}

static void G431_ADC_Update(Motor_HAL_SensorData_t *data) {
  // Read from JDR registers (Injected group)
  // Note: This matches GetMotorADC1PhaseCurrent implementation

  // Scale factors from foc_driver.h
  // #define FAC_CURRENT ((3.3f / 4095.0f) / (CURRENT_SHUNT_RES *
  // CURRENT_AMP_GAIN)) #define VOLTAGE_TO_ADC_FACTOR ...

  // We reuse the macros or constants if available, or redefine them
  // here/include them Ideally we should get them from a unified config. For
  // now, we assume macros exist or we access current_data

  // Let's use the existing current_data struct to hold offsets,
  // but do the reading here to fulfill the HAL contract.

  float adc_i_a =
      (float)hadc1.Instance
          ->JDR3; // Note: Check if JDR3 corresponds to Phase A in previous code
  float adc_i_b = (float)hadc1.Instance->JDR2;
  float adc_i_c = (float)hadc1.Instance->JDR1;
  float adc_vbus = (float)hadc1.Instance->JDR4;

  // 零偏来自 current_data (adc.h)，校准结果通过 ADC_SetCurrentOffsets 写入
  data->i_a = (adc_i_a - current_data.Ia_offset) * FAC_CURRENT;
  data->i_b = (adc_i_b - current_data.Ib_offset) * FAC_CURRENT;
  data->i_c = (adc_i_c - current_data.Ic_offset) * FAC_CURRENT;

  data->v_bus = adc_vbus * VOLTAGE_TO_ADC_FACTOR;

  // 读取温度（带降频、滤波和错误检查）
  data->temp = G431_ReadTemperature();
}

static void G431_ADC_CalibrateOffsets(void) {
  // Reuse existing calibration logic if possible, or reimplement
  // For simplicity, we trigger the existing calibration function if exposed,
  // or implement a simple average here.

  uint32_t sum_a = 0, sum_b = 0, sum_c = 0;
  const int samples = 1000;

  for (int i = 0; i < samples; i++) {
    HAL_Delay(1); // Simple blocking delay for calibration
    sum_a += hadc1.Instance->JDR3;
    sum_b += hadc1.Instance->JDR2;
    sum_c += hadc1.Instance->JDR1;
  }

  current_data.Ia_offset = (float)sum_a / samples;
  current_data.Ib_offset = (float)sum_b / samples;
  current_data.Ic_offset = (float)sum_c / samples;
}

static const Motor_HAL_AdcInterface_t g431_adc = {
    .update = G431_ADC_Update, .calibrate_offsets = G431_ADC_CalibrateOffsets};

/* ============================================================================
 * Encoder Interface Implementation
 * ============================================================================
 */

static void G431_Encoder_Update(void) {
  // Call the MT6816 update function
  MT6816_Update(&encoder_data, CURRENT_MEASURE_PERIOD);
}

static void G431_Encoder_GetData(Motor_HAL_EncoderData_t *data) {
  data->angle_rad = encoder_data.mec_angle_rad;
  // Wait, check MT6816 struct. It has mec_angle_rad?
  // Checking Step 1089... yes: float mec_angle_rad;

  data->velocity_rad = encoder_data.velocity_rad_s;
  data->elec_angle = encoder_data.elec_angle_rad;
  data->raw_value = encoder_data.raw_angle;
}

static void G431_Encoder_SetOffset(float offset) {
  encoder_data.offset_rev =
      offset; // Assuming offset_rev is the mechanical zero offset
}

static const Motor_HAL_EncoderInterface_t g431_encoder = {
    .update = G431_Encoder_Update,
    .get_data = G431_Encoder_GetData,
    .set_offset = G431_Encoder_SetOffset};

/* ============================================================================
 * Main Handle Construction
 * ============================================================================
 */

Motor_HAL_Handle_t g431_hal_handle = {
    .pwm = &g431_pwm, .adc = &g431_adc, .encoder = &g431_encoder};
