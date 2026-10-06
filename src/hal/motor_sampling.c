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

#include "motor_sampling.h"
#include "board_configuration.h"
#include "hardware_interface.h"
void ADC_SetCurrentOffsets(float Ia, float Ib, float Ic) {
  current_data.Ia_offset = Ia;
  current_data.Ib_offset = Ib;
  current_data.Ic_offset = Ic;
}
CURRENT_DATA current_data = {0};
/**
 * : 10k NTC
 * B: 3950
 *  (R25): 10kΩ
 */
static const uint16_t Temp_TAB[] = {
    3738, 3719, 3698, 3677, 3655, 3631, 3607, 3582, 3556, 3530, 3502, 3473,
    3443, 3412, 3381, 3348, 3314, 3280, 3244, 3208, 3170, 3132, 3093, 3053,
    3012, 2970, 2928, 2885, 2842, 2797, 2752, 2707, 2661, 2615, 2568, 2522,
    2474, 2427, 2379, 2332, 2284, 2237, 2189, 2142, 2095, 2048, 2001, 1954,
    1908, 1863, 1818, 1773, 1729, 1685, 1642, 1600, 1558, 1517, 1477, 1437,
    1398, 1360, 1323, 1286, 1250, 1215, 1180, 1147, 1114, 1082, 1051, 1020,
    990,  961,  933,  905,  878,  852,  827,  802,  778,  755,  732,  710,
    688,  668,  647,  628,  609,  590,  572,  555,  538,  522,  506,  491,
    476,  461,  447,  434,  421,  408,  395,  384,  372,  361,  350,  340,
    330,  320,  311,  302,  293,  284,  276,  268,  261,  253,  246,  239,
    233,  226,  220,  214,  209,  203};
void GetTempNtc(uint16_t value_adc, float *value_temp) {
  uint8_t index_l, index_r;
  uint8_t Temp_Tab_Zize = 126;
  int32_t temp = 0;
  index_l = 0;
  index_r = Temp_Tab_Zize - 1;
  // Binary search
  for (; index_r - index_l > 1;) {
    if ((value_adc <= Temp_TAB[index_l]) &&
        (value_adc > Temp_TAB[(index_r + index_l) % 2 == 0
                                  ? (index_r + index_l) / 2
                                  : (index_r + index_l) / 2 + 1])) {
      index_r = (index_r + index_l) % 2 == 0 ? (index_r + index_l) / 2
                                             : (index_r + index_l) / 2 + 1;
    } else {
      index_l = (index_r + index_l) / 2;
    }
  }
  // Linear interpolation
  if (Temp_TAB[index_l] == value_adc) {
    temp = (((int16_t)index_l) - 20) * 10;
  } else if (Temp_TAB[index_r] == value_adc) {
    temp = (((int16_t)index_r) - 20) * 10;
  } else {
    if (Temp_TAB[index_l] - Temp_TAB[index_r] == 0) {
      temp = (((int16_t)index_l) - 20) * 10;
    } else {
      temp = (((int16_t)index_l) - 20) * 10 +
             ((Temp_TAB[index_l] - value_adc) * 100 + 5) / 10 /
                 (Temp_TAB[index_l] - Temp_TAB[index_r]);
    }
  }
  *value_temp = ((float)temp / 10.0f);
}
// temperatureparam
#define TEMP_UPDATE_INTERVAL_MS 20    // 50Hz temperatureupdatefrequency
#define TEMP_LPF_ALPHA 0.1f           // filter (0-1，)
#define TEMP_ADC_MIN_VALID 100        //  ADC （）
#define TEMP_ADC_MAX_VALID 4000       //  ADC （）
#define TEMP_DEFAULT 25.0f            // temperature（fault）
static uint32_t s_last_temp_update = 0;
static float s_temp_filtered = TEMP_DEFAULT;
static bool s_temp_sensor_ok = true;
/**
 * @brief temperature（、filtererrorcheck）
 * @return temperature
 */
static float G431_ReadTemperature(void) {
  // 1. ：checkupdate
  uint32_t now = HAL_GetSystemTick();
  if (now - s_last_temp_update < TEMP_UPDATE_INTERVAL_MS) {
    return s_temp_filtered;  // filter
  }
  s_last_temp_update = now;
  // 2.  ADC （，filter）
  // ： DMA
  uint16_t adc_raw = adc2_dma_value[0][0];
  // 3. errorcheck：ADC
  if (adc_raw < TEMP_ADC_MIN_VALID || adc_raw > TEMP_ADC_MAX_VALID) {
    if (s_temp_sensor_ok) {
      // fault，error
      s_temp_sensor_ok = false;
      // ：ERROR_REPORT(ERROR_SENSOR_TEMP, "Temp sensor out of range");
    }
    // filter，update
    return s_temp_filtered;
  }
  // 4. temperature
  float temp;
  GetTempNtc(adc_raw, &temp);
  // 5. filter
  s_temp_filtered += TEMP_LPF_ALPHA * (temp - s_temp_filtered);
  s_temp_sensor_ok = true;
  return s_temp_filtered;
}
void ADC_ReadSensors(MotorSensorData *data) {
  /* Read the injected conversion ranks in the board configuration order. */
  float a = (float)HW_ADC_IA_HANDLE.Instance->HW_ADC_IA_JDR;
  float b = (float)HW_ADC_IB_HANDLE.Instance->HW_ADC_IB_JDR;
  float c = (float)HW_ADC_IC_HANDLE.Instance->HW_ADC_IC_JDR;
  float vbus = (float)HW_ADC_VBUS_HANDLE.Instance->HW_ADC_VBUS_JDR;
  data->i_a = (a - current_data.Ia_offset) * HW_FAC_CURRENT;
  data->i_b = (b - current_data.Ib_offset) * HW_FAC_CURRENT;
  data->i_c = (c - current_data.Ic_offset) * HW_FAC_CURRENT;
  data->v_bus = vbus * HW_VOLTAGE_FACTOR;
  data->temp = G431_ReadTemperature();
}
