#include "adc.h"

void ADC_Init(ADC_HandleTypeDef *hadc) { current_data.hadc = hadc; }

void ADC_SetCurrentOffsets(float Ia, float Ib, float Ic) {
  current_data.Ia_offset = Ia;
  current_data.Ib_offset = Ib;
  current_data.Ic_offset = Ic;
}

CURRENT_DATA current_data = {
    .hadc = NULL,
    .Temp_Result = 0.0f,
    .Ia_offset = 0.0f,
    .Ib_offset = 0.0f,
    .Ic_offset = 0.0f,
    .current_offset_sum_a = 0.0f,
    .current_offset_sum_b = 0.0f,
    .current_offset_sum_c = 0.0f,
};

/**
 * 类型: 10k NTC
 * B值: 3950
 * 参考电阻 (R25): 10kΩ
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

// NOTE: Public functions are documented in adc.h.

uint16_t adc1_median_filter(uint8_t channel) {
  uint16_t i, j, temp;
  for (j = 0; j < adc1_samples - 1; j++) {
    for (i = 1; i < adc1_samples - j; i++) {
      // Simple bubble sort
      if (adc1_dma_value[i][channel] > adc1_dma_value[i - 1][channel]) {
        temp = adc1_dma_value[i][channel];
        adc1_dma_value[i][channel] = adc1_dma_value[i - 1][channel];
        adc1_dma_value[i - 1][channel] = temp;
      }
    }
  }
  return adc1_dma_value[(adc1_samples - 1) / 2][channel];
}

uint16_t adc1_avg_filter(uint8_t channel) {
  uint32_t temp = 0;
  uint16_t i;

  for (i = 0; i < adc1_samples; i++) {
    temp += adc1_dma_value[i][channel];
  }
  return temp / adc1_samples;
}

uint16_t adc2_median_filter(uint8_t channel) {
  uint16_t i, j, temp;
  for (j = 0; j < adc2_samples - 1; j++) {
    for (i = 1; i < adc2_samples - j; i++) {
      if (adc2_dma_value[i][channel] > adc2_dma_value[i - 1][channel]) {
        temp = adc2_dma_value[i][channel];
        adc2_dma_value[i][channel] = adc2_dma_value[i - 1][channel];
        adc2_dma_value[i - 1][channel] = temp;
      }
    }
  }
  return adc2_dma_value[(adc2_samples - 1) / 2][channel];
}

uint16_t adc2_avg_filter(uint8_t channel) {
  uint32_t temp = 0; // Fixed: uninitialized variable
  uint16_t i;
  for (i = 0; i < adc2_samples; i++) {
    temp += adc2_dma_value[i][channel];
  }
  return temp / adc2_samples;
}
