#ifndef VOFA_H
#define VOFA_H

#include "common.h"
#include "motor.h"
#include "motor_adc.h"
#include "mt6816_encoder.h"
#include "usart.h"
#include "usbd_cdc_if.h"

#define byte0(dw_temp) (*(char *)(&dw_temp))
#define byte1(dw_temp) (*((char *)(&dw_temp) + 1))
#define byte2(dw_temp) (*((char *)(&dw_temp) + 2))
#define byte3(dw_temp) (*((char *)(&dw_temp) + 3))

#define SCOPE_BUFFER_SIZE                                                      \
  64 // Adjust based on available RAM (64 * 12 * 4 = 3 KB)
#define SCOPE_CHANNELS 12

typedef struct {
  float data[SCOPE_BUFFER_SIZE][SCOPE_CHANNELS];
  volatile uint16_t head;
  volatile uint16_t tail;
} ScopeBuffer_t;

void Scope_Init(void);
void Scope_Update(void);
void Scope_Process(void);

void vofa_start(void);
void vofa_send_data(uint8_t num, float data);
void vofa_sendframetail(void);
void Vofa_Packet(void);
void vofa_Receive(uint8_t *buf, uint16_t len);

#endif
