#ifndef VOFA_BUS_SCAN_H
#define VOFA_BUS_SCAN_H

#include "protocol_types.h"
#include <stdbool.h>
#include <stdint.h>

#define VOFA_BUS_SCAN_MAX_NODES 16U
#define VOFA_BUS_SCAN_TARGET_HOST 0xFEU

typedef struct {
  uint8_t node_id;
  uint32_t uid_word0;
  uint32_t uid_word1;
} VofaBusNode;

typedef struct {
  bool active;
  uint32_t deadline_ms;
  uint8_t node_count;
  uint8_t emit_index;
  VofaBusNode nodes[VOFA_BUS_SCAN_MAX_NODES];
} VofaBusScanState;

typedef enum {
  VOFA_BUS_SCAN_EVENT_NONE = 0,
  VOFA_BUS_SCAN_EVENT_NODE,
  VOFA_BUS_SCAN_EVENT_DONE,
} VofaBusScanEventType;

typedef struct {
  VofaBusScanEventType type;
  VofaBusNode node;
  uint8_t total_nodes;
} VofaBusScanEvent;

void VofaBusScan_Init(VofaBusScanState *state);
void VofaBusScan_Begin(VofaBusScanState *state, uint32_t now_ms,
                       uint32_t timeout_ms, uint8_t local_node_id,
                       uint32_t uid_word0, uint32_t uid_word1);
void VofaBusScan_Cancel(VofaBusScanState *state);
bool VofaBusScan_IsBusy(const VofaBusScanState *state);
void VofaBusScan_ObserveFrame(VofaBusScanState *state, const CAN_Frame *frame);
bool VofaBusScan_PollEvent(VofaBusScanState *state, uint32_t now_ms,
                           VofaBusScanEvent *event);

#endif
