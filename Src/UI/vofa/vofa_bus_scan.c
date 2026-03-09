#include "vofa_bus_scan.h"
#include "inovxio_protocol.h"
#include <string.h>

static uint32_t VofaBusScan_LoadU32LE(const uint8_t *data) {
  return ((uint32_t)data[0]) | ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static bool VofaBusScan_AddNode(VofaBusScanState *state, uint8_t node_id,
                                uint32_t uid_word0, uint32_t uid_word1) {
  uint8_t i = 0;

  if (state == NULL) {
    return false;
  }

  for (i = 0; i < state->node_count; ++i) {
    if (state->nodes[i].node_id == node_id) {
      if (state->nodes[i].uid_word0 == 0U && state->nodes[i].uid_word1 == 0U) {
        state->nodes[i].uid_word0 = uid_word0;
        state->nodes[i].uid_word1 = uid_word1;
      }
      return false;
    }
  }

  if (state->node_count >= VOFA_BUS_SCAN_MAX_NODES) {
    return false;
  }

  state->nodes[state->node_count].node_id = node_id;
  state->nodes[state->node_count].uid_word0 = uid_word0;
  state->nodes[state->node_count].uid_word1 = uid_word1;
  state->node_count++;
  return true;
}

void VofaBusScan_Init(VofaBusScanState *state) {
  if (state == NULL) {
    return;
  }

  memset(state, 0, sizeof(*state));
}

void VofaBusScan_Begin(VofaBusScanState *state, uint32_t now_ms,
                       uint32_t timeout_ms, uint8_t local_node_id,
                       uint32_t uid_word0, uint32_t uid_word1) {
  if (state == NULL) {
    return;
  }

  memset(state, 0, sizeof(*state));
  state->active = true;
  state->deadline_ms = now_ms + timeout_ms;
  (void)VofaBusScan_AddNode(state, local_node_id, uid_word0, uid_word1);
}

void VofaBusScan_Cancel(VofaBusScanState *state) {
  if (state == NULL) {
    return;
  }

  state->active = false;
  state->emit_index = state->node_count;
}

bool VofaBusScan_IsBusy(const VofaBusScanState *state) {
  if (state == NULL) {
    return false;
  }

  return state->active || state->emit_index < state->node_count;
}

void VofaBusScan_ObserveFrame(VofaBusScanState *state, const CAN_Frame *frame) {
  uint8_t cmd_type = 0;
  uint8_t target_id = 0;
  uint8_t node_id = 0;

  if (state == NULL || frame == NULL || !state->active) {
    return;
  }

  if (!frame->is_extended || frame->dlc < 8U) {
    return;
  }

  cmd_type = (uint8_t)((frame->id >> 24) & 0x1FU);
  target_id = (uint8_t)(frame->id & 0xFFU);
  node_id = (uint8_t)((frame->id >> 8) & 0xFFU);

  if (cmd_type != PRIVATE_CMD_GET_ID || target_id != VOFA_BUS_SCAN_TARGET_HOST) {
    return;
  }

  (void)VofaBusScan_AddNode(state, node_id, VofaBusScan_LoadU32LE(&frame->data[0]),
                            VofaBusScan_LoadU32LE(&frame->data[4]));
}

bool VofaBusScan_PollEvent(VofaBusScanState *state, uint32_t now_ms,
                           VofaBusScanEvent *event) {
  if (state == NULL || event == NULL) {
    return false;
  }

  memset(event, 0, sizeof(*event));

  if (state->emit_index < state->node_count) {
    event->type = VOFA_BUS_SCAN_EVENT_NODE;
    event->node = state->nodes[state->emit_index++];
    event->total_nodes = state->node_count;
    return true;
  }

  if (state->active && (int32_t)(now_ms - state->deadline_ms) >= 0) {
    state->active = false;
    event->type = VOFA_BUS_SCAN_EVENT_DONE;
    event->total_nodes = state->node_count;
    return true;
  }

  return false;
}
