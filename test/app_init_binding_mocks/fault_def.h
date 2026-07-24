#ifndef TEST_APP_INIT_BINDING_MOCKS_FAULT_DEF_H
#define TEST_APP_INIT_BINDING_MOCKS_FAULT_DEF_H
#include <stdbool.h>
#include <stdint.h>
#define FAULT_NONE 0u
#define FAULT_POSITION_INIT (1u << 9)
typedef struct { uint8_t unused; } DetectionConfig;
typedef struct { DetectionConfig detection_config; bool (*fault_callback)(uint32_t fault_code, void *motor); bool auto_clear_on_recover; } SafetyConfig;
#endif