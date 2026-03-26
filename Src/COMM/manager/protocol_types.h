/**
 * @file    protocol_types.h
 * @brief   Shared types for communication protocols.
 * @details
 * - Context: Common data structures for multi-protocol support.
 * - Units:   Position [rad], Velocity [rad/s], Torque [Nm], Current [A].
 */

#ifndef PROTOCOL_TYPES_H
#define PROTOCOL_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Supported protocol types.
 */
typedef enum {
  PROTOCOL_INOVXIO = 0, /**< Inovxio (MinerU) private protocol */
  PROTOCOL_CANOPEN = 1, /**< CANopen DS402 */
  PROTOCOL_MIT = 2      /**< MIT Cheetah lightweight protocol */
} ProtocolType;

/**
 * @brief CAN frame structure.
 */
typedef struct {
  uint32_t id;      /**< CAN ID (11-bit or 29-bit) */
  uint8_t dlc;      /**< Data length (0-8) */
  uint8_t data[8];  /**< Payload data */
  bool is_extended; /**< Extended frame flag */
  bool is_rtr;      /**< Remote transmission request flag */
} CAN_Frame;

/**
 * @brief Motor command structure.
 */
typedef struct {
  /* MIT impedance control parameters */
  float pos_setpoint; /**< [rad] Position setpoint */
  float vel_setpoint; /**< [rad/s] Velocity setpoint */
  float kp;           /**< Stiffness coefficient */
  float kd;           /**< Damping coefficient */
  float torque_ff;    /**< [Nm] Feedforward torque */

  /* Other control mode parameters */
  float iq_ref;       /**< [A] Current setpoint */
  float speed_ref;    /**< [rad/s] Speed setpoint */
  float position_ref; /**< [rad] Position setpoint */

  /* Control flags */
  uint8_t control_mode; /**< Control mode */
  bool enable_motor;    /**< Motor enable flag */
  bool set_zero;        /**< Set zero position flag */

  /* Protocol switching */
  bool is_protocol_switch; /**< Protocol switch request flag */
  uint8_t target_protocol; /**< Target protocol type */

  /* Parameter read/write */
  uint16_t param_index; /**< Parameter index */
  uint32_t param_value; /**< Parameter value */
  bool is_param_read;   /**< Parameter read request */
  bool is_param_write;  /**< Parameter write request */
  bool is_fault_query;  /**< Fault query request */

  /* Standard FSM Integration */
  bool has_control_word; /**< Flag indicating valid raw controlword */
  uint16_t control_word; /**< Raw DS402 controlword */

} MotorCommand;

/**
 * @brief Motor status structure.
 */
typedef struct {
  /* Real-time feedback */
  float position;    /**< [rad] Current position */
  float velocity;    /**< [rad/s] Current velocity */
  float torque;      /**< [Nm] Current torque */
  float current;     /**< [A] Current */
  float temperature; /**< [degC] Temperature */
  float voltage;     /**< [V] Bus voltage */

  /* Status information */
  uint8_t motor_state;   /**< Motor state */
  uint8_t control_mode;  /**< Control mode */
  uint32_t fault_code;   /**< Fault code */
  uint32_t warning_code; /**< Warning code */

  /* CAN information */
  uint8_t can_id; /**< CAN ID */

} MotorStatus;

/**
 * @brief Parse result codes.
 */
typedef enum {
  PARSE_OK = 0,            /**< Parse successful */
  PARSE_ERR_INVALID_FRAME, /**< Invalid frame */
  PARSE_ERR_CHECKSUM,      /**< Checksum error */
  PARSE_ERR_UNSUPPORTED,   /**< Unsupported command */
  PARSE_UNKNOWN_ID,        /**< Unknown CAN ID (not an error, just ignored) */
} ParseResult;

#endif /* PROTOCOL_TYPES_H */
