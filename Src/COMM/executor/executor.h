/**
 * @file executor.h
 * @brief Command Executor - Business Logic Layer
 * @details Handles the execution of MotorCommands (Statemachine, Targets,
 * Params)
 */

#ifndef COMM_EXECUTOR_H
#define COMM_EXECUTOR_H

#include "protocol_types.h"
#include <stdbool.h>

/**
 * @brief Execute a parsed MotorCommand
 * @param cmd Pointer to the command to execute
 * @return true if executed successfully
 */
void Executor_ProcessCommand(const MotorCommand *cmd);

#endif // COMM_EXECUTOR_H
