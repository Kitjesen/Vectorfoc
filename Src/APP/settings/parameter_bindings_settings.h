// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

/**
 * @file parameter_bindings_settings.h
 * @brief APP composition root for parameter table runtime storage bindings.
 */
#ifndef PARAMETER_BINDINGS_SETTINGS_H
#define PARAMETER_BINDINGS_SETTINGS_H

#include "param_table.h"

/**
 * Bind the parameter metadata table to this firmware's runtime storage.
 *
 * Call after Detection_Init() and before Param_SystemInitOnce().  This only
 * supplies RAM addresses and defaults; it does not apply runtime side effects.
 */
ParamResult ParameterBindingsSettings_Install(void);

#endif /* PARAMETER_BINDINGS_SETTINGS_H */
