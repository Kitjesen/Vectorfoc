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

#ifndef CONTROL_MODE_H
#define CONTROL_MODE_H

/* The mode is shared by command decoding, the FSM, and control dispatch.
 * Keeping it outside MOTOR_DATA lets control interfaces use the mode without
 * importing the complete motor runtime object. */
typedef enum {
  CONTROL_MODE_OPEN = 0,
  CONTROL_MODE_TORQUE = 1,
  CONTROL_MODE_VELOCITY = 2,
  CONTROL_MODE_POSITION = 3,
  CONTROL_MODE_VELOCITY_RAMP = 4,
  CONTROL_MODE_POSITION_RAMP = 5,
  CONTROL_MODE_MIT = 6,
  CONTROL_MODE_VF = 8,
  CONTROL_MODE_IF = 9,
} CONTROL_MODE;

#endif /* CONTROL_MODE_H */
