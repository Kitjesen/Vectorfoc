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

#include "park.h"
#include "trigonometry.h"

void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id,
                    float *Iq) {
  float sin_val, cos_val;
  Trig_FastSinCos(theta, &sin_val, &cos_val);

  *Id = Ialpha * cos_val + Ibeta * sin_val;
  *Iq = -Ialpha * sin_val + Ibeta * cos_val;
}

void Park_Inverse(float Vd, float Vq, float theta, float *Valpha,
                  float *Vbeta) {
  float sin_val, cos_val;
  Trig_FastSinCos(theta, &sin_val, &cos_val);

  *Valpha = Vd * cos_val - Vq * sin_val;
  *Vbeta = Vd * sin_val + Vq * cos_val;
}
