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
