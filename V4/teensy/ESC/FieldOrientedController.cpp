#include "FieldOrientedController.h"

#include "Transforms.h"
#include <Arduino.h>

FieldOrientedController::FieldOrientedController(ZSMMethod method, int p_gain, int i_gain)
  : SinusoidalController(method)
{
  p = p_gain;
  i = i_gain;
  sum_d = sum_q = 0;
  d_sum = q_sum = 0;
  limit = 500;
}

int FieldOrientedController::controllerLoop(int error, int *sum)
{
  *sum += (error * i) / factor;
  if (*sum > limit)
    *sum = limit;
  else if (*sum < -limit)
    *sum = -limit;
  int out = (error * p) / factor + *sum;
  if (out > limit)
    out = limit;
  else if (out < -limit)
    out = -limit;
  return out;
}

void FieldOrientedController::Update(int torque, int flux, int current_a, int current_b, int current_c, int edeg)
{
  int alpha, beta, current_d, current_q;
  Transforms::Clarke(current_a, current_b, current_c, &alpha, &beta);
  Transforms::Park(alpha, beta, edeg, &current_d, &current_q);
  int err = torque - current_q;
  int out_q = controllerLoop(err, &sum_q);
  Serial.print(err);
  Serial.print(",");
  Serial.print(sum_q);
  Serial.print(",");
  Serial.print(out_q);
  Serial.print(",");

  err = flux - current_d;
  int out_d = controllerLoop(err, &sum_d);
  Serial.print(err);
  Serial.print(",");
  Serial.print(sum_d);
  Serial.print(",");
  Serial.print(out_d);
  Serial.print(",");
  SinusoidalController::Update(out_d, out_q, edeg);
}




