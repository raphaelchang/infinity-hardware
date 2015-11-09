#include "FieldOrientedController.h"

#include "Transforms.h"

FieldOrientedController::FieldOrientedController(ZSMMethod method, int p_gain, int i_gain, int excess_gain)
  : SinusoidalController(method)
{
  p = p_gain;
  i = i_gain;
  c = excess_gain;
  sum_d = sum_q = 0;
}

void FieldOrientedController::Update(int torque, int flux, int current_a, int current_b, int current_c, int edeg)
{
  int alpha, beta, current_d, current_q;
  Transforms::Clarke(current_a, current_b, current_c, &alpha, &beta);
  Transforms::Park(alpha, beta, edeg, &current_d, &current_q);
  int err = torque - current_q;
  int u = sum_q + (p * err) / factor;
  int out_q = 0;
  if (u > factor)
    out_q = factor;
  else if (u < -factor)
    out_q = -factor;
  else
    out_q = u;
  int excess = u - out_q;
  sum_q = sum_q + (i * err) / factor - (c * excess) / factor;

  err = flux - current_d;
  u = sum_d + (p * err) / factor;
  int out_d = 0;
  if (u > factor)
    out_d = factor;
  else if (u < -factor)
    out_d = -factor;
  else
    out_d = u;
  excess = u - out_d;
  sum_d = sum_d + (i * err) / factor - (c * excess) / factor;
  SinusoidalController::Update(out_d, out_q, edeg);
}

