#ifndef FIELD_ORIENTED_CONTROLLER_H_
#define FIELD_ORIENTED_CONTROLLER_H_

#include "SinusoidalController.h"

class FieldOrientedController : public SinusoidalController
{
  public:
  FieldOrientedController(ZSMMethod method, int p_gain, int i_gain);
  void Update(int torque, int flux, int current_a, int current_b, int current_c, int edeg);

  private:
  int controllerLoop(int error, int *sum);
  int p;
  int i;
  int sum_d;
  int sum_q;
  int limit;
  int d_sum;
  int q_sum;
};

#endif



