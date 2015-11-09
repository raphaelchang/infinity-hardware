#include "SinusoidalController.h"

#include "ZSM.h"
#include "Transforms.h"

SinusoidalController::SinusoidalController(ZSMMethod method)
{
  this->method = method;
  a = b = c = 0;
}

void SinusoidalController::Update(int voltage_d, int voltage_q, int edeg)
{
  int alpha, beta;
  Transforms::InversePark(voltage_d, voltage_q, edeg, &alpha, &beta);
  Transforms::InverseClarke(alpha, beta, &a, &b, &c);
  switch(method)
  {
    case MIDPOINT_CLAMP:
      ZSM::MidpointClamp(&a, &b, &c);
      break;
    case TOP_CLAMP:
      ZSM::TopClamp(&a, &b, &c);
      break;
    case BOTTOM_CLAMP:
      ZSM::BottomClamp(&a, &b, &c);
      break;
    case TOP_BOTTOM_CLAMP:
      ZSM::TopBottomClamp(&a, &b, &c);
      break;
    case SINUSOIDAL:
      ZSM::Sinusoidal(&a, &b, &c);
      break;
  }
}

int SinusoidalController::GetPhaseA()
{
  return a;
}

int SinusoidalController::GetPhaseB()
{
  return b;
}

int SinusoidalController::GetPhaseC()
{
  return c;
}

