#ifndef SINUSOIDAL_CONTROLLER_H_
#define SINUSOIDAL_CONTROLLER_H_

class SinusoidalController
{
  public:
  typedef enum {
    MIDPOINT_CLAMP,
    TOP_CLAMP,
    BOTTOM_CLAMP,
    TOP_BOTTOM_CLAMP,
    SINUSOIDAL
  } ZSMMethod;
  SinusoidalController(ZSMMethod method);
  void Update(int voltage_d, int voltage_q, int edeg);
  int GetPhaseA();
  int GetPhaseB();
  int GetPhaseC();

  private:
  int a, b, c;
  ZSMMethod method;
};

#endif




