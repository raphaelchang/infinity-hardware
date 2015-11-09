#ifndef ZSM_H_
#define ZSM_H_

/*
 * Zero Sequence Modulation algorithms - generates PWM duty cycle from clarke transform (phase voltages, which have amplitude 1/sqrt(3) of the bus voltage)
 * Inputs are outputs of inverse clarke transformation from transforms.h
 * Resulting sinusoidal phase voltage (referenced to neutral) is phase voltage referenced to ground (duty cycle) minus neutral voltage (average), and have amplitude 1/sqrt(3) of bus voltage
 */

#include "math.h"
#include "Util.h"
#define min(arg1,arg2) (arg1<arg2 ? arg1 : arg2)
#define max(arg1,arg2) (arg1>arg2 ? arg1 : arg2)
#define factor 1000

class ZSM
{
  public:
  static void Sinusoidal(int *a, int *b, int *c)
  {
    *a = (866 * (*a)) / factor + factor / 2;
    *b = (866 * (*b)) / factor + factor / 2;
    *c = (866 * (*c)) / factor + factor / 2;
  }
  
  static void MidpointClamp(int *a, int *b, int *c)
  {
    int shift = (factor - (min(min(*a, *b), *c) + max(max(*a, *b), *c))) / 2;
    *a += shift;
    *b += shift;
    *c += shift;
  }
  
  static void TopClamp(int *a, int *b, int *c)
  {
    int shift = factor - max(max(*a, *b), *c);
    *a += shift;
    *b += shift;
    *c += shift;
  }
  
  static void BottomClamp(int *a, int *b, int *c)
  {
    int shift = min(min(*a, *b), *c);
    *a -= shift;
    *b -= shift;
    *c -= shift;
  }
  
  static void TopBottomClamp(int *a, int *b, int *c)
  {
    if ((*a) * (*b) * (*c) > 0) // Two negatives, largest amplitude positive
    {
      TopClamp(a, b, c);
    }
    else // Two positives, largest amplitude negative
    {
      BottomClamp(a, b, c);
    }
  }
};


#endif

