/*
 * Zero Sequence Modulation algorithms
 * Inputs are outputs of inverse clarke transformation from transforms..h
 */

#include "math.h"
#define min(arg1,arg2) (arg1<arg2 ? arg1 : arg2)
#define max(arg1,arg2) (arg1>arg2 ? arg1 : arg2)

void sinusoidal(float *a, float *b, float *c)
{
  *a = 3.0 / 4 * (*a) + 1 / 2.0;
  *b = 3.0 / 4 * (*b) + 1 / 2.0;
  *c = 3.0 / 4 * (*c) + 1 / 2.0;
}

void midpoint_clamp(float *a, float *b, float *c)
{
  float shift = 1 / 2.0 - (min(min(*a, *b), *c) + max(max(*a, *b), *c)) / 2;
  *a += shift;
  *b += shift;
  *c += shift;
}

void top_clamp(float *a, float *b, float *c)
{
  float shift = 1 - max(max(*a, *b), *c);
  *a += shift;
  *b += shift;
  *c += shift;
}

void bottom_clamp(float *a, float *b, float *c)
{
  float shift = min(min(*a, *b), *c);
  *a -= shift;
  *b -= shift;
  *c -= shift;
}

void top_bottom_clamp(float *a, float *b, float *c)
{
  if ((*a) * (*b) * (*c) > 0) // Two negatives, largest amplitude positive
  {
    top_clamp(a, b, c);
  }
  else // Two positives, largest amplitude negative
  {
    bottom_clamp(a, b, c);
  }
}


