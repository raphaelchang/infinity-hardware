/*
 * Clarke and Park transforms and inverses
 */
 
#include "math.h"

void park(float alpha, float beta, float theta, float *d, float *q)
{
  *d = alpha * cos(theta) + beta * sin(theta);
  *q = beta * cos(theta) - alpha * sin(theta);
}

void inversePark(float d, float q, float theta, float *alpha, float *beta)
{
  *alpha = d * cos(theta) - q * sin(theta);
  *beta = d * sin(theta) + q * cos(theta);
}

void clarke(float a, float b, float c, float *alpha, float *beta)
{
  *alpha = 3/2.0 * a;
  *beta = sqrt(3) / 2 * b - sqrt(3) / 2 * c;
}

void inverseClarke(float alpha, float beta, float *a, float *b, float *c)
{
  *a = 2/3.0 * alpha;
  *b = -1/3.0 * alpha + 1 / (sqrt(3)) * beta;
  *c = -1/3.0 * alpha - 1 / (sqrt(3)) * beta;
}
