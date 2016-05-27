#ifndef AS5134_H_
#define AS5134_H_
#include "Arduino.h"
#include <TimerOne.h>

class AS5134
{
  public:
  AS5134(int cs, int clk, int dio, int quad_a, int quad_b, int minPeriod);
  int Read();
  int GetRPM();
  byte GetStatus();

  private:
  static void CalculateSpeed();
  static void ResetSpeed();
  int cs;
  int clk;
  int dio;
  int quad_a;
  int quad_b;
  int minPeriod;
  static unsigned long lastTime;
  static int measured_speed;
  static int rpm;
  byte status;
};

#endif




