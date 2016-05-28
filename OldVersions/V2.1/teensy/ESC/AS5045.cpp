#include "AS5045.h"

unsigned long AS5134::lastTime = 0;
int AS5134::measured_speed = 0;
int AS5134::rpm = 0;

AS5134::AS5134(int cs, int clk, int dio, int quad_a, int quad_b, int minPeriod)
{
  digitalWrite(cs, LOW);
  digitalWrite(clk, LOW);
  pinMode(cs,  OUTPUT);
  pinMode(clk, OUTPUT);
  pinMode(dio, INPUT);
  *portConfigRegister(dio) |= PORT_PCR_PE; //pull enable
  *portConfigRegister(dio) &= ~PORT_PCR_PS; //pull down
  pinMode(quad_a, INPUT);
  pinMode(quad_b, INPUT);
  this->cs = cs;
  this->clk = clk;
  this->dio = dio;
  this->quad_a = quad_a;
  this->quad_b = quad_b;
  this->minPeriod = minPeriod;
  attachInterrupt(quad_a, AS5134::CalculateSpeed, RISING);
  Timer1.initialize(minPeriod);
  Timer1.start();
  Timer1.attachInterrupt(AS5134::ResetSpeed);
}

int AS5134::Read() {
  int clock_p;
  unsigned int data = 0 ;
  digitalWrite (cs, LOW) ;
  for (byte i = 0 ; i < 12 ; i++)
  {
    digitalWrite (clk, LOW) ;
    for (int i = 0; i < 4; i++)
      __asm__("nop\n\t");
    digitalWrite (clk, HIGH) ;
    for (int i = 0; i < 10; i++)
      __asm__("nop\n\t");
    data = (data << 1) | digitalRead (dio) ;
  }
  byte status = 0;
  for (byte i = 0 ; i < 6 ; i++)
  {
    digitalWrite (clk, LOW) ;
    for (int i = 0; i < 4; i++)
      __asm__("nop\n\t");
    digitalWrite (clk, HIGH) ;
    for (int i = 0; i < 10; i++)
      __asm__("nop\n\t");
    status = (status << 1) | digitalRead (dio) ;
  }
  this->status = status >> 1;
  digitalWrite (cs, HIGH);
  return (4096 - data) % 4096;
}

int AS5134::GetRPM()
{
  return rpm;
}

void AS5134::CalculateSpeed()
{
  unsigned long current = micros();
  int dt = current - lastTime;
  measured_speed = 4 * 1000000 / dt; // deg/s
  rpm = measured_speed * 60 / 360;
  lastTime = current;
  Timer1.restart();
}

void AS5134::ResetSpeed()
{
  measured_speed = rpm = 0;
}

byte AS5134::GetStatus()
{
  return status;
}



