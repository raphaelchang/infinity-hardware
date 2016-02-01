#include "AS5134.h"

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

int AS5134::Read(){
  
  int clock_p;
  unsigned int data=0;
  digitalWrite(cs,LOW);
  digitalWrite(clk,LOW);
  digitalWrite(cs,HIGH);
  for(clock_p=21;clock_p--;clock_p>0){
    digitalWrite(clk,LOW);
    digitalWrite(clk,HIGH);
    if(clock_p<9){
      if(digitalRead(dio))
        data+=(1<<(clock_p));
    }
  }
  digitalWrite(clk,LOW);
  digitalWrite(cs,LOW);
  return (360 - data) % 360;
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

