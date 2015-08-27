#include "sine.h"

#include "pwm.h"

#define PERIPHERAL_BUS_CLOCK 48000000   // Bus Clock 48MHz
#define FTM0_CLK_PRESCALE 0             // FTM0 Prescaler
#define FTM0_OVERFLOW_FREQUENCY 48000   // PWM frequency in Hz
#define FTM0_DEADTIME_DTVAL 1           // Dead Time
#define PWM_MAX (PERIPHERAL_BUS_CLOCK / FTM0_OVERFLOW_FREQUENCY / 2)

int dio = 0;
int clk = 1;
int cs  = 2;
int encoder_index = 7;
int brake_switch = 19;
int pwm[6] = {22, 9, 6, 23, 10, 20};
int pole_pairs = 7;
int factor = 1000;
int zero = 13; // Energize phase 0-1 to zero
int zero_offset;
double duty = 1.0;
const bool sine = true;
const bool trap = true;
int speed = sine ? duty * PWM_MAX : duty * 255;
int offset;
int fudge = 18; // Electrical degrees to advance by
bool reverse = false;
int lastAngle;
unsigned long lastTime;
bool brake = false;
int measured_speed;
int rpm;
const int phase_advance_factor = 500; // rpm/deg

void init_as5134(void){
  digitalWrite(cs, LOW);
  digitalWrite(clk, LOW);
  pinMode(cs,  OUTPUT);
  pinMode(clk, OUTPUT);
  pinMode(dio, INPUT);
}

int read_as5134(void){
  int clock_p;
  unsigned int data=0;
  digitalWrite(cs,LOW);
  digitalWrite(clk,LOW);
  digitalWrite(cs,HIGH);
  for(clock_p=21;clock_p--;clock_p>0){
    digitalWrite(clk,LOW);
    digitalWrite(clk,HIGH);
    if(clock_p<9)
    {
      if(digitalRead(dio))
        data+=(1<<(clock_p));
    }
  }
  digitalWrite(clk,LOW);
  digitalWrite(cs,LOW);
  return data;
}

void power_phase(int in, int out, int duty_cycle)
{
  for (int i = 0; i < 6; i++)
  {
    if (i == in)
      analogWrite(pwm[i], 255 - duty_cycle);
    else if (i == out + 3)
      analogWrite(pwm[i], 0);
    else
      analogWrite(pwm[i], 255);
  }
}

void set_phase_voltages(int u, int v, int w, int duty_cycle)
{
  if (!(u == 0 || v == 0 || w == 0))
  {
    power_phase(-99, -99, 0);
    return;
  }
  PWM_SetDutyCycle((u * duty_cycle) / PWM_MAX, (v * duty_cycle) / PWM_MAX, (w * duty_cycle) / PWM_MAX);
//  if (rpm > 2)
//  {
//    Serial.print(u);
//    Serial.print(",");
//    Serial.print(v);
//    Serial.print(",");
//    Serial.print(w);
//    Serial.print(",");
//    Serial.print(u-v);
//    Serial.print(",");
//    Serial.print(v-w);
//    Serial.print(",");
//    Serial.print(w-u);
//    Serial.print(",");
//    Serial.println(abs(u-v) + abs(v-w) + abs(w-u));
//  }
//  if (u == 0)
//  {
//    analogWrite(pwm[0], 255);
//    analogWrite(pwm[3], 0);
//  }
//  else
//  {
//    analogWrite(pwm[3], 255);
//    analogWrite(pwm[0], 255 - u * duty_cycle / 255);
//  }
//  if (v == 0)
//  {
//    analogWrite(pwm[1], 255);
//    analogWrite(pwm[4], 0);
//  }
//  else
//  {
//    analogWrite(pwm[4], 255);
//    analogWrite(pwm[1], 255 - v * duty_cycle / 255);
//  }
//  if (w == 0)
//  {
//    analogWrite(pwm[2], 255);
//    analogWrite(pwm[5], 0);
//  }
//  else
//  {
//    analogWrite(pwm[5], 255);
//    analogWrite(pwm[2], 255 - w * duty_cycle / 255);
//  }
}

int wrap(int angle)
{
  if (angle < 0)
  {
    angle += 360 * factor;
    return angle;
  }
  while (angle > 360 * factor)
    angle -= 360 * factor;
  return angle;
}

int scale(int angle)
{
  angle %= (360 * factor) / pole_pairs;
  return angle * pole_pairs;
}

void calculate_speed()
{
  unsigned long current = micros();
  int dt = current - lastTime;
  measured_speed = 4 * 1000000 / dt; // deg/s
  rpm = measured_speed * 60 / 360;
  lastTime = current;
}

void brake_trigger_on()
{
  brake = true;
  PWM_BrakeMode();
  detachInterrupt(brake_switch);
  attachInterrupt(brake_switch, brake_trigger_off, FALLING);
}

void brake_trigger_off()
{
  brake = false;
  PWM_BrakeModeEnd();
  detachInterrupt(brake_switch);
  attachInterrupt(brake_switch, brake_trigger_on, RISING);
}

void setup() {
  if (sine)
  {
    if (!reverse)
      offset = (360 * factor) / pole_pairs / 3;
    else
      offset = (360 * factor) / pole_pairs / 6;
  }
  else
  {
    offset = (360 * factor) / pole_pairs / 4;
  }
  init_as5134();
  if (!sine)
  {
    for (int i = 0; i < 6; i++)
    {
      pinMode(pwm[i], OUTPUT);
      analogWriteFrequency(pwm[i], FTM0_OVERFLOW_FREQUENCY);
      analogWrite(pwm[i], 255);
    }
  }
  else if (sine)
  {
    PWMInit(PERIPHERAL_BUS_CLOCK, FTM0_CLK_PRESCALE, FTM0_OVERFLOW_FREQUENCY, FTM0_DEADTIME_DTVAL, false);
  }
  pinMode(brake_switch, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  pinMode(encoder_index, INPUT);
  attachInterrupt(encoder_index, calculate_speed, RISING);
  attachInterrupt(brake_switch, brake_trigger_on, RISING);
}

void loop() {
  static int e = 0;
  if (micros() - lastTime > 500000)
  {
    measured_speed = rpm = 0;
  }
  digitalWrite(LED_BUILTIN, !brake);
  int advance = fudge * factor / pole_pairs + rpm * factor / phase_advance_factor / pole_pairs;
  if (!reverse)
    zero_offset = zero * factor - offset - advance;
  else
    zero_offset = zero * factor + offset + advance;
  int orig = read_as5134();
  int angle = wrap(orig * factor - zero_offset);
  int scaled = scale(angle);
  int step = scaled / (60 * factor);
  if (e++ % 4 == 0)
  {
    //Serial.print("Step: ");
    //Serial.println(step);
    //Serial.println(orig);
  }
  if (!sine)
  {
    switch (step)
    {
      case 0:
        power_phase(0, 1, speed);
        break;
      case 1:
        power_phase(2, 1, speed);
        break;
      case 2:
        power_phase(2, 0, speed);
        break;
      case 3:
        power_phase(1, 0, speed);
        break;
      case 4:
        power_phase(1, 2, speed);
        break;
      case 5:
        power_phase(0, 2, speed);
        break;
      case 6:
        power_phase(-99, -99, 0);
        break;
    }
  }
  else
  {
    if (!brake)
    {
      if (step == 0 || step == 1)
      {
        if (!trap)
          set_phase_voltages(-sintable[(scaled / factor + 240) % 360], 0, sintable[scaled / factor], speed);
        else
          set_phase_voltages(-traptable[(scaled / factor + 240) % 360], 0, traptable[scaled / factor], speed);
      }
      else if (step == 2 || step == 3)
      {
        if (!trap)
          set_phase_voltages(0, sintable[(scaled / factor + 240) % 360], -sintable[(scaled / factor + 120) % 360], speed);
        else
          set_phase_voltages(0, traptable[(scaled / factor + 240) % 360], -traptable[(scaled / factor + 120) % 360], speed);
      }
      else if (step == 4 || step == 5)
      {
        if (!trap)
          set_phase_voltages(sintable[(scaled / factor + 120) % 360], -sintable[scaled / factor], 0, speed);
        else
          set_phase_voltages(traptable[(scaled / factor + 120) % 360], -traptable[scaled / factor], 0, speed);
      }
    }
    else
    {
      PWM_Brake(450);
    }
  }
  if (e % 1000 == 0)
    Serial.println(rpm);
}
