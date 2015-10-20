#include "sine.h"

#include "pwm.h"

#include "transforms.h"
#include "zsm.h"

#define PERIPHERAL_BUS_CLOCK 48000000   // Bus Clock 48MHz
#define FTM0_CLK_PRESCALE 0             // FTM0 Prescaler
#define FTM0_OVERFLOW_FREQUENCY 24000   // PWM frequency in Hz
#define FTM0_DEADTIME_DTVAL 1           // Dead Time
#define PWM_MAX (PERIPHERAL_BUS_CLOCK / FTM0_OVERFLOW_FREQUENCY / 2)

int dio = 0;
int clk = 1;
int cs  = 2;
int encoder_a = 3;
int encoder_b = 4;
int led_forward = 5;
int led_reverse = 11;
int led_error = 12;
int pwrgd = 14;
int fault = 15;
int octw = 16;
int voltage_sense = 3;
int pwm_in = 18;
int dc_cal = 19;
int en_gate = 21;
int so1 = 10;
int so2 = 11;
int pwm[6] = {22, 9, 6, 23, 10, 20};
double duty = 0.0;
int pole_pairs = 7;
int zero = 13; // Energize phase 0-1 to zero
int zero_offset;
const bool sine = true;
const bool trap = false;
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
  static int e = 0;
  return e++ % 360;
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
      analogWrite(pwm[i], duty_cycle);
    else if (i == out + 3)
      analogWrite(pwm[i], 255);
    else
      analogWrite(pwm[i], 0);
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
}

void brake_trigger_off()
{
  brake = false;
  PWM_BrakeModeEnd();
}

void sine_commutate(int speed)
{
  int orig = read_as5134();
  int angle = wrap((orig - zero) * factor);
  int edeg = scale(angle) / factor;
  int alpha, beta, a, b, c;
  inversePark(0, speed, edeg, &alpha, &beta);
  inverseClarke(alpha, beta, &a, &b, &c);
  top_bottom_clamp(&a, &b, &c);
  int avg = (a + b + c) / 3;
  /*Serial.print(edeg);
  Serial.print(",");
  Serial.print(a);
  Serial.print(",");
  Serial.print(b);
  Serial.print(",");
  Serial.print(c);
  Serial.print(",");
  Serial.print(a - avg);
  Serial.print(",");
  Serial.print(b - avg);
  Serial.print(",");
  Serial.print(c - avg);
  Serial.print(",");
  Serial.println(avg);*/
  PWM_SetDutyCycle((a * PWM_MAX) / factor, (b * PWM_MAX) / factor, (c * PWM_MAX) / factor);
}

void step_commutate(int speed)
{
  int advance = fudge * factor / pole_pairs + rpm * factor / phase_advance_factor / pole_pairs;
  if (!reverse)
    zero_offset = zero * factor - offset - advance;
  else
    zero_offset = zero * factor + offset + advance;
  int orig = read_as5134();
  int angle = wrap(orig * factor - zero_offset);
  int scaled = scale(angle);
  int step = scaled / (60 * factor);
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
      analogWrite(pwm[i], 0);
    }
  }
  else if (sine)
  {
    PWMInit(PERIPHERAL_BUS_CLOCK, FTM0_CLK_PRESCALE, FTM0_OVERFLOW_FREQUENCY, FTM0_DEADTIME_DTVAL, false);
  }
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  Serial3.begin(115200);
  pinMode(encoder_a, INPUT);
  pinMode(encoder_b, INPUT);
  pinMode(led_forward, OUTPUT);
  pinMode(led_reverse, OUTPUT);
  pinMode(led_error, OUTPUT);
  pinMode(pwrgd, INPUT_PULLUP);
  pinMode(fault, INPUT_PULLUP);
  pinMode(octw, INPUT_PULLUP);
  pinMode(pwm_in, INPUT);
  pinMode(dc_cal, OUTPUT);
  pinMode(en_gate, OUTPUT);
  attachInterrupt(encoder_a, calculate_speed, RISING);
  digitalWrite(dc_cal, LOW);
  digitalWrite(en_gate, LOW);
}

void loop() {
  if (Serial.available() > 0)
  {
    char b = Serial.read();
      Serial.println(b);
    if (b == '1')
    {
      duty = 0.5;
    }
    else
    {
      duty = 0.0;
    }
  }
  if (duty > 0)
  {
    digitalWrite(en_gate, HIGH);
    digitalWrite(led_forward, HIGH);
  }
  else
  {
    digitalWrite(en_gate, LOW);
    digitalWrite(led_forward, LOW);
  }
  int speed = sine ? duty * PWM_MAX : duty * 255;
  static int e = 0;
  if (micros() - lastTime > 500000)
  {
    measured_speed = rpm = 0;
  }
  if (sine)
  {
    sine_commutate(speed);
  }
  else
  {
    step_commutate(speed);
  }
  if (digitalRead(octw) == LOW || digitalRead(fault) == LOW)
  {
    digitalWrite(led_error, HIGH);
  }
  else
  {
    digitalWrite(led_error, LOW);
  }
}
