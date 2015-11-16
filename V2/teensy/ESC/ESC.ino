#include "pwm.h"

#include "AS5134.h"
#include "SinusoidalController.h"
#include "Transforms.h"
#include "Util.h"
#include <TimerOne.h>

#define PERIPHERAL_BUS_CLOCK 48000000   // Bus Clock 48MHz
#define FTM0_CLK_PRESCALE 0             // FTM0 Prescaler
#define FTM0_OVERFLOW_FREQUENCY 32000   // PWM frequency in Hz
#define FTM0_DEADTIME_DTVAL 12           // Dead Time
#define PWM_MAX (PERIPHERAL_BUS_CLOCK / FTM0_OVERFLOW_FREQUENCY / 2)

AS5134 *as5134;
SinusoidalController *sineController;
int dio = 0;
int clk = 1;
int cs  = 2;
int encoder_a = 3;
int encoder_b = 4;
int led_forward = 5;
int led_reverse = 11;
int led_error = 12;
int pwm_in = 14;
int ff1 = 15;
int ff2 = 16;
int voltage_sense = A3;
int so1 = A4;
int so2 = A5;
int reset = 21;
int pwm[6] = {22, 9, 6, 23, 10, 20};
double duty = 0.0;
int zero = 180; // Energize phase 0-1 to zero
int zero_offset;
const bool sine = true;
const bool trap = false;
int offset;
int fudge = 18; // Electrical degrees to advance by
bool reverse = false;
bool brake = false;
int current_a;
int current_b;
int current_c;
int current_d;
int current_q;
int blink_interval = 1000;
int blink_counter = 0;

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

void measure_current()
{
  int a = analogRead(so1);
  int b = analogRead(so2);
  a -= 65536; // Offset
  b -= 65536;
  a *= 3300; // Scale to 1000 * voltage
  a /= 65536;
  b *= 3300;
  b /= 65536;
  a *= 1000; // Divide by resistance
  b *= 1000;
  a /= 10; // Divide by gain
  b /= 10;
  int c = -(a + b);
  current_a = a;
  current_b = b;
  current_c = c;
  int orig = as5134->Read();
  int angle = Util::Wrap((orig - zero) * factor);
  int edeg = Util::Scale(angle) / factor;
  int alpha, beta;
  Transforms::Clarke(a, b, c, &alpha, &beta);
  Transforms::Park(alpha, beta, edeg, &current_d, &current_q);
}

void step_commutate(int speed)
{
  int advance = fudge * factor / pole_pairs;
  if (!reverse)
    zero_offset = zero * factor - offset - advance;
  else
    zero_offset = zero * factor + offset + advance;
  int orig = as5134->Read();
  int angle = Util::Wrap(orig * factor - zero_offset);
  int scaled = Util::Scale(angle);
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
  as5134 = new AS5134(cs, clk, dio, encoder_a, encoder_b, 500000);
  if (!sine)
  {
    offset = (360 * factor) / pole_pairs / 4;
    for (int i = 0; i < 6; i++)
    {
      pinMode(pwm[i], OUTPUT);
      analogWriteFrequency(pwm[i], FTM0_OVERFLOW_FREQUENCY);
      analogWrite(pwm[i], 0);
    }
  }
  else if (sine)
  {
    sineController = new SinusoidalController(SinusoidalController::MIDPOINT_CLAMP);
    PWMInit(PERIPHERAL_BUS_CLOCK, FTM0_CLK_PRESCALE, FTM0_OVERFLOW_FREQUENCY, FTM0_DEADTIME_DTVAL, false);
  }
  analogReference(DEFAULT);
  analogReadResolution(16);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  Serial3.begin(115200);
  pinMode(led_forward, OUTPUT);
  pinMode(led_reverse, OUTPUT);
  pinMode(led_error, OUTPUT);
  pinMode(ff1, INPUT_PULLUP);
  pinMode(ff2, INPUT_PULLUP);
  pinMode(pwm_in, INPUT);
  pinMode(reset, OUTPUT);
  pinMode(so1, INPUT);
  pinMode(so2, INPUT);
  digitalWrite(reset, LOW);
}

void loop() {
  measure_current();
  digitalWrite(reset, HIGH);
  if (Serial.available() > 0)
  {
    char b = Serial.read();
      Serial.println(b);
    if (b == '1')
    {
      duty = 0.3;
    }
    else if (b == '2')
    {
      sineController->Update(factor / 4, 0, 0);
      int a = sineController->GetPhaseA();
      int b = sineController->GetPhaseB();
      int c = sineController->GetPhaseC();
//      Serial.print(a);
//      Serial.print(",");
//      Serial.print(b);
//      Serial.print(",");
//      Serial.println(c);
      PWM_SetDutyCycle((a * PWM_MAX) / factor, (b * PWM_MAX) / factor, (c * PWM_MAX) / factor);
      delay(100);
      int orig = as5134->Read();
      Serial.println(orig);
      PWM_SetDutyCycle(0,0,0);
    }
    else
    {
      duty = 0.0;
    }
  }
  if (duty > 0 || duty < 0)
  {
    int led = duty > 0 ? led_forward : led_reverse;
    digitalWrite(led, HIGH);
    blink_counter++;
    if (blink_counter > blink_interval)
    {
      blink_counter = 0;
    }
    if (blink_counter > blink_interval * duty)
    {
      digitalWrite(led, LOW);
    }
  }
  else
  {
    digitalWrite(led_forward, LOW);
    digitalWrite(led_reverse, LOW);
  }
  int speed = sine ? duty * factor : duty * 255;
  static int e = 0;
  if (sine)
  {
    int orig = as5134->Read();
    int angle = Util::Wrap((orig - zero) * factor);
    int edeg = Util::Scale(angle) / factor;
    sineController->Update(0, speed, edeg);
    int a = sineController->GetPhaseA();
    int b = sineController->GetPhaseB();
    int c = sineController->GetPhaseC();
//    Serial.print(a);
//    Serial.print(",");
//    Serial.print(b);
//    Serial.print(",");
//    Serial.println(c);
    if (speed == 0)
    {
      PWM_SetDutyCycle(0,0,0);
    }
    else
    {
      Serial.print(current_a);
      Serial.print(",");
      Serial.print(current_b);
      Serial.print(",");
      Serial.println(current_c);
      PWM_SetDutyCycle((a * PWM_MAX) / factor, (b * PWM_MAX) / factor, (c * PWM_MAX) / factor);
    }
  }
  else
  {
    step_commutate(speed);
  }
  if (digitalRead(ff1) == HIGH || digitalRead(ff2) == HIGH)
  {
    Serial.print(digitalRead(ff1));
    Serial.println(digitalRead(ff2));
    digitalWrite(led_error, HIGH);
  }
  else
  {
    digitalWrite(led_error, LOW);
  }
}


