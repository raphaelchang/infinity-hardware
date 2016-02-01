#include "pwm.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "AS5045.h"
#include "SinusoidalController.h"
#include "FieldOrientedController.h"
#include "Transforms.h"
#include "Util.h"
#include <TimerOne.h>
#include <string>

#define PERIPHERAL_BUS_CLOCK 48000000   // Bus Clock 48MHz
#define FTM0_CLK_PRESCALE 0             // FTM0 Prescaler
#define FTM0_OVERFLOW_FREQUENCY 25000   // PWM frequency in Hz
#define FTM0_DEADTIME_DTVAL 32           // Dead Time
#define PWM_MAX (PERIPHERAL_BUS_CLOCK / FTM0_OVERFLOW_FREQUENCY / 2)
//#define MODULATE
//#define PRINT

AS5134 *as5134;
SinusoidalController *sineController;
FieldOrientedController *focController;
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
int conv = 17;
int voltage_sense = A10;
int so1 = A4;
int so2 = A5;
int reset = 21;
int pwm[6] = {22, 9, 6, 23, 10, 20};
int duty = 0;
int speed = 0;
int zero = 344000;
int orig_zero;
int mod_dir = 1;
int zero_offset;
const bool sine = true;
const bool trap = false;
int offset;
int fudge = 0; // Electrical degrees to advance by
bool reverse = false;
bool brake = false;
int current_a_raw = 50000;
int current_b_raw = 50000;
int current_a;
int current_b;
int current_c;
int current_d;
int current_q;
int current_alpha;
int current_beta;
int blink_interval = 5000;
int blink_counter = 0;
int lastangle;
int glitchcount = 0;
int read_phase = 0;
boolean phase_a_high = false;
boolean phase_b_high = false;
char new_command[10];
int new_index = 0;
boolean active = false;
int lastReceivedTime;

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

void calculate_current(int edeg)
{
  int a = current_a_raw;
  int b = current_b_raw;
  a -= 49600; // Offset
  b -= 49800;
  a *= 3300; // Scale to 1000 * voltage
  a /= 65536;
  b *= 3300;
  b /= 65536;
  a *= 1000; // Divide by resistance
  b *= 1000;
  a /= 20; // Divide by gain
  b /= 20;
  int c = -(a + b);
  current_a = a;
  current_b = c;
  current_c = b;
  Transforms::Clarke(current_a, current_b, current_c, &current_alpha, &current_beta);
  Transforms::Park(current_alpha, current_beta, edeg, &current_d, &current_q);
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
    sineController = new SinusoidalController(SinusoidalController::SINUSOIDAL);
    focController = new FieldOrientedController(SinusoidalController::SINUSOIDAL, 1, 1);
    PWMInit(PERIPHERAL_BUS_CLOCK, FTM0_CLK_PRESCALE, FTM0_OVERFLOW_FREQUENCY, FTM0_DEADTIME_DTVAL, false);
    PWM_DisableInterrupts();
    PWM_EnableInterrupt(0);
  }
  analogReference(DEFAULT);
  analogReadResolution(16);
  analogReadAveraging(1);
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
  pinMode(conv, OUTPUT);
  digitalWrite(reset, LOW);
  lastangle = as5134->Read();
  orig_zero = zero;
  lastReceivedTime = millis();
}

void ftm0_isr(void)
{
    if (FTM0_C0SC & 0x80 || FTM0_C1SC & 0x80)
    {
      if (read_phase % 10 == 0)
      {
        delayMicroseconds(5);
        digitalWrite(conv, HIGH);
        current_a_raw = analogRead(so1);
        digitalWrite(conv, LOW);
        PWM_DisableInterrupts();
        if (phase_b_high)
          PWM_EnableInterrupt(4);
        else
          PWM_EnableInterrupt(5);
      }
      else
      {
        if (FTM0_C1SC & 0x80)
        {
          PWM_ClearInterrupt(1);
          //PWM_EnableInterrupt(1);
        }
        else if (FTM0_C0SC & 0x80)
        {
          PWM_ClearInterrupt(0);
          //PWM_EnableInterrupt(0);
        }
      }
      read_phase++;
    }
    else if (FTM0_C4SC & 0x80 || FTM0_C5SC & 0x80)
    {
      if (read_phase % 10 == 0)
      {
        delayMicroseconds(5);
        digitalWrite(conv, HIGH);
        current_b_raw = analogRead(so2);
        digitalWrite(conv, LOW);
        PWM_DisableInterrupts();
        if (phase_a_high)
          PWM_EnableInterrupt(0);
        else
          PWM_EnableInterrupt(1);
      }
      else
      {
        if (FTM0_C4SC & 0x80)
        {
          PWM_ClearInterrupt(4);
          //PWM_EnableInterrupt(4);
        }
        else if (FTM0_C5SC & 0x80)
        {
          PWM_ClearInterrupt(5);
          //PWM_EnableInterrupt(5);
        }
      }
      read_phase++;
    }
}

void loop() {
  int startTime = micros();
  #ifdef MODULATE
  static int f = 0;
  if (f++ % 100 == 0)
  {
    zero += mod_dir * 100;
    if (zero - orig_zero > 400 || zero - orig_zero < -400)
    {
      mod_dir *= -1;
    }
  }
  #endif
  if (millis() - lastReceivedTime > 500)
  {
    active = false;
    duty = 0;
  }
  if (Serial3.available() > 0)
  {
    char b = Serial3.read();
      Serial.println(b);
      if (b == '\n')
      {
        if (new_command[1] == 'z')
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
          int oorig = as5134->Read();
          int orig = oorig * 360 * factor / 4096;
          Serial.println(orig);
          PWM_SetDutyCycle(0,0,0);
        }
        else if (new_command[1] == 'c')
        {
          active = false;
          duty = 0;
        }
        else
        {
          new_command[new_index] = '\0';
          int new_duty = atoi(new_command);
          duty = new_duty - 500;
          active = true;
        }
          new_index = 0;
          lastReceivedTime = millis();
      }
      new_command[new_index++] = b;
    
//    else
//    {
//      int oorig = as5134->Read();
//      int orig = oorig * 360 * factor / 4096;
//      Serial.println(orig);
//      duty = 0;
//    }
  }
  digitalWrite(reset, active);
  if (duty > 0 || duty < 0)
  {
    int led = duty > 0 ? led_forward : led_reverse;
    digitalWrite(led, HIGH);
    blink_counter++;
    if (blink_counter > blink_interval)
    {
      blink_counter = 0;
    }
    if (blink_counter > (blink_interval * duty) / factor)
    {
      digitalWrite(led, LOW);
    }
  }
  else
  {
    digitalWrite(led_forward, LOW);
    digitalWrite(led_reverse, LOW);
  }
  if (sine)
  {
    if (speed < duty)
    {
      speed++;
    }
    else if (speed > duty)
    {
      speed--;
    }
  }
  static int e = 0;
  if (sine)
  {
//    Serial.print(a);
//    Serial.print(",");
//    Serial.print(b);
//    Serial.print(",");
//    Serial.println(c);
    if (!active)
    {
    //Serial.println(orig);
      PWM_SetDutyCycle(0,0,0);
    }
    else
    {
    int oorig = as5134->Read();
    int orig = oorig * 360 * factor / 4096;
    Serial.println(orig);
    int angle = Util::Wrap(orig - zero);
    int edeg = Util::Scale(angle) / factor;
    calculate_current(edeg);
    //focController->Update(speed * 100, 0, current_a, current_b, current_c, edeg);
    sineController->Update(0, speed, edeg);
    int a = sineController->GetPhaseA();
    int b = sineController->GetPhaseB();
    int c = sineController->GetPhaseC();
    #ifdef MODULATE
      Serial.print(zero);
      Serial.print(",");
    #endif
    #ifdef PRINT
      Serial.print(edeg * 10);
      Serial.print(",");
//      Serial.print(a);
//      Serial.print(",");
//      Serial.print(b);
//      Serial.print(",");
//      Serial.print(c);
//      Serial.print(",");
      Serial.print(current_a);
      Serial.print(",");
      Serial.print(current_b);
      Serial.print(",");
      Serial.print(current_c);
      Serial.print(",");
      Serial.print(current_d);
      Serial.print(",");
      Serial.println(current_q);
    #endif
      phase_a_high = (a * PWM_MAX) / factor > PWM_MAX / 2;
      phase_b_high = (c * PWM_MAX) / factor > PWM_MAX / 2;
      PWM_SetDutyCycle((a * PWM_MAX) / factor, (b * PWM_MAX) / factor, (c * PWM_MAX) / factor);
    }
  }
  else
  {
    step_commutate(speed);
  }
  if (digitalRead(ff1) == HIGH || digitalRead(ff2) == HIGH)
  {
    //Serial.print(digitalRead(ff1));
    //Serial.println(digitalRead(ff2));
//    pinMode(ff2, OUTPUT);
//    for (int i = 0; i < 10; i++)
//    {
//      digitalWrite(ff2, LOW);
//      delayMicroseconds(1);
//      Serial.print(digitalRead(ff1));
//      digitalWrite(ff2, HIGH);
//      delayMicroseconds(1);
//    }
//    pinMode(ff2, INPUT);
    digitalWrite(led_error, HIGH);
  }
  else
  {
    digitalWrite(led_error, LOW);
  }
  if (speed > 0) 
  Serial.println(micros() - startTime);
}






