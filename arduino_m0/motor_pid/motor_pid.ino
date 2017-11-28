#include "PID_v1.h"

#define LOG_SERIAL    SerialUSB




double enc_speed, goal_speed, pwm_output;
double Kp=20, Ki=5, Kd=0.0;

PID motorPID(&enc_speed, &pwm_output, &goal_speed, Kp, Ki, Kd, DIRECT);




typedef struct
{
  int32_t  counter;

  int32_t  speed;
  int32_t  start_counter;

  int32_t  on_time;
  uint32_t start_time;

  double   pwm_out;

  uint8_t  enc_pin[2];
  uint8_t  mot_pin[2];
} motor_t;


motor_t motor;


void setup()
{
  motor.counter = 0;
  motor.on_time = 8000;
  motor.speed = 0;
  motor.pwm_out = 0;
  motor.enc_pin[0] = 6;
  motor.enc_pin[1] = 7;
  motor.mot_pin[0] = 3;
  motor.mot_pin[1] = 4;

  
  pinMode(motor.enc_pin[0], INPUT_PULLUP);
  pinMode(motor.enc_pin[1], INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(motor.enc_pin[0]), motor_enc_count_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor.enc_pin[1]), motor_enc_count_b, CHANGE);


  analogWrite(motor.mot_pin[0], 0);
  analogWrite(motor.mot_pin[1], 0);


  // PID Setting
  //
  motorPID.SetSampleTime(10);
  motorPID.SetOutputLimits(0, 255);
  motorPID.SetMode(AUTOMATIC);
  goal_speed = 0;

  Serial.begin(115200);
}

void loop()
{
  static uint8_t state = 0;
  static uint32_t pre_time = millis();
  static uint32_t pre_time_log = millis();
  static uint32_t pre_time_delay = millis();
  static uint8_t  pwm = 0;
  static uint32_t pre_time_pid = millis();




  switch(state)
  {
    case 0:
      if (millis()-pre_time_delay >= 3000)
      {
        pre_time_delay = millis();
        state = 1;
        goal_speed = 5;
      }
      break;

    case 1:
      if (millis()-pre_time_delay >= 3000)
      {
        pre_time_delay = millis();
        state = 2;
        goal_speed = 10;
      }
      break;

    case 2:
      if (millis()-pre_time_delay >= 3000)
      {
        pre_time_delay = millis();
        state = 3;
        goal_speed = 15;
      }
      break;
    case 3:
      if (millis()-pre_time_delay >= 3000)
      {
        pre_time_delay = millis();
        state = 4;
        goal_speed = 20;
      }
      break;
    case 4:
      if (millis()-pre_time_delay >= 3000)
      {
        pre_time_delay = millis();
        state = 5;
        goal_speed = 25;
      }
      break;
    case 5:
      if (millis()-pre_time_delay >= 3000)
      {
        pre_time_delay = millis();
        state = 6;
        goal_speed = 40;
      }
      break;
    case 6:
      if (millis()-pre_time_delay >= 3000)
      {
        pre_time_delay = millis();
        state = 7;
        goal_speed = 50;
      }
      break;      
    case 7:
      if (millis()-pre_time_delay >= 3000)
      {
        pre_time_delay = millis();
        state = 7;
        goal_speed = 0;
      }
      break;
  }

  enc_speed = (double)motor.speed;
  if (motorPID.Compute() == true)
  {
    pwm = constrain(pwm_output, 0, 255);

    if (goal_speed == 0) pwm = 0;
    analogWrite(motor.mot_pin[0], pwm);
    //analogWrite(motor.mot_pin[0], 50);
  }

  // Encoder Speed Update
  if (millis()-pre_time >= 10)
  {
    pre_time = millis();
    motor.speed = motor.counter - motor.start_counter;
    motor.start_counter = motor.counter;
  }

  // Log
  if (millis()-pre_time_log >= 50)
  {
    pre_time_log = millis();

    LOG_SERIAL.print(goal_speed);
    LOG_SERIAL.print(" 0 ");
    //Serial.print(motor.on_time);
    LOG_SERIAL.print(" 35 ");
    LOG_SERIAL.print(motor.speed);
    LOG_SERIAL.print("  ");
    //Serial.print(motor.pwm_out);
    //Serial.print(pwm);
    LOG_SERIAL.print("  ");
    //Serial.println(motor.counter);
    //Serial.println(err);
    LOG_SERIAL.println();
  }

}


void motor_enc_count_a(void)
{
  uint8_t enc_bit = 0;

  if (digitalRead(motor.enc_pin[0]) == HIGH)
  {
    enc_bit |= (1<<0);
  }
  if (digitalRead(motor.enc_pin[1]) == HIGH)
  {
    enc_bit |= (1<<1);
  }

  switch(enc_bit)
  {
    case 0x01:
    case 0x02:
      motor.counter--;
      break;

    default:
      motor.counter++;
      break;
  }
}

void motor_enc_count_b(void)
{
  uint8_t enc_bit = 0;

  if (digitalRead(motor.enc_pin[0]) == HIGH)
  {
    enc_bit |= (1<<0);
  }
  if (digitalRead(motor.enc_pin[1]) == HIGH)
  {
    enc_bit |= (1<<1);
  }

  switch(enc_bit)
  {
    case 0x00:
    case 0x03:
      motor.counter--;
      break;

    default:
      motor.counter++;
      break;
  }
}
