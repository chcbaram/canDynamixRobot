/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "motor.h"

#include "src/PID_v1.h"
#include "src/timer_zero.h"



#define MOT_L_PWM_P     3
#define MOT_L_PWM_M     4
#define ENC_L_A         2
#define ENC_L_B         5

#define MOT_R_PWM_P     6
#define MOT_R_PWM_M     8
#define ENC_R_A         7
#define ENC_R_B         9





typedef struct
{
  int32_t  counter;

  int32_t  speed;
  int32_t  start_counter;

  double   pwm_out;

  uint8_t  enc_pin[2];
  uint8_t  mot_pin[2];

  int8_t   mot_dir;

  // PID
  double enc_speed;
  double goal_speed;
  double pwm_output;
  PID   *p_pid;
} motor_cfg_t;


motor_cfg_t motor_cfg[2];


static void motorUpdateISR(void);


uint8_t motorGetEncBit(motor_cfg_t *p_motor);
void motorUpdateEncPhaseA(motor_cfg_t *p_motor);
void motorUpdateEncPhaseB(motor_cfg_t *p_motor);

void motorLeftEncPhaseA_ISR(void);
void motorLeftEncPhaseB_ISR(void);
void motorRightEncPhaseA_ISR(void);
void motorRightEncPhaseB_ISR(void);





void motorBegin(void)
{
  motor_cfg[L_MOTOR].mot_dir    = 1;            // 1 or -1
  motor_cfg[L_MOTOR].mot_pin[0] = MOT_L_PWM_M;
  motor_cfg[L_MOTOR].mot_pin[1] = MOT_L_PWM_P;
  motor_cfg[L_MOTOR].enc_pin[0] = ENC_L_A;      // Interrupt Pin
  motor_cfg[L_MOTOR].enc_pin[1] = ENC_L_B;      // Interrupt Pin


  motor_cfg[R_MOTOR].mot_dir    = 1;            // 1 or -1
  motor_cfg[R_MOTOR].mot_pin[0] = MOT_R_PWM_P;
  motor_cfg[R_MOTOR].mot_pin[1] = MOT_R_PWM_M;
  motor_cfg[R_MOTOR].enc_pin[0] = ENC_R_B;      // Interrupt Pin
  motor_cfg[R_MOTOR].enc_pin[1] = ENC_R_A;      // Interrupt Pin


  for (int i=0; i<2; i++)
  {
    motor_cfg[i].counter = 0;
    motor_cfg[i].speed   = 0;
    motor_cfg[i].pwm_out = 0;
    motor_cfg[i].p_pid   = new PID(&motor_cfg[i].enc_speed, &motor_cfg[i].pwm_output, &motor_cfg[i].goal_speed, 20, 5, 0, DIRECT);
    motor_cfg[i].start_counter = 0;

    pinMode(motor_cfg[i].enc_pin[0], INPUT_PULLUP);
    pinMode(motor_cfg[i].enc_pin[1], INPUT_PULLUP);

    analogWrite(motor_cfg[i].mot_pin[0], 0);
    analogWrite(motor_cfg[i].mot_pin[1], 0);

    motor_cfg[i].p_pid->SetSampleTime(10);
    motor_cfg[i].p_pid->SetOutputLimits(-255, 255);
    motor_cfg[i].p_pid->SetMode(AUTOMATIC);
  }

  attachInterrupt(digitalPinToInterrupt(motor_cfg[L_MOTOR].enc_pin[0]),  motorLeftEncPhaseA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor_cfg[L_MOTOR].enc_pin[1]),  motorLeftEncPhaseB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor_cfg[R_MOTOR].enc_pin[0]), motorRightEncPhaseA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor_cfg[R_MOTOR].enc_pin[1]), motorRightEncPhaseB_ISR, CHANGE);
  

  timerZeroBegin();
  timerZeroSet(TIMER_ZERO_CH1, 10, motorUpdateISR);
  timerZeroStart(TIMER_ZERO_CH1);
}

int32_t motorGetSpeed(uint8_t ch)
{
  return motor_cfg[ch].speed;
}

int32_t motorGetCounter(uint8_t ch)
{
  return motor_cfg[ch].counter;
}

int32_t motorGetGoalSpeed(uint8_t ch)
{
  return (int32_t)motor_cfg[ch].goal_speed;
}

void motorSetSpeed(uint8_t ch, int16_t speed)
{
  motor_cfg[ch].goal_speed = (double)speed;
}

void motorMoveSpeed(int16_t left_speed, int16_t right_speed)
{
  motor_cfg[L_MOTOR].goal_speed = (double)left_speed;
  motor_cfg[R_MOTOR].goal_speed = (double)right_speed;
}


void motorSetPwm(uint8_t ch, int16_t pwm_data )
{
  uint16_t pwm_out;

  if (pwm_data >= 0)
  {
    pwm_out = pwm_data;
    analogWrite(motor_cfg[ch].mot_pin[0], pwm_out);
    analogWrite(motor_cfg[ch].mot_pin[1], 0);
  }
  else
  {
    pwm_out = -pwm_data;
    analogWrite(motor_cfg[ch].mot_pin[0], 0);
    analogWrite(motor_cfg[ch].mot_pin[1], pwm_out);
  }
}

// Motor Update
//
void motorUpdateISR(void)
{
  for (int i=0; i<2; i++)
  {
    motor_cfg[i].speed = motor_cfg[i].counter - motor_cfg[i].start_counter;
    motor_cfg[i].start_counter = motor_cfg[i].counter;
    motor_cfg[i].enc_speed = (double)motor_cfg[i].speed;

    if (motor_cfg[i].p_pid->ComputeISR())
    {
      if (motor_cfg[i].goal_speed == 0)
      {
        motorSetPwm(i, 0);
      }
      else
      {
        motorSetPwm(i, (int16_t)motor_cfg[i].pwm_output);
      }
    }
  }
}



void motorLeftEncPhaseA_ISR(void)
{
  motorUpdateEncPhaseA(&motor_cfg[L_MOTOR]);
}

void motorLeftEncPhaseB_ISR(void)
{
  motorUpdateEncPhaseB(&motor_cfg[L_MOTOR]);
}

void motorRightEncPhaseA_ISR(void)
{
  motorUpdateEncPhaseA(&motor_cfg[R_MOTOR]);
}

void motorRightEncPhaseB_ISR(void)
{
  motorUpdateEncPhaseB(&motor_cfg[R_MOTOR]);
}


uint8_t motorGetEncBit(motor_cfg_t *p_motor)
{
  uint8_t enc_bit = 0;
  
  if (digitalRead(p_motor->enc_pin[0]) == HIGH)
  {
    enc_bit |= (1<<0);
  }
  if (digitalRead(p_motor->enc_pin[1]) == HIGH)
  {
    enc_bit |= (1<<1);
  }

  return enc_bit;
}

void motorUpdateEncPhaseA(motor_cfg_t *p_motor)
{
  switch(motorGetEncBit(p_motor))
  {
    case 0x01:
    case 0x02:
      p_motor->counter--;
      break;

    default:
      p_motor->counter++;
      break;
  }
}

void motorUpdateEncPhaseB(motor_cfg_t *p_motor)
{
  switch(motorGetEncBit(p_motor))
  {
    case 0x00:
    case 0x03:
      p_motor->counter--;
      break;

    default:
      p_motor->counter++;
      break;
  }
}
