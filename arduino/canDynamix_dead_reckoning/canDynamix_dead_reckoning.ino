#include "src/motor.h"
#include <math.h>


#define WHEEL_RADIUS      (0.034/2)                           // 바퀴 반지름(meter)
#define WHEEL_SEPARATION  0.083                               // 바퀴간 거리(meter)

#define ENCODER_RES       1400                                // 1바퀴 회전시 엔코더 펄스 수 
#define TICK2RAD          (((360./ENCODER_RES) * PI) / 180.)  // 엔코더 1펄스일때 회전하는 각도(라디안) 

#define DEG2RAD(x)        (x * 0.01745329252)                 // *PI/180
#define RAD2DEG(x)        (x * 57.2957795131)                 // *180/PI


float mTheta = 0;
float mX = 0;
float mY = 0; 

int32_t pre_encoder_cnt[2] = {0,};


void setup() 
{
  motorBegin();
  Serial.begin(115200);

  motorMoveSpeed(0, 0);
  delay(1000);
}

void loop() 
{
  static uint32_t pre_time;
  uint32_t process_time;

  int32_t cur_encoder_cnt[2];

  cur_encoder_cnt[L_MOTOR] = motorGetCounter(L_MOTOR);
  cur_encoder_cnt[R_MOTOR] = motorGetCounter(R_MOTOR);

  pre_time = micros();
  encoderUpdate(cur_encoder_cnt[L_MOTOR] - pre_encoder_cnt[L_MOTOR], cur_encoder_cnt[R_MOTOR] - pre_encoder_cnt[R_MOTOR]);
  process_time = micros() - pre_time;

  pre_encoder_cnt[L_MOTOR] = cur_encoder_cnt[L_MOTOR];
  pre_encoder_cnt[R_MOTOR] = cur_encoder_cnt[R_MOTOR];

  delay(50);
  
  Serial.print("x : " + String(mX * 100));
  Serial.print(" ");
  Serial.print("y : " + String(mY * 100));
  Serial.print(" ");

  Serial.print("theta : " + String(RAD2DEG(mTheta)));  
  Serial.print("t : " + String(process_time));
  Serial.print(" ");
  
  Serial.println(" ");

  if (Serial.available())
  {
    uint8_t ch;

    ch = Serial.read();
    
    if (ch == 'o')
    {
      motorMoveSpeed(5, 5);
    }
    else if (ch == 'l')
    {
      motorMoveSpeed(-5, -5);
    }
    else if (ch == 'k')
    {
      motorMoveSpeed(0, 5);
    }    
    else if (ch == ';')
    {
      motorMoveSpeed(5, 0);
    }    
    else
    {
      motorMoveSpeed(0, 0);
    }

  }
}

void encoderUpdate(float cnt_l, float cnt_r)
{
  float wheel_l, wheel_r; 
  float delta_s, delta_theta;

  wheel_l = TICK2RAD * cnt_l;
  wheel_r = TICK2RAD * cnt_r;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;


  mX     += delta_s * cos(mTheta + (delta_theta / 2.0));
  mY     += delta_s * sin(mTheta + (delta_theta / 2.0));
  mTheta += delta_theta;

  // mTheta : -180 ~ 180
  if (mTheta > PI)
  {
    mTheta = mTheta - 2.*PI;
  }
  if (mTheta < -PI)
  {
    mTheta = mTheta + 2.*PI;
  }
}

