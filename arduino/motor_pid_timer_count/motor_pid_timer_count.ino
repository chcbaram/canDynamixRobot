#include "motor.h"



#define ENCODER_RES     1400.
#define WHEEL_LENGTH    (34.*PI)

int32_t go_count;

void setup() 
{
  motorBegin();
  Serial.begin(115200);

  motorMoveSpeed(0, 0);
  delay(1000);
  motorMoveSpeed(0, 0);

  go_count = (int32_t)(100. * 1400. / WHEEL_LENGTH);


}

void loop() 
{

  Serial.print(motorGetCounter(L_MOTOR));
  Serial.print(" ");
  Serial.println(motorGetCounter(R_MOTOR));
  delay(20);
  /*
  if (motorGetCounter(L_MOTOR) >= go_count)
  {
    motorMoveSpeed(0, 0);
    Serial.println(motorGetCounter(R_MOTOR));
    delay(100);    
  }
  */

}

