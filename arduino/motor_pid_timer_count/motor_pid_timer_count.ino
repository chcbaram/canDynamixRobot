#include "motor.h"



void setup() 
{
  motorBegin();
  Serial.begin(115200);

  motorMoveSpeed(0, 0);
  delay(1000);
  motorMoveSpeed(0, 0);
}

void loop() 
{

  Serial.print(motorGetCounter(L_MOTOR));
  Serial.print(" ");
  Serial.println(motorGetCounter(R_MOTOR));
  delay(20);
}

