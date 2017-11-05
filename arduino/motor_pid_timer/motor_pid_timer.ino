#include "motor.h"





void setup() 
{


  motorBegin();
  //motorSetSpeed(L_MOTOR, 10);
  motorMoveSpeed(10, 10);
  Serial.begin(115200);
}

void loop() 
{

  
  Serial.print(motorGetGoalSpeed(L_MOTOR));
  Serial.print(" 0 30 ");
  Serial.println(motorGetSpeed(L_MOTOR));
  delay(50);
}

