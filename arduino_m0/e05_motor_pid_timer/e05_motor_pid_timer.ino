#include "motor.h"

#define LOG_SERIAL    SerialUSB





void setup() 
{


  motorBegin();
  //motorSetSpeed(L_MOTOR, 10);
  motorMoveSpeed(10, 10);
  LOG_SERIAL.begin(115200);
}

void loop() 
{

  
  LOG_SERIAL.print(motorGetGoalSpeed(L_MOTOR));
  LOG_SERIAL.print(" 0 30 ");
  LOG_SERIAL.println(motorGetSpeed(L_MOTOR));
  delay(50);
}

