#include "motor.h"





void setup() 
{
  motorBegin();
  Serial.begin(115200);

  motorMoveSpeed(0, 0);
  delay(1000);
}

void loop() 
{

  motorMoveSpeed(5, 5);
  delay(3000);
  motorMoveSpeed(0, 0);
  delay(1000);
  motorMoveSpeed(-5, -5);
  delay(3000);
  motorMoveSpeed(0, 0);
  delay(1000);
  
  motorMoveSpeed(10, 10);
  delay(3000);
  motorMoveSpeed(0, 0);
  delay(1000);
  motorMoveSpeed(-10, -10);
  delay(3000);
  motorMoveSpeed(0, 0);
  delay(1000);

  motorMoveSpeed(15, 15);
  delay(3000);
  motorMoveSpeed(0, 0);
  delay(1000);
  motorMoveSpeed(-15, -15);
  delay(3000);
  motorMoveSpeed(0, 0);
  delay(1000);

  motorMoveSpeed(20, 20);
  delay(3000);
  motorMoveSpeed(0, 0);
  delay(1000);
  motorMoveSpeed(-20, -20);
  delay(3000);
  motorMoveSpeed(0, 0);
  delay(1000);

  motorMoveSpeed(25, 25);
  delay(3000);
  motorMoveSpeed(0, 0);
  delay(1000);
  motorMoveSpeed(-25, -25);
  delay(3000);
  motorMoveSpeed(0, 0);
  delay(1000);
  
}

