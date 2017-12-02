#include "motor.h"
#include <Servo.h>


#define LOG_SERIAL    SerialUSB



Servo rcservo;




void setup() 
{
  rcservo.attach(10); 
  rcservo.write(0);

  motorBegin();
  //motorSetSpeed(L_MOTOR, 10);
  motorMoveSpeed(10, 10);
  LOG_SERIAL.begin(115200);
}

void loop() 
{
  static uint8_t  angle = 0;
  static uint32_t pre_time;
  
  LOG_SERIAL.print(motorGetGoalSpeed(L_MOTOR));
  LOG_SERIAL.print(" 0 30 ");
  LOG_SERIAL.println(motorGetSpeed(L_MOTOR));
  delay(50);

  if (millis()-pre_time >= 2000)
  {
    pre_time = millis();

    angle = (angle + 90) % 180;
    rcservo.write(angle);
  }

}

