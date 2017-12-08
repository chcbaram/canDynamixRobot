#include "motor.h"
#include "imu.h"
#include <Servo.h>


#define LOG_SERIAL    SerialUSB



Servo rcservo;
imu_t imu;



void setup() 
{
  rcservo.attach(10); 
  rcservo.write(0);

  motorBegin();
  motorMoveSpeed(0, 0);
  LOG_SERIAL.begin(115200);
  
  
  imuBegin(&imu);  
  imuCalibration(&imu);
}

void loop() 
{
  static uint8_t  angle = 0;
  static uint32_t pre_time;
  static uint32_t pre_time_imu;


  if (millis()-pre_time >= 2000)
  {
    pre_time = millis();

    angle = (angle + 90) % 180;
    rcservo.write(angle);
  }


  
    if (imuUpdate(&imu) == true)
    {
      if (millis()-pre_time_imu > 50)
      {
        pre_time_imu = millis();
        //LOG_SERIAL.println("R/P/Y: " + String(imu.roll) + " " + String(imu.pitch) + " " + String(imu.yaw));  
        //LOG_SERIAL.print(String(imu.ax) + "\t " + String(imu.ay) + "\t " + String(imu.az));
        //LOG_SERIAL.print(String(imu.gx) + "\t " + String(imu.gy) + "\t " + String(imu.gz));  
        //LOG_SERIAL.print(" ");
        LOG_SERIAL.println("\tR/P/Y: " + String(imu.roll) + "\t " + String(imu.pitch) + "\t " + String(imu.yaw));  
      }
    }  
}

