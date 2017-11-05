#include "motor.h"
#include "src/mpu9250/MPU9250-DMP.h"


MPU9250_DMP imu;

const signed char orientationMatrix[9] = {
   0,  1,  0,
  -1,  0,  0,
   0,  0,  1
};

void setup() 
{
  inv_error_t  err_code;

  motorBegin();
  Serial.begin(115200);

  motorMoveSpeed(0, 0);
  delay(1000);

  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
  }  

  err_code = imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
                          DMP_FEATURE_SEND_RAW_ACCEL |
                          DMP_FEATURE_SEND_CAL_GYRO  |
                          DMP_FEATURE_GYRO_CAL, // Use gyro calibration
                          100); // Set DMP FIFO rate to 10 Hz                                                
  imu.dmpSetOrientation(orientationMatrix);
  if (err_code == 0)
  {
    Serial.println("dmpBegin OK");
  }    
}

void loop() 
{
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();
      printIMUData();
    }
  }
}

void printIMUData(void)
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);


  Serial.println("R/P/Y: " + String(imu.roll) + " " + String(imu.pitch) + " " + String(imu.yaw));

}



