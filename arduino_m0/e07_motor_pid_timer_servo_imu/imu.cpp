#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "imu.h"
#include "./src/mpu9250/MPU9250-DMP.h"


#define LOG_SERIAL    SerialUSB



static MPU9250_DMP imu;

const signed char orientationMatrix[9] = {
   0,  1,  0,
  -1,  0,  0,
   0,  0,  1
};



void imuSelfTest(void);




bool imuBegin(imu_t *p_imu)
{
  bool ret = false;
  inv_error_t  err_code;


  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      LOG_SERIAL.println("Unable to communicate with MPU-9250");
      LOG_SERIAL.println("Check connections, and try again.");
      LOG_SERIAL.println();
      delay(5000);
    }
  }  

  //imu.setLPF(188);     // 188
  //imu.setAccelFSR(16);   // 2, 16
  //imu.setGyroFSR(1000);  // 2000, 1000
  

  
  //imu.setAccelFSR(16);
  //imu.setGyroFSR(1000); 
  
  //long a[3] = { 0, 20, 0};
    
  //mpu_set_accel_bias_6500_reg(a);
      
  //long g[3] = { -40, 13, 8};
  //long g[3] = { -20, 5, 0};
      
  //mpu_set_gyro_bias_reg(g);
  


  err_code = imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
                          DMP_FEATURE_SEND_RAW_ACCEL |
                          DMP_FEATURE_SEND_RAW_GYRO  |
                          DMP_FEATURE_GYRO_CAL, // Use gyro calibration
                          100); // Set DMP FIFO rate to 10 Hz                                                
  //imu.dmpSetOrientation(orientationMatrix);
  if (err_code == 0)
  {
    ret = true;
  }    
  
  memset((uint8_t *)p_imu, 0x00, sizeof(imu_t));
  

  return ret;
}

bool imuUpdate(imu_t *p_imu)
{
  bool ret = false;


  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();

      p_imu->qw = imu.calcQuat(imu.qw);
      p_imu->qx = imu.calcQuat(imu.qx);
      p_imu->qy = imu.calcQuat(imu.qy);
      p_imu->qz = imu.calcQuat(imu.qz);
    
      p_imu->roll  = (float)imu.roll;
      p_imu->pitch = (float)imu.pitch;
      p_imu->yaw   = (float)imu.yaw;

      p_imu->gx = imu.gx;
      p_imu->gy = imu.gy;
      p_imu->gz = imu.gz;

      p_imu->ax = imu.ax;
      p_imu->ay = imu.ay;
      p_imu->az = imu.az;

      ret = true;
    }
  }  

  return ret;
}

void imuSelfTest(void)
{
  int result;
  char test_packet[4] = {0};
  long gyro[3], accel[3];
  unsigned char i = 0;

  result = mpu_run_6500_self_test(gyro, accel, 0);

  #if 1
  //if (result == 0x7) 
  {
    /* Test passed. We can trust the gyro data here, so let's push it down
     * to the DMP.
     */
    for(i = 0; i<3; i++) 
    {
      gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
      accel[i] *= 2048.f; //convert to +-16G
      accel[i] = accel[i] >> 16;
      gyro[i] = (long)(gyro[i] >> 16);
    }

    mpu_set_gyro_bias_reg(gyro);
    mpu_set_accel_bias_6500_reg(accel);
  }  

  SerialUSB.println(result);
  #endif
}

void imuCalibration(imu_t *p_imu)
{
	int      cal_int = 0;
  uint8_t  axis = 0;
  uint16_t cal_count = 1000;
  long     a[3] = {0, 0, 0};
  long     g[3] = {0, 0, 0};
  uint32_t pre_time;
  unsigned short gyro_fsr;
  unsigned char accel_fsr;


  mpu_set_accel_bias_6500_reg(a);  
  mpu_set_gyro_bias_reg(g);

  accel_fsr = imu.getAccelFSR();
  gyro_fsr  = imu.getGyroFSR();
  
  imu.setAccelFSR(16);
  imu.setGyroFSR(1000); 


  for(axis=0; axis<3; axis++)
  {
    p_imu->acc_cal[axis]  = 0;
    p_imu->gyro_cal[axis] = 0;
  }

  cal_int = 0;
  pre_time = millis();
  while(1)
  {
    if (imuUpdate(p_imu) == true)
    {
      a[0] = imu.ax;
      a[1] = imu.ay;
      a[2] = imu.az;
      g[0] = imu.gx;
      g[1] = imu.gy;
      g[2] = imu.gz;

      for(axis=0; axis<3; axis++)
      {
        p_imu->acc_cal[axis]  += (float)a[axis];
        p_imu->gyro_cal[axis] += (float)g[axis];
      }      
      cal_int++;

      if (cal_int >= cal_count)
      {
        break;
      }
    }

    if (millis() - pre_time >= 2000)
    {
      break;
    }
  }

  if (cal_int > 0)
  {
    for(axis=0; axis<3; axis++)
    {
      p_imu->acc_cal[axis]  /= (float)cal_int;
      p_imu->gyro_cal[axis] /= (float)cal_int;   
    }
    p_imu->acc_cal[2] = 0;
  }

  mpu_set_accel_bias_6500_reg(p_imu->acc_cal);  
  mpu_set_gyro_bias_reg(p_imu->gyro_cal);  

  imu.setAccelFSR(accel_fsr);
  imu.setGyroFSR(gyro_fsr);   
}