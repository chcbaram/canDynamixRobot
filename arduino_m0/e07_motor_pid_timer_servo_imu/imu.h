#ifndef _IMU_H_
#define _IMU_H_




typedef struct
{
  float qw; // q0
  float qx; // q1
  float qy; // q2
  float qz; // q3
  
  float roll;
  float pitch;
  float yaw;

  int16_t ax;
  int16_t ay;
  int16_t az;

  int16_t gx;
  int16_t gy;
  int16_t gz;
  
  long acc_cal[3];
  long gyro_cal[3];  
} imu_t;


bool imuBegin(imu_t *p_imu);


bool imuUpdate(imu_t *p_imu);
void imuCalibration(imu_t *p_imu);

#endif

