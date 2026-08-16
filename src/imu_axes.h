#pragma once

// MPU6050 yaw axis mapping. Standard mount: board flat, Z vertical (yaw).
// Flip sign in platformio.ini if heading correction fights drift: -D IMU_GYRO_YAW_SIGN=-1.0f

#ifndef IMU_GYRO_YAW_SIGN
#define IMU_GYRO_YAW_SIGN 1.0f
#endif

static inline float imu_yaw_rate(float gx, float gy, float gz) {
  (void)gx;
  (void)gy;
  return IMU_GYRO_YAW_SIGN * gz;
}
