#ifndef VELOCITY_ESTIMATOR_H
#define VELOCITY_ESTIMATOR_H

#include <stdint.h>

#define VELOCITY_ESTIMATOR_ACCEL_BIAS_SAMPLE_COUNT 64U

typedef struct
{
  /* MPU-frame angles. With this board's mounting, X is aircraft pitch and
   * Y is aircraft roll. Keep these paired with the unremapped MPU axes. */
  double mpu_angle_x_deg;
  double mpu_angle_y_deg;
  double acceleration_x_ms2;
  double acceleration_y_ms2;
  double acceleration_z_ms2;
  double gyro_x_dps;
  double gyro_y_dps;
  double gyro_z_dps;
} VelocityEstimatorImuSample_t;

typedef struct
{
  double vertical_velocity_ms;
  uint8_t vertical_velocity_valid;
  double srf05_altitude_cm;
  uint8_t srf05_altitude_valid;
} VelocityEstimatorEstimate_t;

typedef enum
{
  VELOCITY_ESTIMATOR_SRF05_REJECTED = 0,
  VELOCITY_ESTIMATOR_SRF05_ACCEPTED,
  VELOCITY_ESTIMATOR_SRF05_TILT_REJECTED
} VelocityEstimatorSrf05Result_t;

void VelocityEstimator_Init(void);
void VelocityEstimator_ResetVertical(void);
double VelocityEstimator_CorrectGyroZ(double raw_gyro_z_dps);
void VelocityEstimator_UpdateImu(
    const VelocityEstimatorImuSample_t *imu_sample);
void VelocityEstimator_Predict(double dt, uint32_t now_ms);
VelocityEstimatorSrf05Result_t VelocityEstimator_CorrectSrf05Altitude(
    double altitude_m, uint32_t sample_time_ms);
void VelocityEstimator_HandleSrf05Timeout(void);
void VelocityEstimator_CorrectBmp280Altitude(double altitude_m,
                                             uint32_t sample_time_ms);
VelocityEstimatorEstimate_t VelocityEstimator_GetEstimate(void);

#endif /* VELOCITY_ESTIMATOR_H */
