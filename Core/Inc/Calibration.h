#ifndef __CALIBRATION_H
#define __CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

void Calibrate_MPU6050(void);
double FilterYaw(double yaw_meas_deg);

#ifdef __cplusplus
}
#endif

#endif /* __CALIBRATION_H */
