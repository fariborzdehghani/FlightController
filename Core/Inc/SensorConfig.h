#ifndef __SENSOR_CONFIG_H
#define __SENSOR_CONFIG_H

/*
 * Set an individual flag to 0u to exclude that sensor from initialization,
 * sampling, and its dependent flight-control logic. These values may also be
 * overridden by compiler definitions (for example,
 * -DSENSOR_BMP280_ENABLED=0).
 */
#ifndef SENSOR_MPU6050_ENABLED
#define SENSOR_MPU6050_ENABLED 1u
#endif

#ifndef SENSOR_QMC5883_ENABLED
#define SENSOR_QMC5883_ENABLED 1u
#endif

#ifndef SENSOR_SRF05_ENABLED
#define SENSOR_SRF05_ENABLED 1u
#endif

#ifndef SENSOR_BMP280_ENABLED
#define SENSOR_BMP280_ENABLED 1u
#endif

/* Fixed accelerometer full scale: +/-8 g (AFS_SEL=2).
 * Keep the register setting, raw conversion and recorded range together.
 * This measurement range is separate from the filter's 0.8-1.2 g gate.
 */
#define MPU6050_ACCEL_RANGE_G 8u
#define MPU6050_ACCEL_CONFIG_VALUE 0x10u
#define MPU6050_ACCEL_SCALE_LSB_PER_G 4096.0

/* MPU attitude filtering. These settings also accept compiler -D overrides.
 * Set rejection to 0u to restore the original accelerometer correction path.
 */
#ifndef MPU6050_ACCEL_REJECTION_ENABLED
#define MPU6050_ACCEL_REJECTION_ENABLED 1u
#endif

/* Gyro cutoff in Hz: 5 (stronger), 10, 20, or 42 (less smoothing/lag).
 * The corresponding accelerometer cutoffs are 5, 10, 21, and 44 Hz.
 * The default is 20 Hz; use 10 or 5 Hz for stronger vibration filtering.
 * All choices retain
 * the existing 125 Hz MPU sample rate. This is not the sample-rate setting.
 */
#ifndef MPU6050_DLPF_HZ
#define MPU6050_DLPF_HZ 20u
#endif

#if (MPU6050_ACCEL_REJECTION_ENABLED != 0u) && \
    (MPU6050_ACCEL_REJECTION_ENABLED != 1u)
#error "MPU6050_ACCEL_REJECTION_ENABLED must be 0 or 1"
#endif

#if (MPU6050_DLPF_HZ != 5u) && (MPU6050_DLPF_HZ != 10u) && \
    (MPU6050_DLPF_HZ != 20u) && (MPU6050_DLPF_HZ != 42u)
#error "MPU6050_DLPF_HZ must be 5, 10, 20, or 42"
#endif

#if (SENSOR_MPU6050_ENABLED != 0u) && (SENSOR_MPU6050_ENABLED != 1u)
#error "SENSOR_MPU6050_ENABLED must be 0 or 1"
#endif

#if (SENSOR_QMC5883_ENABLED != 0u) && (SENSOR_QMC5883_ENABLED != 1u)
#error "SENSOR_QMC5883_ENABLED must be 0 or 1"
#endif

#if (SENSOR_SRF05_ENABLED != 0u) && (SENSOR_SRF05_ENABLED != 1u)
#error "SENSOR_SRF05_ENABLED must be 0 or 1"
#endif

#if (SENSOR_BMP280_ENABLED != 0u) && (SENSOR_BMP280_ENABLED != 1u)
#error "SENSOR_BMP280_ENABLED must be 0 or 1"
#endif

#endif /* __SENSOR_CONFIG_H */
