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
