#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"
#include "stdint.h"
#include "stm32f1xx_hal.h"

typedef struct
{
		double Q_angle;
		double Q_bias;
		double R_measure;
		double angle;
		double bias;
		double P[2][2];
} Kalman_t;

typedef struct
{
		int16_t Accel_X_RAW;
		int16_t Accel_Y_RAW;
		int16_t Accel_Z_RAW;
		double Ax;
		double Ay;
		double Az;
		double Az_ms;

		int16_t Gyro_X_RAW;
		int16_t Gyro_Y_RAW;
		int16_t Gyro_Z_RAW;
		double Gx;
		double Gy;
		double Gz;

		float Temperature;

		double KalmanAngleX;
		double KalmanAngleY;

		double Vz;
} MPU6050_t;

#define MPU6050_ADDR 0xD0
#define WHO_AM_I_REG 0x75
#define SMPLRT_DIV_REG 0x19
#define CONFIG_REG 0x1A
#define INT_PIN_CFG_REG 0x37
#define PWR_MGMT_1_REG 0x6B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define INT_ENABLE_REG 0x38
#define RAD_TO_DEG 57.295779513082320876798154814105

void MPU6050_Init(I2C_HandleTypeDef *I2Cx);
void MPU6050_ReadAll(MPU6050_t *DataStruct);
double Kalman_GetAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
void MPU6050_Calibrate();
void MPU6050_DisableClock();
void MPU6050_EnableClock();
#endif /* __MPU6050_H */