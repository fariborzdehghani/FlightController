// Read data from consecutive registers of an I2C device
#include "MPU6050.h"
#include "Tools.h"
#include "math.h"

static const uint16_t i2c_timeout = 100u;
static uint32_t MPU6050_timer;
static I2C_HandleTypeDef *mpu_i2c;

static Kalman_t KalmanX = {
		.Q_angle = 0.001f,
		.Q_bias = 0.003f,
		.R_measure = 0.03f};

static Kalman_t KalmanY = {
		.Q_angle = 0.001f,
		.Q_bias = 0.003f,
		.R_measure = 0.03f,
};

static HAL_StatusTypeDef I2C_ReadRegisters(uint8_t start_register,
                                           uint8_t *data, uint16_t length) {
	if (mpu_i2c == NULL || data == NULL || length == 0u) {
		return HAL_ERROR;
	}

	return HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDR, start_register,
	                        I2C_MEMADD_SIZE_8BIT, data, length, i2c_timeout);
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *i2c)
{
	if (i2c == NULL) {
		return HAL_ERROR;
	}

	mpu_i2c = i2c;
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(i2c, MPU6050_ADDR, 5, i2c_timeout);
	HAL_Delay(100);
	if (status == HAL_OK) {
		
		uint8_t check;
		uint8_t Data;

		status = HAL_I2C_Mem_Read(i2c, MPU6050_ADDR, WHO_AM_I_REG,
		                              I2C_MEMADD_SIZE_8BIT, &check, 1,
		                              i2c_timeout);
		if (status != HAL_OK || check != 0x68u) {
			return (status != HAL_OK) ? status : HAL_ERROR;
		}
		
		if (check == 104) // 0x68 will be returned by the sensor if everything goes well
				{
			// Reset the device
			Data = 0x80;
			status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, PWR_MGMT_1_REG,
			                           I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
			if (status != HAL_OK) return status;
			HAL_Delay(100);

			// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
			Data = 7;
			status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, SMPLRT_DIV_REG,
			                           I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
			if (status != HAL_OK) return status;

			// Set CONFIG Register (FSYNC & DLPF)
			Data = 5;
			status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, CONFIG_REG,
			                           I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
			if (status != HAL_OK) return status;

			// Set Gyroscopic configuration in GYRO_CONFIG Register
			// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=3 -> 250	?/s
			Data = 0x00;
			status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, GYRO_CONFIG_REG,
			                           I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
			if (status != HAL_OK) return status;

			// Set accelerometer configuration in ACCEL_CONFIG Register
			// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> 2g
			Data = 0x00;
			status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, ACCEL_CONFIG_REG,
			                           I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
			if (status != HAL_OK) return status;

			// Set Interrupt Detail
			Data = 0x4;
			status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, INT_PIN_CFG_REG,
			                           I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
			if (status != HAL_OK) return status;

			// Set Interrupt Enabled
			Data = 0x1;
			status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, INT_ENABLE_REG,
			                           I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
			if (status != HAL_OK) return status;

			// power management register 0X6B we should write all 0's to wake the sensor up
			Data = 0x0;
			status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, PWR_MGMT_1_REG,
			                           I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
			if (status != HAL_OK) return status;
			MPU6050_timer = HAL_GetTick();
		
			LogInformation(1001, "MPU6050 Started!");
			return HAL_OK;
		}
	}
	return status;
}

HAL_StatusTypeDef MPU6050_ReadAll(MPU6050_t *DataStruct)
{
		if (DataStruct == NULL || mpu_i2c == NULL) {
			return HAL_ERROR;
		}

		uint8_t Rec_Data[14];
		int16_t temp;

		// Read 14 BYTES of data starting from ACCEL_XOUT_H register
		HAL_StatusTypeDef status = I2C_ReadRegisters(ACCEL_XOUT_H_REG, Rec_Data,
		                                                 sizeof(Rec_Data));
		if (status != HAL_OK) {
			return status;
		}

		DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
		DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
		DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
		temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
		DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
		DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
		DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

		// Scale factor for ±2g range is 16384 LSB/g
		DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
		DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
		DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0;

		// Convert from g to m/s² for all axes
		DataStruct->Ax_ms = DataStruct->Ax * 9.80665f;
		DataStruct->Ay_ms = DataStruct->Ay * 9.80665f;
		DataStruct->Az_ms = DataStruct->Az * 9.80665f;

		DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
		DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
		DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
		DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

		// Kalman angle solve
		const uint32_t now = HAL_GetTick();
		double dt = (double)(now - MPU6050_timer) / 1000.0;
		MPU6050_timer = now;
		if (dt <= 0.0) dt = 0.001;
		if (dt > 0.1) dt = 0.1;
		double roll;
		double roll_sqrt = sqrt((double)DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW +
		                        (double)DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);

		if (roll_sqrt != 0.0)
		{
				roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
		}
		else
		{
				roll = 0.0;
		}
		
		double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
		if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
		{
				KalmanY.angle = pitch;
				DataStruct->KalmanAngleY = pitch;
		}
		else
		{
				DataStruct->KalmanAngleY = Kalman_GetAngle(&KalmanY, pitch, DataStruct->Gy, dt);
		}
		if (fabs(DataStruct->KalmanAngleY) > 90) DataStruct->Gx = -DataStruct->Gx;
		DataStruct->KalmanAngleX = Kalman_GetAngle(&KalmanX, roll, DataStruct->Gx, dt);
		return HAL_OK;
}

double Kalman_GetAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
		double rate = newRate - Kalman->bias;
		Kalman->angle += dt * rate;

		Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
		Kalman->P[0][1] -= dt * Kalman->P[1][1];
		Kalman->P[1][0] -= dt * Kalman->P[1][1];
		Kalman->P[1][1] += Kalman->Q_bias * dt;

		double S = Kalman->P[0][0] + Kalman->R_measure;
		double K[2];
		K[0] = Kalman->P[0][0] / S;
		K[1] = Kalman->P[1][0] / S;

		double y = newAngle - Kalman->angle;
		Kalman->angle += K[0] * y;
		Kalman->bias += K[1] * y;

		double P00_temp = Kalman->P[0][0];
		double P01_temp = Kalman->P[0][1];

		Kalman->P[0][0] -= K[0] * P00_temp;
		Kalman->P[0][1] -= K[0] * P01_temp;
		Kalman->P[1][0] -= K[1] * P00_temp;
		Kalman->P[1][1] -= K[1] * P01_temp;

		return Kalman->angle;
}
