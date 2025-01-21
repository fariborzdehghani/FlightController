// Read data from consecutive registers of an I2C device
#include "MPU6050.h"
#include "Tools.h"
#include "math.h"
#include "KalmanFilter.h"

const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

uint32_t MPU6050_timer;
MPU6050_t MPU6050;
HAL_StatusTypeDef MPU6050_devstat = -1;
HAL_StatusTypeDef MPU6050_cmdstat = -1;
uint8_t MPU6050_readystatus = 0;

KalmanFilter kf;

int IsCalibrated = 0;
double Roll_Offset = 0;
double Pitch_Offset = 0;
double Gz_Offset = 0;
double Az_Offset = 0;
double Vz_Offset = 0;

Kalman_t KalmanX = {
		.Q_angle = 0.001f,
		.Q_bias = 0.003f,
		.R_measure = 0.03f};

Kalman_t KalmanY = {
		.Q_angle = 0.001f,
		.Q_bias = 0.003f,
		.R_measure = 0.03f,
};

void I2C_ReadRegisters(uint8_t deviceAddress, uint8_t startRegister, uint8_t *data, uint8_t length) {
		while (I2C1->SR2 & I2C_SR2_BUSY); // Wait until I2C bus is not busy

		I2C1->CR1 |= I2C_CR1_ACK;	// Enable the ACK

		I2C1->CR1 |= I2C_CR1_START; // Generate start condition
		while (!(I2C1->SR1 & I2C_SR1_SB)); // Wait until start condition is generated

		I2C1->DR = deviceAddress << 1; // Send device address in write mode
		while (!(I2C1->SR1 & I2C_SR1_ADDR)); // Wait until address is sent
		//I2C1->CR1 &= ~I2C_CR1_ACK;	// clear the ACK bit
		uint8_t temp = I2C1->SR1 | I2C1->SR2; // Clear ADDR flag

		while (!(I2C1->SR1 & I2C_SR1_TXE));	// wait for TXE bit to set
		I2C1->DR = startRegister; // Send start register address
		while (!(I2C1->SR1 & I2C_SR1_BTF)); // Wait until byte transfer is finished
		//I2C1->CR1 &= ~I2C_CR1_ACK;	// clear the ACK bit

		I2C1->CR1 |= I2C_CR1_START; // Generate repeated start condition
		while (!(I2C1->SR1 & I2C_SR1_SB)); // Wait until repeated start condition is generated

		I2C1->DR = (deviceAddress << 1) | 0x01; // Send device address in read mode
		while (!(I2C1->SR1 & I2C_SR1_ADDR)); // Wait until address is sent and acknowledged
		//I2C1->CR1 &= ~I2C_CR1_ACK;	// clear the ACK bit
		temp = I2C1->SR1 | I2C1->SR2; // Clear ADDR flag

		for (uint8_t i = 0; i < length; i++) {
				if (i == length - 1) {
						I2C1->CR1 &= ~I2C_CR1_ACK; // Disable ACK before reading last byte
				}

				while (!(I2C1->SR1 & I2C_SR1_RXNE)); // Wait until data is received
				data[i] = I2C1->DR; // Read data
				I2C1->CR1 |= I2C_CR1_ACK;

				if (i == length - 1) {
						I2C1->CR1 |= I2C_CR1_STOP; // Send stop condition after reading last byte
				}
		}
}

void MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
	MPU6050_devstat = HAL_I2C_IsDeviceReady(I2Cx, 0xD0, 5, 100);
	HAL_Delay(100);
	if (MPU6050_devstat == HAL_OK) {
		
		MPU6050_readystatus = 10;
		uint8_t check;
		uint8_t Data;

		HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1,
				i2c_timeout);
		
		if (check == 104) // 0x68 will be returned by the sensor if everything goes well
				{
					MPU6050_readystatus = 11;
			// Reset the device
			Data = 0x80;
			HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1,
					&Data, 1, i2c_timeout);

			// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
			Data = 7;
			HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1,
					i2c_timeout);

			// Set CONFIG Register (FSYNC & DLPF)
			Data = 5;
			HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, i2c_timeout);

			// Set Gyroscopic configuration in GYRO_CONFIG Register
			// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=3 -> 250	?/s
			Data = 0x00;
			HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1,
					i2c_timeout);

			// Set accelerometer configuration in ACCEL_CONFIG Register
			// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> 2g
			Data = 0x00;
			HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1,
					i2c_timeout);

			// Set Interrupt Detail
			Data = 0x4;
			HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, INT_PIN_CFG_REG, 1, &Data, 1,
					i2c_timeout);

			// Set Interrupt Enabled
			Data = 0x1;
			HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, INT_ENABLE_REG, 1, &Data, 1,
					i2c_timeout);

			// power management register 0X6B we should write all 0's to wake the sensor up
			Data = 0x0;
			MPU6050_cmdstat = HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);
			
			MPU6050_readystatus = 12;

			// Init Kalman Filter
			KalmanFilter_Init(&kf, 0.1, 0.1);
			
			LogInformation(1001, "MPU6050 Started!");
			// LogInformation(1002, "MPU6050 Calibrating...");
			// MPU6050_Calibrate();
			// LogInformation(1001, "MPU6050 Calibrated!");
		}
		else{
			MPU6050_readystatus = 99;
		}
	}
	else
	{
		MPU6050_readystatus = 98;
	}	
}

void MPU6050_ReadAll(MPU6050_t *DataStruct)
{
		uint8_t Rec_Data[14];
		int16_t temp;

		// Read 14 BYTES of data starting from ACCEL_XOUT_H register
		I2C_ReadRegisters(0x68, ACCEL_XOUT_H_REG, &Rec_Data, 14);

		DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
		DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
		DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
		temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
		DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
		DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
		DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

		DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
		DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
		DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
		DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
		DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
		DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
		DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

		// Kalman angle solve
		double dt = (double)(HAL_GetTick() - MPU6050_timer) / 1000;
		MPU6050_timer = HAL_GetTick();
		double roll;
		double roll_sqrt = sqrt(DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);

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

		if (DataStruct->Ax != 0 || DataStruct->Ay != 0 || DataStruct->Az != 0){
			// Adjust Values
			double Roll_RAD = DataStruct->KalmanAngleX / 180 * M_PI;
			double Pitch_RAD = DataStruct->KalmanAngleY / 180 * M_PI;
			DataStruct->Az_ms = (DataStruct->Az - 1 * cos(Roll_RAD) * cos(Pitch_RAD)) * 9.81 * -1;

			// Calculate Vertical Velocity
			KalmanFilter_Update(&kf, DataStruct->Az_ms, dt);
			DataStruct->Vz = kf.state.v;
		}

		// Add Calibration
		// if (IsCalibrated == 1){
		// 	DataStruct->KalmanAngleX -= Roll_Offset;
		// 	DataStruct->KalmanAngleY -= Pitch_Offset;
		// 	DataStruct->Gz -= Gz_Offset;
		// 	DataStruct->Az_ms -= Az_Offset;
		// 	DataStruct->Vz -= Vz_Offset;
		// }

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

void MPU6050_Calibrate()
{
	int i = 0;
	int Calibration_Cycles = 200;

	double Roll_Offset_Sum = 0;
	double Pitch_Offset_Sum = 0;
	double Gz_Offset_Sum = 0;
	double Az_Offset_Sum = 0;
	double Vz_Offset_Sum = 0;

	for (i = 0; i < Calibration_Cycles; i++)
	{
		MPU6050_ReadAll(&MPU6050);

		Roll_Offset_Sum += MPU6050.KalmanAngleX;
		Pitch_Offset_Sum += MPU6050.KalmanAngleY;
		Gz_Offset_Sum += MPU6050.Gz;
		Az_Offset_Sum += MPU6050.Az_ms;
		Vz_Offset_Sum += MPU6050.Vz;

		HAL_Delay(LOOP_TIME * 1000);
	}

	Roll_Offset = Roll_Offset_Sum / Calibration_Cycles;
	Pitch_Offset = Pitch_Offset_Sum / Calibration_Cycles;
	Gz_Offset = Gz_Offset_Sum / Calibration_Cycles;
	Az_Offset = Az_Offset_Sum / Calibration_Cycles;
	Vz_Offset = Vz_Offset_Sum / Calibration_Cycles;

	IsCalibrated = 1;
}

void MPU6050_DisableClock()
{
	RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
}

void MPU6050_EnableClock()
{
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
}

