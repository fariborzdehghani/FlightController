#include "Tests.h"
#include "srf05.h"
#include "QMC5883.h"
#include "MPU6050.h"
#include "BMPXX80.h"
#include "SensorConfig.h"

extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c1;

float Compas_Value;
float distance = 0;
float heading = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;
float elevation = 0.0f;

static const QMC5883P_Config_t qmc5883Config = {
    .i2c = &hi2c2,
    .address_7bit = QMC5883P_DEFAULT_ADDR_7BIT,
    .timeout_ms = 100,
    .mode = QMC_MODE_CONTINUOUS,
    .odr = QMC_ODR_50HZ,
    .osr1 = QMC_OSR1_4,
    .osr2 = QMC_OSR2_4,
    .range = QMC_RANGE_8G,
    .setreset = QMC_SETRESET_ON_AND_ON
};

static const SRF05_Config_t srf05Config = {
    .timer = &htim3,
    .captureChannel = TIM_CHANNEL_3,
    .triggerPort = GPIOB,
    .triggerPin = GPIO_PIN_1,
    .timeout_ms = 60,
    .timer_tick_us = 1.0f,
};

static const BMP280_Config_t bmp280Config = {
    .i2c = &hi2c2,
    .temperature_resolution = BMP280_TEMPERATURE_20BIT,
    .pressure_oversampling = BMP280_ULTRAHIGHRES,
    .mode = BMP280_NORMALMODE,
    .standby_time = BME280_STANDBY_MS_0_5,
    .filter = BME280_FILTER_OFF
};

static MPU6050_t mpu6050Data;

void test_init(void)
{
  if (SENSOR_SRF05_ENABLED)
  {
    // Initialize SRF05
    (void)SRF05_Init(srf05Config);
  }
    
  if (SENSOR_QMC5883_ENABLED)
  {
    // Initialize QMC5883 compass
    (void)QMC5883P_Init(qmc5883Config);
  }

  if (SENSOR_BMP280_ENABLED)
  {
    // Initialize BMP280
    (void)BMP280_Init(bmp280Config);
  }
    
  if (SENSOR_MPU6050_ENABLED)
  {
    // Initialize MPU6050
    if (MPU6050_Init(&hi2c1) == HAL_OK)
    {
      (void)MPU6050_Calibrate();
    }
  }
    
}

void test_loop(void)
{
  if (SENSOR_SRF05_ENABLED)
  {
    // Read Distance
    (void)SRF05_MeasureDistance(&distance);
  }
  else
  {
    distance = 0.0f;
  }
        
  if (SENSOR_BMP280_ENABLED)
  {
    // Read BMP280 altitude data
    (void)BMP280_ReadAltitude(&elevation);
  }
  else
  {
    elevation = 0.0f;
  }

  if (SENSOR_QMC5883_ENABLED)
  {
    // Read compass heading
    heading = QMC5883P_ReadHeading();
  }
  else
  {
    heading = 0.0f;
  }
    
  if (SENSOR_MPU6050_ENABLED)
  {
    // Read MPU6050 data for roll and pitch
    (void)MPU6050_ReadAll(&mpu6050Data);
    roll = mpu6050Data.KalmanAngleX;
    pitch = mpu6050Data.KalmanAngleY;
  }
  else
  {
    roll = 0.0f;
    pitch = 0.0f;
  }
    
  HAL_Delay(250); 
}
