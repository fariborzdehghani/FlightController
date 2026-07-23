#ifndef QMC5883P_H
#define QMC5883P_H

#include "stm32f4xx_hal.h" // Replace with your series' HAL header if needed
#include <stdbool.h>
#include <stdint.h>

/* Default 7-bit I2C address (per datasheet) */
#define QMC5883P_DEFAULT_ADDR_7BIT                                             \
  0x2Cu /* 0x2C (7-bit) - datasheet default                                    \
         */

/* Registers (from datasheet) */
#define QMC5883P_REG_CHIP_ID 0x00u
#define QMC5883P_REG_X_LSB 0x01u
#define QMC5883P_REG_X_MSB 0x02u
#define QMC5883P_REG_Y_LSB 0x03u
#define QMC5883P_REG_Y_MSB 0x04u
#define QMC5883P_REG_Z_LSB 0x05u
#define QMC5883P_REG_Z_MSB 0x06u
#define QMC5883P_REG_STATUS 0x09u
#define QMC5883P_REG_CTRL1 0x0Au
#define QMC5883P_REG_CTRL2 0x0Bu
/* Note: some app examples reference other addresses (e.g. sign register), but
 * primary control/data regs are above. */

/* Default timeout (ms) if handle->timeout_ms == 0 */
#define QMC5883P_DEFAULT_TIMEOUT_MS 100u

/* Calibration duration (ms) used by QMC5883P_Calibrate(). Adjust as needed. */
#define QMC5883P_CALIBRATION_DURATION_MS 10000u

/* Control register bit / field helpers (codes as in datasheet) */
/* MODE (CTRL1 bits[1:0]) */
typedef enum {
  QMC_MODE_SUSPEND = 0x0,
  QMC_MODE_NORMAL = 0x1,
  QMC_MODE_SINGLE = 0x2,
  QMC_MODE_CONTINUOUS = 0x3
} qmc5883p_mode_t;

/* ODR (CTRL1 bits[3:2]) */
typedef enum {
  QMC_ODR_10HZ = 0x0,
  QMC_ODR_50HZ = 0x1,
  QMC_ODR_100HZ = 0x2,
  QMC_ODR_200HZ = 0x3
} qmc5883p_odr_t;

/* OSR1 (CTRL1 bits[5:4]) code -> mapping: 00->8, 01->4, 10->2, 11->1 (see
 * datasheet) */
typedef enum {
  QMC_OSR1_8 = 0x0,
  QMC_OSR1_4 = 0x1,
  QMC_OSR1_2 = 0x2,
  QMC_OSR1_1 = 0x3
} qmc5883p_osr1_t;

/* OSR2 (CTRL1 bits[7:6]) code -> mapping: 00->1,01->2,10->4,11->8 (see
 * datasheet) */
typedef enum {
  QMC_OSR2_1 = 0x0,
  QMC_OSR2_2 = 0x1,
  QMC_OSR2_4 = 0x2,
  QMC_OSR2_8 = 0x3
} qmc5883p_osr2_t;

/* RNG (CTRL2 bits[3:2]) code mapping: 00->±30G, 01->±12G, 10->±8G, 11->±2G */
typedef enum {
  QMC_RANGE_30G = 0x0,
  QMC_RANGE_12G = 0x1,
  QMC_RANGE_8G = 0x2,
  QMC_RANGE_2G = 0x3
} qmc5883p_range_t;

/* SET/RESET MODE (CTRL2 bits[1:0]) - values per datasheet; default 00 is 'set &
 * reset on' */
typedef enum {
  QMC_SETRESET_ON_AND_ON = 0x0,
  QMC_SETRESET_SET_ONLY = 0x1,
  QMC_SETRESET_OFF = 0x2
} qmc5883p_setreset_t;

/* Data struct */
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t
      temperature; /* not all modules populate temp; datasheet had T registers
                      at 0x07/0x08 in older layouts — keep field for future */
} QMC5883P_Data_t;

/* Configuration struct */
typedef struct {
  I2C_HandleTypeDef *i2c; /* user-supplied HAL I2C handle */
  uint8_t address_7bit; /* 7-bit address (default QMC5883P_DEFAULT_ADDR_7BIT) */
  uint32_t timeout_ms;  /* HAL timeout in ms (0 -> driver uses
                           QMC5883P_DEFAULT_TIMEOUT_MS) */
  qmc5883p_mode_t mode; /* Operating mode */
  qmc5883p_odr_t odr;   /* Output data rate */
  qmc5883p_osr1_t osr1; /* Oversampling ratio 1 */
  qmc5883p_osr2_t osr2; /* Oversampling ratio 2 */
  qmc5883p_range_t range;       /* Magnetic field range */
  qmc5883p_setreset_t setreset; /* Set/Reset mode */
} QMC5883P_Config_t;

/* Handle with current state */
typedef struct {
  QMC5883P_Config_t config;
  bool initialized;
} QMC5883P_Handle_t;

/* API */
HAL_StatusTypeDef QMC5883P_Init(QMC5883P_Config_t config);
HAL_StatusTypeDef QMC5883P_SoftReset(void);
/* Read raw (will poll DRDY until ready or timeout) */
HAL_StatusTypeDef QMC5883P_ReadRaw(QMC5883P_Data_t *out);
/* Read status register */
HAL_StatusTypeDef QMC5883P_ReadStatus(uint8_t *status);
/* Compute heading in degrees from X/Y (simple atan2) */
float QMC5883P_HeadingDeg(const QMC5883P_Data_t *d);
/* Read and calculate heading in one step, returns -1 if error occurs */
float QMC5883P_ReadHeading(void);


/* Perform simple hard-iron calibration over a duration defined by
 * QMC5883P_CALIBRATION_DURATION_MS. This will blink the board LED during
 * calibration. Offsets are stored internally and used by
 * QMC5883P_CompensatedYaw(). */
HAL_StatusTypeDef QMC5883P_Calibrate(void);

/* Compute tilt-compensated yaw (degrees) from stored magnetometer data and
 * supplied `roll` and `pitch` in degrees. Returns heading in [0,360). */
HAL_StatusTypeDef QMC5883P_CompensatedYaw(float roll_deg, float pitch_deg,
                                          float *yaw_deg);

#endif /* QMC5883P_H */
