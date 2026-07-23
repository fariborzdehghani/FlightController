#include "qmc5883.h"
#include "main.h"
#include <limits.h>
#include <math.h>


/* ---------------- One Euro filter (angle) ---------------- */
typedef struct {
  uint8_t inited;
  uint32_t last_ms;

  float x_hat;   /* filtered angle (deg, 0..360) */
  float dx_hat;  /* filtered derivative (deg/s) */
} OneEuroAngle_t;

/* Tune these */
#define YAW_MIN_CUTOFF_HZ   2.5f   /* higher = less lag, less smoothing */
#define YAW_BETA            0.06f  /* higher = more responsive during motion */
#define YAW_D_CUTOFF_HZ     8.0f   /* derivative LPF cutoff */

#define YAW_MAX_STEP_DEG    35.0f  /* spike gate per sample (deg) */
#define YAW_MAX_RATE_DPS    500.0f /* slew limit (deg/s) */

static OneEuroAngle_t yaw_flt = {0};

/* Static handle for the driver */
static QMC5883P_Handle_t handle;
static QMC5883P_Data_t data;
static QMC5883P_Data_t compensated_data;

float offsetX =
    -90.0f; /* Example hard-iron offset for X axis, in raw units (LSB) */

float offsetY =
    -20.0f; /* Example hard-iron offset for Y axis, in raw units (LSB) */

float offsetZ =
    -330.0f; /* Example hard-iron offset for Z axis, in raw units (LSB) */

/* internal helpers */
static inline uint16_t dev_addr_8bit(void) {
  return (uint16_t)((handle.config.address_7bit)
                    << 1); /* HAL expects address << 1 for DevAddress param */
}
static inline uint32_t used_timeout(void) {
  return (handle.config.timeout_ms == 0u) ? QMC5883P_DEFAULT_TIMEOUT_MS
                                          : handle.config.timeout_ms;
}

static HAL_StatusTypeDef write_reg(uint8_t reg, uint8_t val) {
  return HAL_I2C_Mem_Write(handle.config.i2c, dev_addr_8bit(), reg,
                           I2C_MEMADD_SIZE_8BIT, &val, 1, used_timeout());
}

static HAL_StatusTypeDef read_reg(uint8_t reg, uint8_t *val) {
  return HAL_I2C_Mem_Read(handle.config.i2c, dev_addr_8bit(), reg,
                          I2C_MEMADD_SIZE_8BIT, val, 1, used_timeout());
}

static HAL_StatusTypeDef read_regs(uint8_t reg, uint8_t *buf, uint16_t len) {
  return HAL_I2C_Mem_Read(handle.config.i2c, dev_addr_8bit(), reg,
                          I2C_MEMADD_SIZE_8BIT, buf, len, used_timeout());
}

static inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }

/* ---------------- Angle helpers ---------------- */
static inline float wrap360f(float a) {
  a = fmodf(a, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a;
}

/* target - current in [-180, +180) */
static inline float ang_diff_deg(float target, float current) {
  float d = wrap360f(target) - wrap360f(current);
  if (d >= 180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return d;
}

static inline float lpf_alpha(float cutoff_hz, float dt_s) {
  if (cutoff_hz <= 0.0f) return 1.0f;
  const float tau = 1.0f / (2.0f * (float)M_PI * cutoff_hz);
  return dt_s / (tau + dt_s);
}


static float one_euro_angle_update(OneEuroAngle_t *f, float x_deg, float dt_s) {
  x_deg = wrap360f(x_deg);

  if (!f->inited) {
    f->inited = 1;
    f->x_hat = x_deg;
    f->dx_hat = 0.0f;
    return f->x_hat;
  }

  /* Spike gate (reject single-sample glitches) */
  float d = ang_diff_deg(x_deg, f->x_hat);
  if (fabsf(d) > YAW_MAX_STEP_DEG) {
    /* clamp instead of full reject (more robust in real motion) */
    d = (d > 0.0f) ? YAW_MAX_STEP_DEG : -YAW_MAX_STEP_DEG;
    x_deg = wrap360f(f->x_hat + d);
  }

  /* Derivative (deg/s) */
  float dx = ang_diff_deg(x_deg, f->x_hat) / dt_s;

  /* Slew-rate limit (prevents jumpy outputs) */
  if (dx > YAW_MAX_RATE_DPS) dx = YAW_MAX_RATE_DPS;
  if (dx < -YAW_MAX_RATE_DPS) dx = -YAW_MAX_RATE_DPS;

  /* Filter derivative */
  {
    float a_d = lpf_alpha(YAW_D_CUTOFF_HZ, dt_s);
    f->dx_hat = f->dx_hat + a_d * (dx - f->dx_hat);
  }

  /* Adaptive cutoff */
  float cutoff = YAW_MIN_CUTOFF_HZ + YAW_BETA * fabsf(f->dx_hat);

  /* Filter angle (wrap-safe) */
  {
    float a = lpf_alpha(cutoff, dt_s);
    float err = ang_diff_deg(x_deg, f->x_hat);
    f->x_hat = wrap360f(f->x_hat + a * err);
  }

  return f->x_hat;
}

/* Public API */

HAL_StatusTypeDef QMC5883P_Init(QMC5883P_Config_t config) {
  if (config.i2c == NULL)
    return HAL_ERROR;

  /* Store configuration */
  handle.config = config;
  if (handle.config.address_7bit == 0) {
    handle.config.address_7bit = QMC5883P_DEFAULT_ADDR_7BIT;
  }

  uint8_t chipid = 0;
  HAL_StatusTypeDef st = read_reg(QMC5883P_REG_CHIP_ID, &chipid);
  if (st != HAL_OK)
    return st;

  /* Datasheet: default CHIP ID is 0x80 */
  if (chipid != 0x80u) {
    /* Not matching expected ID — return error to caller (could be different
     * part / wiring) */
    return HAL_ERROR;
  }

  /* Configure the device with provided settings */
  /* Build CTRL1 */
  uint8_t ctrl1 =
      (uint8_t)(((config.osr2 & 0x3) << 6) | ((config.osr1 & 0x3) << 4) |
                ((config.odr & 0x3) << 2) | (config.mode & 0x3));

  st = write_reg(QMC5883P_REG_CTRL1, ctrl1);
  if (st != HAL_OK)
    return st;

  /* Build CTRL2: keep SOFT_RST/SELF_TEST = 0 for normal config */
  uint8_t ctrl2 =
      (uint8_t)(((config.range & 0x3) << 2) | (config.setreset & 0x3));
  st = write_reg(QMC5883P_REG_CTRL2, ctrl2);
  if (st != HAL_OK)
    return st;

  handle.initialized = true;
  /* Perform module calibration (will blink LED during calibration). */
  HAL_StatusTypeDef cst = QMC5883P_Calibrate();
  if (cst != HAL_OK)
    return cst;

  return HAL_OK;
}

HAL_StatusTypeDef QMC5883P_SoftReset(void) {
  /* CTRL2 bit7 = SOFT_RST (write 1 to reset) */
  uint8_t val = 0x80u;
  return write_reg(QMC5883P_REG_CTRL2, val);
}

HAL_StatusTypeDef QMC5883P_ReadStatus(uint8_t *status) {
  if (status == NULL)
    return HAL_ERROR;
  return read_reg(QMC5883P_REG_STATUS, status);
}

HAL_StatusTypeDef QMC5883P_ReadRaw(QMC5883P_Data_t *out) {
  if (out == NULL || !handle.initialized)
    return HAL_ERROR;

  uint32_t t0 = HAL_GetTick();
  uint32_t timeout = used_timeout();

  /* Wait for DRDY bit in status (bit0) */
  uint8_t status = 0;
  HAL_StatusTypeDef st;
  do {
    st = QMC5883P_ReadStatus(&status);
    if (st != HAL_OK)
      return st;
    if (status & 0x01u)
      break; /* DRDY == 1 */
  } while ((HAL_GetTick() - t0) < timeout);

  if (!(status & 0x01u)) {
    /* Data not ready within timeout */
    return HAL_TIMEOUT;
  }

  uint8_t buf[6];
  st = read_regs(QMC5883P_REG_X_LSB, buf, 6);
  if (st != HAL_OK)
    return st;

  /* Data layout per datasheet: LSB then MSB for each axis (16-bit two's
   * complement) */
  out->y = (int16_t)((uint16_t)buf[1] << 8 | buf[0]); // toward west
  out->x = (int16_t)((uint16_t)buf[3] << 8 | buf[2]); // toward north
  out->z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
  out->temperature = 0; /* Datasheet references T regs in other layouts; ignore
                           unless present */

  return HAL_OK;
}

float QMC5883P_HeadingDeg(const QMC5883P_Data_t *d) {
  /* Basic heading from X/Y; user should apply calibration (hard/soft iron) for
   * accuracy */
  float fx = (float)d->x;
  float fy = (float)d->y;
  float heading = atan2f(fy, fx) * 180.0f / (float)M_PI;
  if (heading < 0.0f)
    heading += 360.0f;
  return heading;
}

float QMC5883P_ReadHeading(void) {
  // QMC5883P_Data_t data;
  HAL_StatusTypeDef status = QMC5883P_ReadRaw(&data);
  if (status != HAL_OK)
    return -1.0f;

  return QMC5883P_HeadingDeg(&data);
}

HAL_StatusTypeDef QMC5883P_Calibrate(void) {
  if (!handle.initialized) {
    return HAL_ERROR;
  }

  yaw_flt = (OneEuroAngle_t){0};
  return HAL_OK;
}

HAL_StatusTypeDef QMC5883P_CompensatedYaw(float roll_deg, float pitch_deg,
                                          float *yaw_deg) {
  if (yaw_deg == NULL) {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = QMC5883P_ReadRaw(&data);
  if (status != HAL_OK)
    return status;

  /* Apply hard-iron offsets */
  compensated_data.x = (int16_t)((float)data.x - offsetX);
  compensated_data.y = (int16_t)((float)data.y - offsetY);
  compensated_data.z = (int16_t)((float)data.z - offsetZ);

  /* Raw -> float */
  const float mx = (float)compensated_data.x;
  const float my = (float)compensated_data.y;
  const float mz = (float)compensated_data.z;

  /* Roll/pitch radians */
  const float roll = deg2rad(roll_deg);
  const float pitch = deg2rad(pitch_deg);

  const float cr = cosf(roll);
  const float sr = sinf(roll);
  const float cp = cosf(pitch);
  const float sp = sinf(pitch);

  /* Tilt compensation (standard) */
  const float Xh = mx * cp + mz * sp;
  const float Yh = mx * sr * sp + my * cr + mz * sr * cp;

  float heading = atan2f(Yh, Xh) * (180.0f / (float)M_PI);

  /* Wrap to [0..360) */
  heading = wrap360f(heading);

  /* Round to nearest integer degree and keep in [0..359] */
  heading = roundf(heading);
  if (heading >= 360.0f) heading -= 360.0f;

  /* Robust, low-lag smoothing (no MPU6050) */
  uint32_t now = HAL_GetTick();
  float dt_s;

  if (!yaw_flt.inited) {
    yaw_flt.last_ms = now;
    dt_s = 0.01f; /* default */
  } else {
    uint32_t dms = now - yaw_flt.last_ms;
    yaw_flt.last_ms = now;
    dt_s = (dms > 0u) ? ((float)dms * 0.001f) : 0.001f;
    if (dt_s > 0.05f) dt_s = 0.05f; /* guard long pauses */
  }

  *yaw_deg = one_euro_angle_update(&yaw_flt, heading, dt_s);
  return HAL_OK;
}

