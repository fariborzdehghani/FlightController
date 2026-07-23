#ifndef SRF05_H
#define SRF05_H

#include "stm32f4xx_hal.h"

/* Measurements below this distance are not accepted. */
#define SRF05_MIN_VALID_DISTANCE_CM 8.0f

/* Measurements above the practical SRF05 range are not accepted. */
#define SRF05_MAX_VALID_DISTANCE_CM 400.0f

typedef struct {
  TIM_HandleTypeDef *timer;
  uint32_t captureChannel;
  GPIO_TypeDef *triggerPort;
  uint16_t triggerPin;
  uint32_t timeout_ms;
  float timer_tick_us;
} SRF05_Config_t;

HAL_StatusTypeDef SRF05_Init(SRF05_Config_t config);

/* Nonblocking API. One caller must own the start/read sequence. */
HAL_StatusTypeDef SRF05_StartMeasurement(void);
HAL_StatusTypeDef SRF05_ReadMeasurement(float *distance_cm,
                                        uint32_t *sample_time_ms);
void SRF05_CancelMeasurement(void);

/* Blocking compatibility API. Call only from main/thread context. */
HAL_StatusTypeDef SRF05_MeasureDistance(float *distance_cm);

#endif /* SRF05_H */
