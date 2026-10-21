#ifndef HOST_STM32_HAL_H
#define HOST_STM32_HAL_H

/* Minimal HAL surface for the flight-control and SRF05 host tests. */
#include <stddef.h>
#include <stdint.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { int unused; } I2C_HandleTypeDef;
typedef struct { uint32_t CCER; } TIM_TypeDef;
typedef struct {
  TIM_TypeDef *Instance;
  uint32_t Channel, flags, arr, captured, interrupts;
  uint32_t polarity, counter_enabled, counter;
} TIM_HandleTypeDef;
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { uint32_t pin_state; } GPIO_TypeDef;
uint32_t HAL_GetTick(void);

#define TIM_CHANNEL_1 0u
#define TIM_CHANNEL_2 4u
#define TIM_CHANNEL_3 8u
#define TIM_CHANNEL_4 12u
#define HAL_TIM_ACTIVE_CHANNEL_1 1u
#define HAL_TIM_ACTIVE_CHANNEL_2 2u
#define HAL_TIM_ACTIVE_CHANNEL_3 4u
#define HAL_TIM_ACTIVE_CHANNEL_4 8u
#define TIM_FLAG_UPDATE 1u
#define TIM_FLAG_CC1 2u
#define TIM_FLAG_CC2 4u
#define TIM_FLAG_CC3 8u
#define TIM_FLAG_CC4 16u
#define TIM_FLAG_CC1OF 512u
#define TIM_FLAG_CC2OF 1024u
#define TIM_FLAG_CC3OF 2048u
#define TIM_FLAG_CC4OF 4096u
#define TIM_IT_UPDATE 1u
#define TIM_INPUTCHANNELPOLARITY_RISING 0u
#define TIM_INPUTCHANNELPOLARITY_FALLING 1u
#define TIM_CCx_DISABLE 0u
#define TIM_CCx_ENABLE 1u
#define GPIO_PIN_RESET 0u
#define GPIO_PIN_SET 1u
#define RESET 0u
#define __HAL_TIM_DISABLE_IT(t, m) ((t)->interrupts &= ~(m))
#define __HAL_TIM_ENABLE_IT(t, m) ((t)->interrupts |= (m))
#define __HAL_TIM_SET_CAPTUREPOLARITY(t, c, p) ((void)(c), (t)->polarity = (p))
#define __HAL_TIM_CLEAR_FLAG(t, m) ((t)->flags &= ~(m))
#define __HAL_TIM_GET_FLAG(t, m) ((t)->flags & (m))
#define __HAL_TIM_GET_AUTORELOAD(t) ((t)->arr)
#define __HAL_TIM_GET_COUNTER(t) ((t)->counter)
#define __get_PRIMASK() 0u
#define __get_IPSR() 0u
#define __get_BASEPRI() 0u
#define __get_FAULTMASK() 0u
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
#define __NOP() ((void)0)
extern uint32_t SystemCoreClock;
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *, uint32_t);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *, uint32_t);
void TIM_CCxChannelCmd(TIM_TypeDef *, uint32_t, uint32_t);

#endif
