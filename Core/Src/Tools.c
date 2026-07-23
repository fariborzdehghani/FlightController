#include "Tools.h"
#include <stdio.h>
#include <string.h>

#define LOG_QUEUE_SIZE         2048u
#define LOG_FORMAT_BUFFER_SIZE 256u

extern UART_HandleTypeDef huart1;

static uint8_t log_queue[LOG_QUEUE_SIZE];
static volatile uint16_t log_head;
static volatile uint16_t log_tail;
static volatile uint16_t active_tx_length;
static volatile uint8_t tx_in_progress;
static volatile uint32_t dropped_logs;

static uint16_t Tools_QueueFree(uint16_t head, uint16_t tail) {
  if (head >= tail) {
    return (uint16_t)(LOG_QUEUE_SIZE - (head - tail) - 1u);
  }
  return (uint16_t)(tail - head - 1u);
}

static uint16_t Tools_CopyToQueue(uint16_t head, const uint8_t *data,
                                  uint16_t length) {
  const uint16_t first =
      (length < (LOG_QUEUE_SIZE - head)) ? length : (LOG_QUEUE_SIZE - head);
  memcpy(&log_queue[head], data, first);
  if (length > first) {
    memcpy(log_queue, &data[first], length - first);
  }
  return (uint16_t)((head + length) % LOG_QUEUE_SIZE);
}

static bool Tools_QueueMessage(const char *data, uint16_t length) {
  if (data == NULL || length == 0u || length >= LOG_QUEUE_SIZE - 1u) {
    return false;
  }

  const uint16_t head = log_head;
  const uint16_t tail = log_tail;
  const uint16_t total_length = (uint16_t)(length + 1u);
  if (Tools_QueueFree(head, tail) < total_length) {
    dropped_logs++;
    return false;
  }

  uint16_t new_head =
      Tools_CopyToQueue(head, (const uint8_t *)data, length);
  const uint8_t newline = '\n';
  new_head = Tools_CopyToQueue(new_head, &newline, 1u);
  __DMB();
  log_head = new_head;
  Tools_ProcessLogQueue();
  return true;
}

void Tools_ProcessLogQueue(void) {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (tx_in_progress || log_tail == log_head) {
    if (!primask) __enable_irq();
    return;
  }

  const uint16_t tail = log_tail;
  active_tx_length = (log_head > tail) ? (uint16_t)(log_head - tail)
                                       : (uint16_t)(LOG_QUEUE_SIZE - tail);
  tx_in_progress = 1u;
  if (!primask) __enable_irq();

  if (HAL_UART_Transmit_IT(&huart1, &log_queue[tail], active_tx_length) !=
      HAL_OK) {
    const uint32_t retry_primask = __get_PRIMASK();
    __disable_irq();
    tx_in_progress = 0u;
    active_tx_length = 0u;
    if (!retry_primask) __enable_irq();
  }
}

void Tools_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == NULL || huart->Instance != USART1 || !tx_in_progress) {
    return;
  }

  log_tail = (uint16_t)((log_tail + active_tx_length) % LOG_QUEUE_SIZE);
  active_tx_length = 0u;
  tx_in_progress = 0u;
  Tools_ProcessLogQueue();
}

void Tools_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == NULL || huart->Instance != USART1 || !tx_in_progress) {
    return;
  }

  log_tail = (uint16_t)((log_tail + active_tx_length) % LOG_QUEUE_SIZE);
  active_tx_length = 0u;
  tx_in_progress = 0u;
  dropped_logs++;
  Tools_ProcessLogQueue();
}

uint32_t Tools_GetDroppedLogCount(void) { return dropped_logs; }

bool LogStringToPC(const char *data) {
  if (data == NULL) {
    return false;
  }

  size_t length = 0u;
  while (length < LOG_QUEUE_SIZE && data[length] != '\0') {
    length++;
  }
  if (length == 0u || length >= LOG_QUEUE_SIZE) {
    return false;
  }
  return Tools_QueueMessage(data, (uint16_t)length);
}

static bool Tools_LogWithLevel(const char *level, int code,
                               const char *message) {
  if (level == NULL || message == NULL) {
    return false;
  }

  char packet[LOG_FORMAT_BUFFER_SIZE];
  const int written = snprintf(packet, sizeof(packet),
                               "%s={\"Code\":%d,\"Content\":\"%s\"}",
                               level, code, message);
  if (written < 0 || (size_t)written >= sizeof(packet)) {
    dropped_logs++;
    return false;
  }
  return LogStringToPC(packet);
}

bool LogError(int code, const char *message) {
  return Tools_LogWithLevel("Error", code, message);
}

bool LogInformation(int code, const char *message) {
  return Tools_LogWithLevel("Information", code, message);
}
