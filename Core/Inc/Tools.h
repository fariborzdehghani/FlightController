#ifndef __TOOLS_H
#define __TOOLS_H

#include "main.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool LogStringToPC(const char *data);
bool LogError(int code, const char *message);
bool LogInformation(int code, const char *message);
void Tools_ProcessLogQueue(void);
void Tools_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void Tools_UART_ErrorCallback(UART_HandleTypeDef *huart);
uint32_t Tools_GetDroppedLogCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __TOOLS_H */
