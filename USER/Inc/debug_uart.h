#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdint.h>
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_UART_MAX_CHANNELS 8

void DebugUart_SetHandle(UART_HandleTypeDef *huart);
void DebugUart_SendPlotAscii(const float *values, uint8_t count);
void DebugUart_SendPlotVofa(const float *values, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif
