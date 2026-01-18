#include "debug_uart.h"
#include "string.h"
#include "stdio.h"

#define DEBUG_UART_TIMEOUT_MS 10
#define DEBUG_UART_LINE_BUF   128

static UART_HandleTypeDef *s_debug_huart = &huart6;

static void DebugUart_SendBytes(const uint8_t *data, uint16_t len)
{
    if ((s_debug_huart == NULL) || (data == NULL) || (len == 0)) {
        return;
    }
    HAL_UART_Transmit(s_debug_huart, (uint8_t *)data, len, DEBUG_UART_TIMEOUT_MS);
}

void DebugUart_SetHandle(UART_HandleTypeDef *huart)
{
    if (huart != NULL) {
        s_debug_huart = huart;
    }
}

void DebugUart_SendPlotAscii(const float *values, uint8_t count)
{
    char buf[DEBUG_UART_LINE_BUF];
    size_t used = 0;
    uint8_t i;

    if ((values == NULL) || (count == 0)) {
        return;
    }
    if (count > DEBUG_UART_MAX_CHANNELS) {
        count = DEBUG_UART_MAX_CHANNELS;
    }

    for (i = 0; i < count; i++) {
        int wrote = snprintf(buf + used, sizeof(buf) - used,
                             (i == (count - 1)) ? "%.3f\r\n" : "%.3f,",
                             (double)values[i]);
        if (wrote < 0) {
            return;
        }
        used += (size_t)wrote;
        if (used >= sizeof(buf)) {
            buf[sizeof(buf) - 3] = '\r';
            buf[sizeof(buf) - 2] = '\n';
            buf[sizeof(buf) - 1] = '\0';
            used = sizeof(buf) - 1;
            break;
        }
    }

    DebugUart_SendBytes((uint8_t *)buf, (uint16_t)strlen(buf));
}

void DebugUart_SendPlotVofa(const float *values, uint8_t count)
{
    uint8_t i;
    uint8_t buf[DEBUG_UART_MAX_CHANNELS * 4 + 4];
    uint16_t len = 0;
    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

    if ((values == NULL) || (count == 0)) {
        return;
    }
    if (count > DEBUG_UART_MAX_CHANNELS) {
        count = DEBUG_UART_MAX_CHANNELS;
    }

    for (i = 0; i < count; i++) {
        union {
            float f;
            uint8_t b[4];
        } u;
        u.f = values[i];
        memcpy(&buf[len], u.b, 4);
        len += 4;
    }

    memcpy(&buf[len], tail, sizeof(tail));
    len += sizeof(tail);

    DebugUart_SendBytes(buf, len);
}
