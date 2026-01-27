#ifndef BSP_USART_H
#define BSP_USART_H
//#include "struct_typedef.h"
#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "rc_task.h"
#define SBUS_RX_BUF_NUM 36u
extern void RC_init(void);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);
void USART1_IRQHandlerCallBack(void);
#endif
