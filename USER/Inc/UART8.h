/**
  ******************************************************************************
  * @file    UART8.h
  * @brief   UART8 上位机通信模块 — DMA接收/发送 + 控制帧解析
  *
  *          协议帧格式: SYNC1(0x5A) | SYNC2(0xA5) | ID | LEN | PAYLOAD | CRC8
  *          ID 0x00: 心跳帧 (上位机→下位机)
  *          ID 0x01: 控制帧 (上位机→下位机, payload = 4×float)
  *          ID 0xFF: 心跳响应 (下位机→上位机)
  ******************************************************************************
  */
#ifndef __UART8_H__
#define __UART8_H__

#ifdef __cplusplus
extern "C" {
#endif
 
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "UART_Protocol.h"
#include <stdint.h>
/* ───────────────────── 类型定义 ───────────────────── */

/** @brief float/uint8_t 共用体，用于控制帧解析 */
typedef union {
    float   f;
    uint8_t bytes[4];
} float_bytes_t;

/* ───────────────────── 控制数据索引 ───────────────────── */

#define UART8_CTRL_FORWARD   0   /* 前进速度 (m/s)            */
#define UART8_CTRL_LATERAL   1   /* 左右速度 (m/s)            */
#define UART8_CTRL_ROTATION  2   /* 旋转速度 (rad/s)          */
#define UART8_CTRL_RESERVED  3   /* 保留                       */
#define UART8_CTRL_COUNT     4   /* 控制量总数                  */

/* ───────────────────── 全局变量声明 ───────────────────── */

extern float_bytes_t g_control[UART8_CTRL_COUNT];   /* 解析后的控制数据           */
extern volatile uint16_t g_uart8_timTick;                  /* 10ms 定时器计数 (外部维护)  */
extern volatile uint8_t  g_uart8_reportflag;                 /* 定时发送标志 (外部置位)    */
extern volatile uint8_t g_uart8_crtlframeflag;                 /* 当前帧解析完成标志 (协议解析器置位) */
extern volatile uint8_t g_uart8_comm_ok;                       /* 通信正常标志 (握手回应后置1)      */

/* ───────────────────── API 函数 ───────────────────── */

/**
 * @brief  初始化 UART8 通信模块。
 * @note   注册命令回调、启动 DMA 空闲接收。
 *         在 All_Init() 中调用。
 */
void UART8_Init(void);

/**
 * @brief  启动 / 重新启动 DMA 空闲接收。
 * @note   在 HAL_UARTEx_RxEventCallback 中接收完成后调用，
 *         也可在上电初始化时手动调用。
 */
void UART8_StartRxDMA(void);

/**
 * @brief  周期任务 — 在主循环中调用 (1~10ms 周期)。
 * @note   排空接收缓冲区、解析帧、检测帧超时；
 *         检查 g_uart8_reportflag 标志以触发定时上报。
 */
void UART8_Process(void);

/**
 * @brief  通过 DMA 发送自定义数据帧。
 * @param  id    命令 ID
 * @param  data  负载数据指针 (len==0 时可为 NULL)
 * @param  len   负载长度 (0~255)
 * @retval 0=成功, 非0=失败
 */
uint8_t UART8_SendFrame(uint8_t id, const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __UART8_H__ */
