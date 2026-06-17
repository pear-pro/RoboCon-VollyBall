/**
  ******************************************************************************
  * @file    UART_Protocol.h
  * @brief   UART 协议框架 — 简化版（单头文件）
  *
  *          包含所有公开 API、类型定义、可配置宏。
  *          帧格式: SYNC1 | SYNC2 | ID | LEN | PAYLOAD | CRC8
  *          CRC-8 多项式 0x07，覆盖 (ID + LEN + PAYLOAD)。
  ******************************************************************************
  */
#ifndef __UART_PROTOCOL_H__
#define __UART_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* ─────────────────────── 用户可配置宏 ─────────────────────── */

#define UART_SYNC1             0x5AU  /* 帧同步字节1                        */
#define UART_SYNC2             0xA5U  /* 帧同步字节2                        */
#define UART_MAX_PAYLOAD       255U   /* 最大负载长度（字节）                */
#define UART_RX_BUF_SIZE       512U   /* 接收环形缓冲区大小（须为 2^n）      */
#define UART_CMD_TABLE_SIZE    32U    /* 最大可注册命令数                    */
#define UART_FRAME_TIMEOUT_MS  100U   /* 帧内字节间最大空闲时间 (ms)         */

/* ─────────────────────── 类型定义 ─────────────────────── */

/** @brief UART 通信帧 */
typedef struct
{
    uint8_t sync1;             /* 同步字节1 (0x5A)                         */
    uint8_t sync2;             /* 同步字节2 (0xA5)                         */
    uint8_t id;                /* 命令/消息 ID                             */
    uint8_t len;               /* 负载长度 (0~255)                         */
    uint8_t payload[256];      /* 负载数据                                 */
    uint8_t crc;               /* CRC-8，覆盖 id+len+payload               */
} UART_Frame;

/** @brief 返回码 */
typedef enum
{
    UART_OK              = 0x00,
    UART_ERR_CRC         = 0x01,
    UART_ERR_TIMEOUT     = 0x02,
    UART_ERR_OVERFLOW    = 0x03,
    UART_ERR_INVALID_LEN = 0x04,
    UART_ERR_BUSY        = 0x05
} UART_ErrorCode;

/** @brief 命令处理回调 */
typedef void (*CmdHandler)(uint8_t id, uint8_t *payload, uint8_t len);

/* ─────────────────────── 公开 API ─────────────────────── */

/**
 * @brief  初始化协议框架。
 * @param  huart  用于发送的 UART 句柄。
 * @note   在任何其他 API 之前调用；重置接收状态机与命令表。
 */
void UART_Protocol_Init(UART_HandleTypeDef *huart);

/**
 * @brief  注册命令处理回调（相同 ID 会覆盖旧回调）。
 * @param  id       命令 ID (0x00 ~ 0xFF)
 * @param  handler  回调函数指针
 */
void UART_RegisterCmd(uint8_t id, CmdHandler handler);

/**
 * @brief  组装并通过 DMA 发送一帧。
 * @param  id    命令 ID
 * @param  data  负载数据指针（len==0 时可为 NULL）
 * @param  len   负载长度（0~255）
 * @return UART_OK 或 UART_ERR_BUSY / UART_ERR_INVALID_LEN
 */
UART_ErrorCode UART_SendFrame(uint8_t id, uint8_t *data, uint8_t len);

/**
 * @brief  将收到的单个字节送入协议解析器。
 * @param  byte  从 UART 接收到的字节
 * @note   在 UART 接收中断/回调中调用。
 */
void UART_RecvByte(uint8_t byte);

/**
 * @brief  周期性任务 — 必须在主循环中调用（每 10~50ms）。
 * @note   检测帧超时、排空接收缓冲。
 */
void UART_Process(void);

/**
 * @brief  便捷函数：在 HAL_UART_RxCpltCallback 中调用以立即排空缓冲。
 */
void UART_RxCpltCallback(void);

/**
 * @brief  对数据缓冲区计算 CRC-8。
 * @param  data  待校验数据指针
 * @param  len   待处理字节数
 * @return 8 位 CRC 值
 */
uint8_t CRC8_Calc(uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __UART_PROTOCOL_H__ */
