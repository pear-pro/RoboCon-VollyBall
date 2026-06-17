/**
  ******************************************************************************
  * @file    UART_Protocol.c
  * @brief   UART 协议框架 — 简化版（单源文件）
  *
  *          整合 CRC8、命令注册/分发、接收状态机、环形缓冲区、
  *          DMA 帧发送于一体。
  ******************************************************************************
  */

#include "UART_Protocol.h"

/* ============================ 内部类型 ============================ */

typedef enum {
    WAIT_SYNC1   = 0,
    WAIT_SYNC2   = 1,
    WAIT_ID      = 2,
    WAIT_LEN     = 3,
    WAIT_PAYLOAD = 4,
    WAIT_CRC     = 5
} RecvState;

typedef struct {
    uint8_t   id;
    CmdHandler handler;
} CmdEntry;

/* ============================ 模块变量 ============================ */

static UART_HandleTypeDef *pHuart = NULL;         /* 绑定的 UART 句柄             */
static UART_Frame          g_rxFrame;             /* 接收帧缓冲区                  */
static CmdEntry            cmdTable[UART_CMD_TABLE_SIZE];
static uint8_t             cmdCount = 0;

/* ── 接收引擎 ── */
static uint8_t   rxRingBuf[UART_RX_BUF_SIZE];
static uint16_t  rxWriteIdx = 0;
static uint16_t  rxReadIdx  = 0;
static RecvState rxState     = WAIT_SYNC1;
static uint8_t   payloadIdx  = 0;
static uint32_t  lastByteTick = 0;

/* ── 发送引擎 ── */
static uint8_t        txBuf[264];        /* 最大帧长 260 字节                  */
static volatile uint8_t txDmaBusy = 0;

/* ======================== CRC-8 查表 ======================== */

static const uint8_t CRC8_Table[256] =
{
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15, 0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65, 0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5, 0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85, 0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2, 0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2, 0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32, 0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42, 0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C, 0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC, 0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C, 0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C, 0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B, 0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B, 0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB, 0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB, 0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3
};

/* ======================== CRC-8 ======================== */

uint8_t CRC8_Calc(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    while (len--)
    {
        crc = CRC8_Table[crc ^ (*data++)];
    }
    return crc;
}

/* ==================== 环形缓冲区 ==================== */

static uint8_t RingBuf_Write(uint8_t byte)
{
    uint16_t next = (rxWriteIdx + 1) & (UART_RX_BUF_SIZE - 1);
    if (next == rxReadIdx) return 0;     /* 满 */
    rxRingBuf[rxWriteIdx] = byte;
    rxWriteIdx = next;
    return 1;
}

static uint8_t RingBuf_Read(uint8_t *byte)
{
    if (rxWriteIdx == rxReadIdx) return 0; /* 空 */
    *byte = rxRingBuf[rxReadIdx];
    rxReadIdx = (rxReadIdx + 1) & (UART_RX_BUF_SIZE - 1);
    return 1;
}

/* ==================== 接收状态机 ==================== */

static void UART_ResetParser(void)
{
    rxState      = WAIT_SYNC1;
    payloadIdx   = 0;
    lastByteTick = HAL_GetTick();
}

static void UART_ParseByte(uint8_t byte)
{
    uint8_t crcBuf[258], calcCrc;
    uint8_t i;

    lastByteTick = HAL_GetTick();

    switch (rxState)
    {
    case WAIT_SYNC1:
        if (byte == UART_SYNC1) { g_rxFrame.sync1 = byte; rxState = WAIT_SYNC2; }
        break;

    case WAIT_SYNC2:
        if (byte == UART_SYNC2)      { g_rxFrame.sync2 = byte; rxState = WAIT_ID; }
        else if (byte == UART_SYNC1) { g_rxFrame.sync1 = byte; /* 重新同步 */ }
        else                         { rxState = WAIT_SYNC1; }
        break;

    case WAIT_ID:
        g_rxFrame.id = byte;
        rxState = WAIT_LEN;
        break;

    case WAIT_LEN:
        if (byte > UART_MAX_PAYLOAD) { rxState = WAIT_SYNC1; break; }
        g_rxFrame.len = byte;
        rxState    = (byte == 0) ? WAIT_CRC : WAIT_PAYLOAD;
        payloadIdx = 0;
        break;

    case WAIT_PAYLOAD:
        g_rxFrame.payload[payloadIdx++] = byte;
        if (payloadIdx >= g_rxFrame.len) rxState = WAIT_CRC;
        break;

    case WAIT_CRC:
        g_rxFrame.crc = byte;
        /* 对 id + len + payload 计算 CRC8 */
        crcBuf[0] = g_rxFrame.id;
        crcBuf[1] = g_rxFrame.len;
        for (i = 0; i < g_rxFrame.len; i++)
            crcBuf[2 + i] = g_rxFrame.payload[i];
        calcCrc = CRC8_Calc(crcBuf, 2 + g_rxFrame.len);
        if (calcCrc == g_rxFrame.crc)
        {
            /* CRC 通过 → 分发 */
            for (i = 0; i < cmdCount; i++)
            {
                if (cmdTable[i].id == g_rxFrame.id && cmdTable[i].handler != NULL)
                {
                    cmdTable[i].handler(g_rxFrame.id, g_rxFrame.payload, g_rxFrame.len);
                    break;
                }
            }
        }
        /* 重置以接收下一帧 */
        rxState    = WAIT_SYNC1;
        payloadIdx = 0;
        break;
    }
}

static void UART_FlushRxBuffer(void)
{
    uint8_t byte;
    while (RingBuf_Read(&byte))
        UART_ParseByte(byte);
}

/* ==================== 发送引擎 ==================== */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == pHuart)
        txDmaBusy = 0;
}

UART_ErrorCode UART_SendFrame(uint8_t id, uint8_t *data, uint8_t len)
{
    uint8_t crcBuf[258], i, txLen;

    if (pHuart == NULL)              return UART_ERR_BUSY;
    if (len > UART_MAX_PAYLOAD)      return UART_ERR_INVALID_LEN;

    /* 等待上一次 DMA 发送完成（超时 100ms） */
    {
        uint32_t startTick = HAL_GetTick();
        while (txDmaBusy)
        {
            if ((HAL_GetTick() - startTick) > 100U)
            {
                if (txDmaBusy) return UART_ERR_BUSY;
            }
        }
    }

    /* 组装帧 */
    txBuf[0] = UART_SYNC1;
    txBuf[1] = UART_SYNC2;
    txBuf[2] = id;
    txBuf[3] = len;
    for (i = 0; i < len; i++)
        txBuf[4 + i] = (data != NULL) ? data[i] : 0;

    /* CRC8 over id+len+payload */
    crcBuf[0] = id;
    crcBuf[1] = len;
    for (i = 0; i < len; i++)
        crcBuf[2 + i] = (data != NULL) ? data[i] : 0;
    txBuf[4 + len] = CRC8_Calc(crcBuf, 2 + len);

    txLen     = 4 + len + 1;    /* sync(2)+id(1)+len(1)+payload+crc(1) */
    txDmaBusy = 1;

    if (HAL_UART_Transmit_DMA(pHuart, txBuf, txLen) != HAL_OK)
    {
        txDmaBusy = 0;
        return UART_ERR_BUSY;
    }
    return UART_OK;
}

/* ==================== 命令注册 / 初始化 ==================== */

void UART_RegisterCmd(uint8_t id, CmdHandler handler)
{
    uint8_t i;
    for (i = 0; i < cmdCount; i++)
    {
        if (cmdTable[i].id == id)
        {
            cmdTable[i].handler = handler;
            return;
        }
    }
    if (cmdCount < UART_CMD_TABLE_SIZE)
    {
        cmdTable[cmdCount].id      = id;
        cmdTable[cmdCount].handler = handler;
        cmdCount++;
    }
}

void UART_Protocol_Init(UART_HandleTypeDef *huart)
{
    pHuart   = huart;
    cmdCount = 0;
    txDmaBusy = 0;
    UART_ResetParser();
}

/* ==================== 公开接口 ==================== */

void UART_RecvByte(uint8_t byte)
{
    RingBuf_Write(byte);
}

void UART_Process(void)
{
    /* 帧超时检测 */
    if (rxState != WAIT_SYNC1)
    {
        if ((HAL_GetTick() - lastByteTick) > UART_FRAME_TIMEOUT_MS)
            UART_ResetParser();
    }
    UART_FlushRxBuffer();
}

void UART_RxCpltCallback(void)
{
    UART_FlushRxBuffer();
}
