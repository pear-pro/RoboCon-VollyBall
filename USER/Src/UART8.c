/**
  ******************************************************************************
  * @file    UART8.c
  * @brief   UART8 上位机通信模块 — DMA接收/发送 + 控制帧解析
  *
  *          - 使用 IDLE 中断 + DMA 接收不定长数据
  *          - 基于 UART_Protocol 框架进行帧解析与命令分发
  *          - 控制帧 (ID=0x01) payload = 4×float (前进/左右/旋转/保留)
  *          - 心跳帧 (ID=0x00) 自动回复 ID=0xFF
  ******************************************************************************
  */

#include "UART8.h"

/* ========================= 全局变量 ========================= */

float_bytes_t g_control[UART8_CTRL_COUNT] = {0};   /* 控制数据: 前进/左右/旋转/保留 */

/* ── DMA 接收缓冲区 (双缓冲思想, 当前用单缓冲) ── */
static uint8_t  g_rxDMABuf[UART_RX_BUF_SIZE];       /* DMA 接收目标缓冲            */
static uint16_t g_rxDMALen = 0;                      /* 上一次 DMA 接收到的字节数    */

/* ── 定时发送 ── */
volatile uint16_t g_uart8_timTick  = 0;                    /* 10ms 定时器计数 (TIM回调维护) */
volatile uint8_t  g_uart8_reportflag = 0;                    /* 定时发送标志 (TIM回调置位)   */

volatile uint8_t g_uart8_crtlframeflag = 0;                    /* 当前帧解析完成标志 (协议解析器置位) */

/* ==================== 命令回调函数 ==================== */

/**
 * @brief  心跳响应回调 (ID=0x00)
 * @note   收到上位机心跳帧后回复 ID=0xFF (无 payload)
 */
static void Cmd_HeartBeat_Response(uint8_t id, uint8_t *payload, uint8_t len)
{
    (void)id;
    (void)payload;
    (void)len;
    UART_SendFrame(0xFF, NULL, 0);
}

/**
 * @brief  控制帧解析回调 (ID=0x01)
 * @note   将 payload 按 4 字节一组解析为 float，存入 g_control
 *         payload 长度应为 16 字节 (4×float)
 */
static void Cmd_ControlFrame_Analyze(uint8_t id, uint8_t *payload, uint8_t len)
{
    (void)id;

    if (payload == NULL || len == 0)
        return;

    /* 按 4 字节解析 float (小端序) */
    uint8_t i;
    for (i = 0; i < len && i < (UART8_CTRL_COUNT * 4U); i += 4)
    {
        uint8_t idx = i >> 2;  /* i/4 */
        if (idx >= UART8_CTRL_COUNT)
            break;

        g_control[idx].bytes[0] = payload[i];
        g_control[idx].bytes[1] = payload[i + 1];
        g_control[idx].bytes[2] = payload[i + 2];
        g_control[idx].bytes[3] = payload[i + 3];
    }
    g_uart8_crtlframeflag = 1;  // 设置控制帧解析完成标志
    
}

/* ==================== DMA 接收 ==================== */

/**
 * @brief  启动 UART8 DMA 空闲接收
 * @note   使用 HAL_UARTEx_ReceiveToIdle_DMA 接收不定长数据，
 *         硬件 IDLE 中断触发后自动调用 HAL_UARTEx_RxEventCallback
 */
void UART8_StartRxDMA(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8, g_rxDMABuf, UART_RX_BUF_SIZE);
    /* 注意: 若 DMA 已在运行，先 Abort 再重启会更安全，
     *       但 HAL_UARTEx_ReceiveToIdle_DMA 内部会检查状态。 */
}

/**
 * @brief  UART IDLE / DMA 接收完成回调
 * @note   HAL 库在以下情况会调用此回调:
 *         1. 硬件 IDLE 事件 (帧间隔超时)
 *         2. DMA 缓冲区满
 *         3. 收到指定字节数
 *         本函数将接收到的数据送入协议解析器，然后重启 DMA 接收。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != UART8)
        return;

    g_rxDMALen = Size;

    if (Size > 0 && Size <= UART_RX_BUF_SIZE)
    {
        /* 将接收到的字节逐个送入协议解析器 */
        uint16_t i;
        for (i = 0; i < Size; i++)
        {
            UART_RecvByte(g_rxDMABuf[i]);
        }
    }

    /* 重新启动 DMA 空闲接收 */
    UART8_StartRxDMA();
}

/* ==================== DMA 发送 ==================== */

/**
 * @brief  发送控制数据 (4×float → 16 字节 payload)
 * @param  data  指向 4 个 float 的数组
 * @retval 0=成功, 非0=失败
 */
uint8_t UART8_SendControl(const float data[UART8_CTRL_COUNT])
{
    if (data == NULL)
        return 1;

    /* 将 float 数组转换为字节数组 */
    uint8_t payload[UART8_CTRL_COUNT * 4];
    uint8_t i;
    for (i = 0; i < UART8_CTRL_COUNT; i++)
    {
        float_bytes_t fb;
        fb.f = data[i];
        payload[i * 4 + 0] = fb.bytes[0];
        payload[i * 4 + 1] = fb.bytes[1];
        payload[i * 4 + 2] = fb.bytes[2];
        payload[i * 4 + 3] = fb.bytes[3];
    }

    UART_ErrorCode ret = UART_SendFrame(0x01, payload, sizeof(payload));
    return (ret == UART_OK) ? 0 : 1;
}

/**
 * @brief  发送自定义数据帧 (thin wrapper)
 * @param  id    命令 ID
 * @param  data  负载数据
 * @param  len   负载长度
 * @retval 0=成功, 非0=失败
 */
uint8_t UART8_SendFrame(uint8_t id, const uint8_t *data, uint8_t len)
{
    UART_ErrorCode ret = UART_SendFrame(id, (uint8_t *)data, len);
    return (ret == UART_OK) ? 0 : 1;
}

/* ==================== 初始化 ==================== */

/**
 * @brief  初始化 UART8 通信模块
 * @note   1. 初始化协议框架 (绑定 huart8)
 *         2. 注册命令回调
 *         3. 启动 DMA 空闲接收
 */
void UART8_Init(void)
{
    /* 1. 初始化协议框架 */
    UART_Protocol_Init(&huart8);

    /* 2. 注册命令回调 */
    UART_RegisterCmd(0x00, Cmd_HeartBeat_Response);
    UART_RegisterCmd(0x01, Cmd_ControlFrame_Analyze);

    /* 3. 启动 DMA 空闲接收 */
    UART8_StartRxDMA();
}

/* ==================== 周期任务 ==================== */

/**
 * @brief  周期任务 — 在主循环中调用
 * @note   排空接收缓冲区、检测帧超时。
 *         可在此扩展定时上报逻辑 (检查 g_uart8_reportflag)。
 */
void UART8_Process(void)
{
    /* 排空协议接收缓冲区并解析 */
    UART_Process();

    /* ── 定时上报 (示例: 每 100ms 上报一次控制数据) ── */
    if (g_uart8_reportflag)
    {
        g_uart8_reportflag = 0;

        /* 上报当前控制数据 (可根据需要修改上报内容) */
        float report[UART8_CTRL_COUNT];
        report[UART8_CTRL_FORWARD]  = g_control[UART8_CTRL_FORWARD].f;
        report[UART8_CTRL_LATERAL]  = g_control[UART8_CTRL_LATERAL].f;
        report[UART8_CTRL_ROTATION] = g_control[UART8_CTRL_ROTATION].f;
        report[UART8_CTRL_RESERVED] = g_control[UART8_CTRL_RESERVED].f;
        UART8_SendControl(report);
    }
}
