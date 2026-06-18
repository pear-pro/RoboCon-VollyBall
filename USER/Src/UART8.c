/**
  ******************************************************************************
  * @file    UART8.c
  * @brief   UART8 上位机通信模块 — DMA接收/发送 + 控制帧解析
  *
  *          - 使用 IDLE 中断 + DMA 接收不定长数据
  *          - 基于 UART_Protocol 框架进行帧解析与命令分发
  *          - ID 0xF0: 握手帧 (MCU上电发送→上位机回应→通信建立)
  *          - ID 0xF1: 底盘控制帧 (上位机→下位机, payload = 4×float)
  *          - ID 0xF2: 击球控制帧 (上位机→下位机, payload = 2×uint8)
  *          - ID 0x00: 心跳帧 (上位机→下位机, payload = 0)
  ******************************************************************************
  */

#include "UART8.h"
#include "FSM.h"
#include "car_ctrl.h"
#include "includes.h"

/* ========================= 全局变量 ========================= */

float_bytes_t g_control[UART8_CTRL_COUNT] = {0};   /* 控制数据: 前进/左右/旋转/保留 */

/* ── DMA 接收缓冲区 (双缓冲思想, 当前用单缓冲) ── */
static uint8_t  g_rxDMABuf[UART_RX_BUF_SIZE];       /* DMA 接收目标缓冲            */
static uint16_t g_rxDMALen = 0;                      /* 上一次 DMA 接收到的字节数    */

/* ── 定时发送 ── */
volatile uint16_t g_uart8_timtick  = 0;                    /* 10ms 定时器计数 (TIM3回调维护) */
volatile uint8_t  g_uart8_reportflag = 0;                    /* 定时发送标志 （暂时保留）*/

volatile uint8_t g_uart8_comm_ok      = 0;                    /* 通信正常标志 (握手回应后置1)      */
volatile uart8_hit_state_t g_uart8_hitstate = UART8_HIT_READY; /* 击球状态 */

/* ── 握手重试 ── */
#define UART8_HANDSHAKE_TIMEOUT_MS  500U    /* 握手超时重试间隔 (ms)         */
static uint32_t hs_last_tick   = 0;         /* 上次握手发送时刻          */
volatile uint16_t g_uart8_norx_tick = 0; /* 未收到数据计时 */

#define xUART8_DEBUG  /* 调试模式: 不强制通信正常，允许直接处理命令 */

static void Is_Comm_OK(void)
{
    #ifdef UART8_DEBUG
    // 在调试模式下不强制通信正常，允许直接处理命令以测试控制逻辑
    #else
        if(!g_uart8_comm_ok)
            return;
    #endif
}

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
    g_uart8_norx_tick = 0; /* 收到数据，重置未收到数据计时 */
}

/**
 * @brief  握手回应回调 (ID=0xF0)
 * @note   收到上位机对握手帧的回应后，置位通信正常标志。
 *         握手重试机制检测到 g_uart8_comm_ok==1 后自动停止。
 */
static void Cmd_HandShake_Response(uint8_t id, uint8_t *payload, uint8_t len)
{
    (void)id;
    (void)payload;
    (void)len;
    g_uart8_comm_ok = 1;  /* 收到上位机握手回应，通信正常 */
    g_uart8_norx_tick = 0; /* 重置未收到数据计时 */
}

/**
 * @brief  底盘控制帧解析回调 (ID=0xF1)
 * @note   将 payload 按 4 字节一组解析为 float，存入 g_control
 *         payload 长度应为 16 字节 (4×float)
 */
static void Cmd_ControlFrame_Analyze(uint8_t id, uint8_t *payload, uint8_t len)
{
   Is_Comm_OK();
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
    car_tary = g_control[0].f*1000.0/5.0; // 前进速度
    car_tarx = -g_control[1].f*1000.0/5.0; // 左右速度
    g_uart8_norx_tick = 0; /* 收到数据，重置未收到数据计时 */
}

/**
 * @brief  击球控制帧解析回调 (ID=0xF2)
 * @note   仅在上一次击球完成后 (g_uart8_hitstate==READY) 才处理新命令。
 *         payload[0] = 角度预设档位 (0=大角度, 1=中角度, 2=小角度)
 *         payload[1] = 击球动作 (1=按下击球, 0=松手回零)
 * @note   处理后将 g_uart8_hitstate 置为 PROCESSING，由 pid_tim.c 中的
 *         击球冷却逻辑在 1s 后恢复到 READY 状态。
 */
static void Cmd_HitFrame_Analyze(uint8_t id, uint8_t *payload, uint8_t len)
{
    Is_Comm_OK();
    (void)id;
    if(payload == NULL || len < 2)
        return;
    if(g_uart8_hitstate != UART8_HIT_READY)
        return; // 只有在击球准备就绪状态才处理新的击球命令，避免覆盖正在处理的命令
    uint8_t preset = payload[0];
    uint8_t action = payload[1];
    /* 根据 preset 和 action 执行相应的击球动作 */
    if(preset < 3) // 预设0-2为不同的击球角度
        hit_set_preset(preset);
    if(action == 1)
        hit_request_press();
    else
        hit_request_release();
    g_uart8_hitstate = UART8_HIT_PROCESSING; // 设置击球状态为处理中
    g_uart8_norx_tick = 0; /* 收到数据，重置未收到数据计时 */
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
 * @brief  发送握手帧 (ID=0xF0)
 * @note   上电时由 UART8_Init 调用首次握手；通信未建立时由 UART8_Process
 *         中的重试机制周期性调用；击球完成后由 pid_tim.c 调用以通知上位机。
 */
void UART8_HandShake(void)
{
    UART_SendFrame(0xF0,NULL,0);
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
    UART_RegisterCmd(0xF0, Cmd_HandShake_Response);
    UART_RegisterCmd(0xF1, Cmd_ControlFrame_Analyze);
    // UART_RegisterCmd(0xF2, Cmd_HitFrame_Analyze);

    /* 3. 启动 DMA 空闲接收 */
    UART8_StartRxDMA();

    /* 4. 初始化握手重试状态并发送首次握手 */
    hs_last_tick = HAL_GetTick();
    UART8_HandShake();
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

    /* 握手超时重试：通信未建立时无限重试，每 500ms 发送一次 */
    if (!g_uart8_comm_ok)
    {
        if ((HAL_GetTick() - hs_last_tick) >= UART8_HANDSHAKE_TIMEOUT_MS)
        {
            UART8_HandShake();
            hs_last_tick = HAL_GetTick();
        }
    }
}
