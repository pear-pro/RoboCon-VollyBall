#include "debug_uart.h"
#include "usart.h"
#include "motor_can.h"
#include <stdlib.h>

extern motor_info_t C620_up_angle;
 
/*...........示例..............
    float f3[3]={88.77,0.66,55.44};
    Vofa_JustFloat(f3, 3);
        ...........示例..............*/

void Vofa_JustFloat(float *_data, uint8_t _num)
{
    uint8_t tempData[100];
    uint8_t temp_end[4] = {0, 0, 0x80, 0x7F};
    float temp_copy[_num];

    memcpy(&temp_copy, _data, sizeof(float) * _num);

    memcpy(tempData, (uint8_t *)&temp_copy, sizeof(temp_copy));
    memcpy(&tempData[_num * 4], &temp_end[0], 4);

    //....在此替换串口发送函�?..........
    HAL_UART_Transmit_DMA(&huart6, tempData, (_num + 1) * 4);
    //......................................
}
#define DEBUG_TUNE_LINE_MAX 128U
#define DEBUG_TUNE_TX_MAX 192U

typedef struct
{
    uint32_t ms;
    float target;
    float angle;
    float error;
    float speed;
    int16_t torque;
    int16_t out;
    float angle_kp;
    float angle_ki;
    float angle_kd;
    float speed_kp;
    float speed_ki;
    float speed_kd;
    int32_t angle_maxout;
    int32_t speed_maxout;
} debug_tune_snapshot_t;

static uint8_t tune_rx_byte;
static char tune_rx_line[DEBUG_TUNE_LINE_MAX];
static char tune_cmd_line[DEBUG_TUNE_LINE_MAX];
static volatile uint16_t tune_rx_len;
static volatile uint8_t tune_cmd_ready;
static volatile uint8_t tune_log_enabled;
static volatile uint8_t tune_report_divider = 5U;
static volatile uint8_t tune_report_tick;
static volatile uint8_t tune_snapshot_pending;
static volatile uint8_t tune_control_active;
static volatile uint32_t tune_ms;
static debug_tune_snapshot_t tune_snapshot;

static int32_t tune_scale_float(float value, float scale)
{
    float scaled = value * scale;
    return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static void tune_send_text(const char *text)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)text, (uint16_t)strlen(text), 50U);
}


static void tune_rx_feed_byte(uint8_t ch)
{
    if (ch == '\n' || ch == '\r')
    {
        if (tune_rx_len > 0U && !tune_cmd_ready)
        {
            tune_rx_line[tune_rx_len] = '\0';
            memcpy(tune_cmd_line, tune_rx_line, DEBUG_TUNE_LINE_MAX);
            tune_cmd_ready = 1U;
        }
        tune_rx_len = 0U;
    }
    else if (tune_rx_len < (DEBUG_TUNE_LINE_MAX - 1U))
    {
        tune_rx_line[tune_rx_len++] = (char)ch;
    }
    else
    {
        tune_rx_len = 0U;
    }
}

static void tune_poll_uart_rx(void)
{
    while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET)
    {
        tune_rx_feed_byte((uint8_t)(huart6.Instance->DR & 0xFFU));
    }
}
static void tune_pid_clear_state(pid_t *pid)
{
    pid->error[NOW_ERR] = 0.0f;
    pid->error[LAST_ERR] = 0.0f;
    pid->error[LLAST_ERR] = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->out = 0.0f;
}

static char *tune_next_token(char **cursor)
{
    char *start = *cursor;
    while (*start == ' ' || *start == '\t')
    {
        start++;
    }
    if (*start == '\0')
    {
        *cursor = start;
        return NULL;
    }

    char *end = start;
    while (*end != '\0' && *end != ' ' && *end != '\t')
    {
        end++;
    }
    if (*end != '\0')
    {
        *end = '\0';
        end++;
    }
    *cursor = end;
    return start;
}

static uint8_t tune_parse_float(char **cursor, float *out)
{
    char *token = tune_next_token(cursor);
    char *end = NULL;
    if (token == NULL)
    {
        return 0U;
    }
    *out = strtof(token, &end);
    return (end != token) ? 1U : 0U;
}

static uint8_t tune_parse_int(char **cursor, int32_t *out)
{
    char *token = tune_next_token(cursor);
    char *end = NULL;
    if (token == NULL)
    {
        return 0U;
    }
    *out = strtol(token, &end, 10);
    return (end != token) ? 1U : 0U;
}

static void tune_send_status(const char *prefix)
{
    char tx[DEBUG_TUNE_TX_MAX];
    int len = snprintf(tx, sizeof(tx),
                       "%s,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                       prefix,
                       (long)tune_scale_float(C620_up_angle.target_angle, 100.0f),
                       (long)tune_scale_float(C620_up_angle.Angle_pid.get, 100.0f),
                       (long)tune_scale_float(C620_up_angle.target_angle - C620_up_angle.Angle_pid.get, 100.0f),
                       (long)tune_scale_float(C620_up_angle.Speed_pid.get, 1.0f),
                       (long)C620_up_angle.Rxmsg.Torque,
                       (long)tune_scale_float(C620_up_angle.Angle_pid.kp, 1000.0f),
                       (long)tune_scale_float(C620_up_angle.Angle_pid.ki, 1000.0f),
                       (long)tune_scale_float(C620_up_angle.Angle_pid.kd, 1000.0f),
                       (long)tune_scale_float(C620_up_angle.Speed_pid.kp, 1000.0f),
                       (long)tune_scale_float(C620_up_angle.Speed_pid.ki, 1000.0f),
                       (long)tune_scale_float(C620_up_angle.Speed_pid.kd, 1000.0f),
                       (long)C620_up_angle.Angle_pid.maxout,
                       (long)C620_up_angle.Speed_pid.maxout);
    if (len > 0)
    {
        tune_send_text(tx);
    }
}

static void tune_process_command(char *line)
{
    char *cursor = line;
    char *cmd = tune_next_token(&cursor);
    float f[8];
    int32_t i0;

    if (cmd == NULL)
    {
        return;
    }

    if (strcmp(cmd, "PID") == 0)
    {
        for (uint8_t i = 0U; i < 6U; i++)
        {
            if (!tune_parse_float(&cursor, &f[i]))
            {
                tune_send_text("ERR PID needs: PID akp aki akd skp ski skd [amax smax]\r\n");
                return;
            }
        }
        pid_reset(&C620_up_angle.Angle_pid, f[0], f[1], f[2]);
        pid_reset(&C620_up_angle.Speed_pid, f[3], f[4], f[5]);
        if (tune_parse_float(&cursor, &f[6]) && tune_parse_float(&cursor, &f[7]))
        {
            C620_up_angle.Angle_pid.maxout = (int32_t)f[6];
            C620_up_angle.Speed_pid.maxout = (int32_t)f[7];
        }
        tune_send_status("OKP");
    }
    else if (strcmp(cmd, "LIMIT") == 0)
    {
        if (!tune_parse_float(&cursor, &f[0]) || !tune_parse_float(&cursor, &f[1]))
        {
            tune_send_text("ERR LIMIT needs: LIMIT amax smax\r\n");
            return;
        }
        C620_up_angle.Angle_pid.maxout = (int32_t)f[0];
        C620_up_angle.Speed_pid.maxout = (int32_t)f[1];
        tune_send_status("OKL");
    }
    else if (strcmp(cmd, "TARGET") == 0)
    {
        if (!tune_parse_float(&cursor, &f[0]))
        {
            tune_send_text("ERR TARGET needs motor_deg\r\n");
            return;
        }
        tune_control_active = 1U;
        C620_up_angle.target_angle = f[0];
        tune_pid_clear_state(&C620_up_angle.Angle_pid);
        tune_pid_clear_state(&C620_up_angle.Speed_pid);
        tune_send_status("OKT");
    }
    else if (strcmp(cmd, "GOTO") == 0)
    {
        if (!tune_parse_float(&cursor, &f[0]))
        {
            tune_send_text("ERR GOTO needs output_deg\r\n");
            return;
        }
        tune_control_active = 1U;
        C620_up_angle.target_angle = f[0] * C620_UP_REDUCTION_RATIO;
        tune_pid_clear_state(&C620_up_angle.Angle_pid);
        tune_pid_clear_state(&C620_up_angle.Speed_pid);
        tune_send_status("OKG");
    }
    else if (strcmp(cmd, "ZERO") == 0)
    {
        tune_control_active = 1U;
        C620_up_angle.Zero = C620_up_angle.currentRead;
        C620_up_angle.lastRead = C620_up_angle.currentRead;
        C620_up_angle.totalAngle = 0.0f;
        C620_up_angle.Angle_pid.get = 0.0f;
        C620_up_angle.target_angle = 0.0f;
        tune_pid_clear_state(&C620_up_angle.Angle_pid);
        tune_pid_clear_state(&C620_up_angle.Speed_pid);
        tune_send_status("OKZ");
    }
    else if (strcmp(cmd, "STOP") == 0)
    {
        tune_control_active = 1U;
        C620_up_angle.target_angle = C620_up_angle.Angle_pid.get;
        tune_pid_clear_state(&C620_up_angle.Angle_pid);
        tune_pid_clear_state(&C620_up_angle.Speed_pid);
        tune_send_status("OKS");
    }
    else if (strcmp(cmd, "LOG") == 0)
    {
        if (!tune_parse_int(&cursor, &i0))
        {
            tune_send_text("ERR LOG needs: LOG 0|1 [divider]\r\n");
            return;
        }
        tune_log_enabled = (i0 != 0) ? 1U : 0U;
        if (tune_parse_int(&cursor, &i0))
        {
            if (i0 < 1)
            {
                i0 = 1;
            }
            if (i0 > 100)
            {
                i0 = 100;
            }
            tune_report_divider = (uint8_t)i0;
        }
        tune_send_text(tune_log_enabled ? "OK LOG 1\r\n" : "OK LOG 0\r\n");
    }
    else if (strcmp(cmd, "STATUS") == 0)
    {
        tune_send_status("OKI");
    }
    else if (strcmp(cmd, "HELP") == 0)
    {
        tune_send_text("OK CMDS: PID akp aki akd skp ski skd [amax smax]; LIMIT amax smax; TARGET motor_deg; GOTO output_deg; ZERO; STOP; LOG 0|1 [divider]; STATUS\r\n");
    }
    else
    {
        tune_send_text("ERR unknown command\r\n");
    }
}

void DebugTune_Init(void)
{
    tune_rx_len = 0U;
    tune_cmd_ready = 0U;
    tune_log_enabled = 0U;
    tune_report_divider = 5U;
    tune_report_tick = 0U;
    tune_snapshot_pending = 0U;
    tune_ms = 0U;
    tune_send_text("OK DEBUG_TUNE USART6 115200\r\n");
}

void DebugTune_Task(void)
{
    char line[DEBUG_TUNE_LINE_MAX];
    debug_tune_snapshot_t snapshot;
    uint8_t has_command = 0U;
    uint8_t has_snapshot = 0U;

    tune_poll_uart_rx();

    __disable_irq();
    if (tune_cmd_ready)
    {
        memcpy(line, tune_cmd_line, sizeof(line));
        tune_cmd_ready = 0U;
        has_command = 1U;
    }
    if (tune_snapshot_pending)
    {
        snapshot = tune_snapshot;
        tune_snapshot_pending = 0U;
        has_snapshot = 1U;
    }
    __enable_irq();

    if (has_command)
    {
        tune_process_command(line);
    }

    if (has_snapshot)
    {
        char tx[DEBUG_TUNE_TX_MAX];
        int len = snprintf(tx, sizeof(tx),
                           "UPI,%lu,%ld,%ld,%ld,%ld,%d,%d,%ld,%ld\r\n",
                           (unsigned long)snapshot.ms,
                           (long)tune_scale_float(snapshot.target, 100.0f),
                           (long)tune_scale_float(snapshot.angle, 100.0f),
                           (long)tune_scale_float(snapshot.error, 100.0f),
                           (long)tune_scale_float(snapshot.speed, 1.0f),
                           (int)snapshot.torque,
                           (int)snapshot.out,
                           (long)snapshot.angle_maxout,
                           (long)snapshot.speed_maxout);
        if (len > 0)
        {
            tune_send_text(tx);
        }
    }
}

uint8_t DebugTune_IsActive(void)
{
    return tune_control_active;
}

void DebugTune_OnControlTick(int16_t control_out)
{
    tune_ms += 10U;
    if (!tune_log_enabled)
    {
        return;
    }

    tune_report_tick++;
    if (tune_report_tick < tune_report_divider)
    {
        return;
    }
    tune_report_tick = 0U;

    tune_snapshot.ms = tune_ms;
    tune_snapshot.target = C620_up_angle.target_angle;
    tune_snapshot.angle = C620_up_angle.Angle_pid.get;
    tune_snapshot.error = C620_up_angle.target_angle - C620_up_angle.Angle_pid.get;
    tune_snapshot.speed = C620_up_angle.Speed_pid.get;
    tune_snapshot.torque = C620_up_angle.Rxmsg.Torque;
    tune_snapshot.out = control_out;
    tune_snapshot.angle_kp = C620_up_angle.Angle_pid.kp;
    tune_snapshot.angle_ki = C620_up_angle.Angle_pid.ki;
    tune_snapshot.angle_kd = C620_up_angle.Angle_pid.kd;
    tune_snapshot.speed_kp = C620_up_angle.Speed_pid.kp;
    tune_snapshot.speed_ki = C620_up_angle.Speed_pid.ki;
    tune_snapshot.speed_kd = C620_up_angle.Speed_pid.kd;
    tune_snapshot.angle_maxout = C620_up_angle.Angle_pid.maxout;
    tune_snapshot.speed_maxout = C620_up_angle.Speed_pid.maxout;
    tune_snapshot_pending = 1U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}