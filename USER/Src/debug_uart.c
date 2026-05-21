#include "debug_uart.h"
#include "ops.h"
#include "usart.h"
#include <stdlib.h>

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
    int32_t angle_maxout;
    int32_t speed_maxout;
} debug_tune_snapshot_t;

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
static uint8_t tune_rx_dma_byte;

void Vofa_JustFloat(float *_data, uint8_t _num)
{
    uint8_t tempData[100];
    uint8_t temp_end[4] = {0, 0, 0x80, 0x7F};
    float temp_copy[_num];

    memcpy(&temp_copy, _data, sizeof(float) * _num);
    memcpy(tempData, (uint8_t *)&temp_copy, sizeof(temp_copy));
    memcpy(&tempData[_num * 4], &temp_end[0], 4);
    HAL_UART_Transmit_DMA(&huart6, tempData, (_num + 1U) * 4U);
}

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

static void tune_start_rx_dma(void)
{
    if (huart6.RxState == HAL_UART_STATE_READY)
    {
        (void)HAL_UART_Receive_DMA(&huart6, &tune_rx_dma_byte, 1U);
    }
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
        return 0;
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
    char *end = 0;
    if (token == 0)
    {
        return 0U;
    }
    *out = strtof(token, &end);
    return (end != token) ? 1U : 0U;
}

static uint8_t tune_parse_int(char **cursor, int32_t *out)
{
    char *token = tune_next_token(cursor);
    char *end = 0;
    if (token == 0)
    {
        return 0U;
    }
    *out = strtol(token, &end, 10);
    return (end != token) ? 1U : 0U;
}

static uint8_t tune_parse_mode(const char *text, ops_mode_t *mode)
{
    if (strcmp(text, "SPEED") == 0 || strcmp(text, "speed") == 0)
    {
        *mode = ops_mode_speed;
        return 1U;
    }
    if (strcmp(text, "POSITION") == 0 || strcmp(text, "position") == 0 ||
        strcmp(text, "ANGLE") == 0 || strcmp(text, "angle") == 0)
    {
        *mode = ops_mode_position;
        return 1U;
    }
    return 0U;
}

static void tune_send_status(const char *prefix)
{
    ops_control_t *target = ops_get_current();
    motor_info_t *motor = target->motor;
    char tx[DEBUG_TUNE_TX_MAX];
    int len = snprintf(tx, sizeof(tx),
                       "%s,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%s,%s\r\n",
                       prefix,
                       (long)tune_scale_float(motor->target_angle, 100.0f),
                       (long)tune_scale_float(motor->Angle_pid.get, 100.0f),
                       (long)tune_scale_float(motor->target_angle - motor->Angle_pid.get, 100.0f),
                       (long)tune_scale_float(motor->Speed_pid.get, 1.0f),
                       (long)motor->Rxmsg.Torque,
                       (long)tune_scale_float(motor->Angle_pid.kp, 1000.0f),
                       (long)tune_scale_float(motor->Angle_pid.ki, 1000.0f),
                       (long)tune_scale_float(motor->Angle_pid.kd, 1000.0f),
                       (long)tune_scale_float(motor->Speed_pid.kp, 1000.0f),
                       (long)tune_scale_float(motor->Speed_pid.ki, 1000.0f),
                       (long)tune_scale_float(motor->Speed_pid.kd, 1000.0f),
                       (long)motor->Angle_pid.maxout,
                       (long)motor->Speed_pid.maxout,
                       target->name,
                       ops_mode_name(target->mode));
    if (len > 0)
    {
        tune_send_text(tx);
    }
}

static void tune_send_list(void)
{
    char tx[DEBUG_TUNE_TX_MAX];
    for (uint8_t i = 0U; i < ops_get_count(); i++)
    {
        ops_control_t *target = ops_get_target(i);
        if (target == 0)
        {
            continue;
        }
        int len = snprintf(tx, sizeof(tx), "OKLIST,%u,%s,%s,%ld\r\n",
                           (unsigned int)i,
                           target->name,
                           ops_mode_name(target->mode),
                           (long)tune_scale_float(target->ratio, 1000.0f));
        if (len > 0)
        {
            tune_send_text(tx);
        }
    }
}

static void tune_process_command(char *line)
{
    char *cursor = line;
    char *cmd = tune_next_token(&cursor);
    float f[8];
    int32_t i0;

    if (cmd == 0)
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
        ops_set_pid(f[0], f[1], f[2], f[3], f[4], f[5]);
        if (tune_parse_float(&cursor, &f[6]) && tune_parse_float(&cursor, &f[7]))
        {
            ops_set_limit((int32_t)f[6], (int32_t)f[7]);
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
        ops_set_limit((int32_t)f[0], (int32_t)f[1]);
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
        ops_set_target_motor(f[0]);
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
        ops_set_target_output(f[0]);
        tune_send_status("OKG");
    }
    else if (strcmp(cmd, "SPEED") == 0)
    {
        if (!tune_parse_float(&cursor, &f[0]))
        {
            tune_send_text("ERR SPEED needs rpm\r\n");
            return;
        }
        tune_control_active = 1U;
        ops_set_speed(f[0]);
        tune_send_status("OKV");
    }
    else if (strcmp(cmd, "ZERO") == 0)
    {
        tune_control_active = 1U;
        ops_zero();
        tune_send_status("OKZ");
    }
    else if (strcmp(cmd, "STOP") == 0)
    {
        tune_control_active = 1U;
        ops_stop();
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
    else if (strcmp(cmd, "LIST") == 0)
    {
        tune_send_list();
    }
    else if (strcmp(cmd, "SELECT") == 0)
    {
        char *arg = tune_next_token(&cursor);
        if (arg == 0)
        {
            tune_send_text("ERR SELECT needs name_or_index\r\n");
            return;
        }
        char *end = 0;
        long index = strtol(arg, &end, 10);
        uint8_t ok = (end != arg && *end == '\0') ? ops_select_by_index((uint8_t)index) : ops_select_by_name(arg);
        if (!ok)
        {
            tune_send_text("ERR SELECT unknown target\r\n");
            return;
        }
        tune_control_active = 1U;
        tune_send_status("OKSEL");
    }
    else if (strcmp(cmd, "MODE") == 0)
    {
        char *arg = tune_next_token(&cursor);
        ops_mode_t mode;
        if (arg == 0 || !tune_parse_mode(arg, &mode))
        {
            tune_send_text("ERR MODE needs SPEED|POSITION\r\n");
            return;
        }
        tune_control_active = 1U;
        ops_set_mode(mode);
        tune_send_status("OKMODE");
    }
    else if (strcmp(cmd, "HELP") == 0)
    {
        tune_send_text("OK CMDS: LIST; SELECT name|idx; MODE SPEED|POSITION; PID akp aki akd skp ski skd [amax smax]; LIMIT amax smax; TARGET motor_deg; GOTO output_deg; SPEED rpm; ZERO; STOP; LOG 0|1 [divider]; STATUS\r\n");
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
    tune_start_rx_dma();
    tune_send_text("OK DEBUG_TUNE USART6 115200\r\n");
}

void DebugTune_Task(void)
{
    char line[DEBUG_TUNE_LINE_MAX];
    debug_tune_snapshot_t snapshot;
    uint8_t has_command = 0U;
    uint8_t has_snapshot = 0U;

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
        has_snapshot = 0U;
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
    ops_control_t *target = ops_get_current();
    motor_info_t *motor = target->motor;

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
    tune_snapshot.target = (target->mode == ops_mode_position) ? motor->target_angle : motor->Speed_pid.set;
    tune_snapshot.angle = (target->mode == ops_mode_position) ? motor->Angle_pid.get : motor->Speed_pid.get;
    tune_snapshot.error = tune_snapshot.target - tune_snapshot.angle;
    tune_snapshot.speed = motor->Speed_pid.get;
    tune_snapshot.torque = motor->Rxmsg.Torque;
    tune_snapshot.out = control_out;
    tune_snapshot.angle_maxout = motor->Angle_pid.maxout;
    tune_snapshot.speed_maxout = motor->Speed_pid.maxout;
    tune_snapshot_pending = 1U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        tune_rx_feed_byte(tune_rx_dma_byte);
        tune_start_rx_dma();
    }
}
