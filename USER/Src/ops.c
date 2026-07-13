#include "ops.h"
#include "can.h"
#include "debug_uart.h"
#include "motor_can.h"
#include "pid.h"
#include <stdint.h>
#include <string.h>

extern motor_info_t C620_up_angle[2];
extern motor_info_t C620_angle;
extern motor_info_t C620_hit_angle[3];
extern motor_info_t C620[MotorCount];
static void ops_set_up_output(int16_t output)
{
    int16_t cmd[2] = {output, output};
    Set_voltage_up_angle(&hcan2, cmd);
}

static void ops_set_pitch_output(int16_t output)
{
    int16_t cmd[1] = {output};
    Set_voltage_angle(&hcan2, cmd);
}

static void ops_set_hit_output(int16_t output)
{
    int16_t cmd[4] = {0};
    cmd[0] = output;
    cmd[1] = output;
    cmd[2] = output;
    cmd[3] = output;
    Set_voltage_angle(&hcan2, cmd);
}

static ops_control_t ops_targets[OPS_TARGET_MAX] =
{
    {
        .name = "up",
        .motor = &C620_up_angle[1],
        .mode = ops_mode_position,
        .ratio = 19.0f,
        .set_output = ops_set_up_output,
        .max_output = 5000,
    },
    {
        .name = "pitch_speed",
        .motor = &C620_angle,
        .mode = ops_mode_speed,
        .ratio = 19.0f,
        .set_output = ops_set_pitch_output,
        .max_output = 30000,
    },
    {
        .name = "hit",
        .motor = &C620_hit_angle[0],
        .mode = ops_mode_position,
        .ratio = 19.0f,
        .set_output = ops_set_hit_output,
        .max_output = 2000,
    }
};

static const uint8_t ops_target_count = 3U;
static uint8_t ops_now_index = 0U;  //索引，当前选中的是发球的

static void ops_limit_output(int16_t *output, int16_t limit)
{
    if (*output > limit)
    {
        *output = limit;
    }
    else if (*output < -limit)
    {
        *output = -limit;
    }
}

static void ops_pid_clear_state(pid_t *pid)
{
    pid->error[NOW_ERR] = 0.0f;
    pid->error[LAST_ERR] = 0.0f;
    pid->error[LLAST_ERR] = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->out = 0.0f;
}

ops_control_t *ops_get_current(void)
{
    return &ops_targets[ops_now_index];
}

uint8_t ops_get_current_index(void)
{
    return ops_now_index;
}

uint8_t ops_get_count(void)
{
    return ops_target_count;
}

ops_control_t *ops_get_target(uint8_t index)
{
    if (index >= ops_target_count)
    {
        return 0;
    }
    return &ops_targets[index];
}

uint8_t ops_select_by_index(uint8_t index)
{
    if (index >= ops_target_count)
    {
        return 0U;
    }
    ops_now_index = index;
    ops_reset_pid_state();
    return 1U;
}

uint8_t ops_select_by_name(const char *name)
{
    for (uint8_t i = 0U; i < ops_target_count; i++)
    {
        if (strcmp(name, ops_targets[i].name) == 0)
        {
            return ops_select_by_index(i);
        }
    }
    return 0U;
}

void ops_set_mode(ops_mode_t mode)
{
    ops_get_current()->mode = mode;
    ops_reset_pid_state();
}

const char *ops_mode_name(ops_mode_t mode)
{
    return (mode == ops_mode_position) ? "POSITION" : "SPEED";
}

void ops_reset_pid_state(void)
{
    motor_info_t *motor = ops_get_current()->motor;
    ops_pid_clear_state(&motor->Angle_pid);
    ops_pid_clear_state(&motor->Speed_pid);
}

void ops_set_pid(float akp, float aki, float akd, float skp, float ski, float skd)
{
    motor_info_t *motor = ops_get_current()->motor;
    pid_reset(&motor->Angle_pid, akp, aki, akd);
    pid_reset(&motor->Speed_pid, skp, ski, skd);
}

void ops_set_limit(int32_t angle_maxout, int32_t speed_maxout)
{
    motor_info_t *motor = ops_get_current()->motor;
    motor->Angle_pid.maxout = angle_maxout;
    motor->Speed_pid.maxout = speed_maxout;
}

void ops_set_target_motor(float motor_deg)
{
    motor_info_t *motor = ops_get_current()->motor;
    motor->target_angle = motor_deg;
    ops_reset_pid_state();
}

void ops_set_target_output(float output_deg)
{
    ops_control_t *target = ops_get_current();
    target->motor->target_angle = output_deg * target->ratio;
    ops_reset_pid_state();
}

void ops_set_speed(float rpm)
{
    motor_info_t *motor = ops_get_current()->motor;
    motor->Speed_pid.set = rpm;
    ops_pid_clear_state(&motor->Speed_pid);
}

void ops_zero(void)
{
    motor_info_t *motor = ops_get_current()->motor;
    motor->Zero = motor->currentRead;
    motor->lastRead = motor->currentRead;
    motor->totalAngle = 0.0f;
    motor->Angle_pid.get = 0.0f;
    motor->target_angle = 0.0f;
    motor->Speed_pid.set = 0.0f;
    ops_reset_pid_state();
}

void ops_stop(void)
{
    ops_control_t *target = ops_get_current();
    motor_info_t *motor = target->motor;
    if (target->mode == ops_mode_position)
    {
        motor->target_angle = motor->Angle_pid.get;
    }
    else
    {
        motor->Speed_pid.set = 0.0f;
    }
    ops_reset_pid_state();
    target->set_output(0);
}

void ops_control(void)
{
    ops_control_t *target = ops_get_current();
    motor_info_t *motor = target->motor;
    int16_t output = 0;

    if (target->mode == ops_mode_position)
    {
        output = PID_PROCESS_Double(&motor->Angle_pid,
                                    &motor->Speed_pid,
                                    motor->target_angle,
                                    motor->Angle_pid.get,
                                    motor->Speed_pid.get);
    }
    else
    {
        pid_calc(&motor->Speed_pid, motor->Speed_pid.get, motor->Speed_pid.set);
        output = (int16_t)motor->Speed_pid.out;
    }

    ops_limit_output(&output, target->max_output);
    target->set_output(output);
    DebugTune_OnControlTick(output);
}



