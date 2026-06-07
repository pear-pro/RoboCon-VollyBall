#include "FSM.h"
#include "can.h"
#include "debug_uart.h"
#include <stm32f4xx.h>
#include "motor_can.h"
#include "pid.h"
#include <stdint.h>

//声明结构体对象
extern motor_info_t damiao[HIT_MOTOR_COUNT];
extern motor_info_t C620_up_angle;
extern motor_info_t C620_hit_angle[HIT_MOTOR_COUNT];
//发球状态机相关变量
static volatile uint8_t serve_active = 0;
static volatile uint8_t serve_armed = 1;
static volatile uint16_t serve_tick = 0;
static volatile serve_stage_t serve_stage = SERVE_STAGE_IDLE;
//击球状态机相关变量
static volatile hit_state_t hit_stage = HIT_IDLE;
static uint8_t hit_preset_index = 0;

serve_mode_t serve_mode = SERVE_MODE_ANGLE;

static const float hit_angle_table[HIT_MOTOR_COUNT][HIT_MOTOR_COUNT] =
{
    {40.0f, -40.0f, 40.0f},
    {25.0f, -25.0f, 25.0f},
};

static const float hit_angle_return[HIT_MOTOR_COUNT] =
{
    0,0,0
};

//限幅函数，限制输出在[-limit, limit]范围内
static int16_t limit(int32_t value, int16_t limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return (int16_t)value;
}

//判断所有击球电机是否都在目标角度附近
/*只要有一个还没回到阈值范围内，就返回 0；三个都接近 0，才返回 1，然后状态机进入 HIT_IDLE */
static uint8_t hit_all_near(float target, float threshold)
{
    for (uint8_t i = 0; i < HIT_MOTOR_COUNT; i++)
    {
        float err = damiao[i].Angle_pid.get - target;
        if (err < 0.0f)
        {
            err = -err;
        }
        if (err > threshold)
        {
            return 0;
        }
    }
    return 1;
}


void serve_arm(void)
{
    serve_armed = 1;
}

void serve_request_start(void)
{
    if (serve_armed && !serve_active)
    {
        serve_active = 1;
        serve_armed = 0;
        serve_tick = 0;
        serve_stage = SERVE_STAGE_LIFT;
    }
}

uint8_t serve_is_active(void)
{
    return serve_active;
}

//根据预设索引设置对应的目标角度
void hit_set_preset(uint8_t preset)
{
    if (preset >= HIT_MOTOR_COUNT)
    {
        preset = HIT_MOTOR_COUNT - 1U;
    }
    hit_preset_index = preset;
}

//外部调用，触发击球动作
void hit_request_press(void)
{
    hit_stage = HIT_PUT_ANGLE;
}

//外部调用，触发击球回零
void hit_request_release(void)
{
    hit_stage = HIT_RETURN;
}

//判断击球状态机是否处于活动状态
uint8_t hit_is_active(void)
{
    return hit_stage != HIT_IDLE;
}


void remote_control_serve_update(void)
{
    if (!serve_active)
    {
        return;
    }

    switch (serve_stage)
    {
    case SERVE_STAGE_LIFT:
        if (++serve_tick >= SERVE_LIFT_TICKS)
        {
            serve_stage = SERVE_STAGE_LIFT_RETURN;
            serve_tick = 0;
        }
        break;

    case SERVE_STAGE_LIFT_RETURN:
        if (++serve_tick >= SERVE_RETURN_TICKS)
        {
            serve_stage = SERVE_STAGE_HIT;
            serve_tick = 0;
        }
        break;

    case SERVE_STAGE_HIT:
        C620_up_angle.target_angle = 170.0f;
        if (++serve_tick >= SERVE_HIT_TICKS)
        {
            serve_stage = SERVE_STAGE_HIT_RETURN;
            serve_tick = 0;
        }
        break;

    case SERVE_STAGE_HIT_RETURN:
    default:
        C620_up_angle.target_angle = 0.0f;
        if (++serve_tick >= SERVE_HIT_RETURN_TICKS)
        {
            serve_stage = SERVE_STAGE_IDLE;
            serve_tick = 0;
            serve_active = 0;
        }
        break;
    }
}


void remote_control_hit_update(void)
{
    switch (hit_stage)
    {
    case HIT_PUT_ANGLE:
        for (uint8_t i = 0; i < HIT_MOTOR_COUNT; i++)
        {
            damiao[i].target_angle = hit_angle_table[hit_preset_index][i];
        }
        break;

    case HIT_RETURN:
        for (uint8_t i = 0; i < HIT_MOTOR_COUNT; i++)
        {
            damiao[i].target_angle = 0.0f;
        }
        if (hit_all_near(0.0f, HIT_RETURN_DONE_DEG))
        {
            hit_stage = HIT_IDLE;
        }
        break;

    case HIT_IDLE:
    default:
        break;
    }
}

void hit_angle_control(void)
{
    // int16_t voltage[HIT_MOTOR_COUNT] = {0};

    // for (uint8_t i = 0; i < HIT_MOTOR_COUNT; i++)
    // {
    //     int32_t output = PID_PROCESS_Double(&C620_hit_angle[i].Angle_pid,
    //                                         &C620_hit_angle[i].Speed_pid,
    //                                         C620_hit_angle[i].target_angle,
    //                                         C620_hit_angle[i].Angle_pid.get,
    //                                         C620_hit_angle[i].Speed_pid.get);
    //     voltage[i] = limit(output, HIT_OUTPUT_LIMIT);
    // }

    // Set_voltage_hit(&hcan2, voltage);
    Set_dm_mit(&hcan1, 0);
    Set_dm_mit(&hcan1, 1);
    Set_dm_mit(&hcan1, 2);
}

void up_angle_control(void)
{
    int16_t voltage[1] = {0};

    int32_t output;
    if(serve_mode == SERVE_MODE_ANGLE)
    {
        output = PID_PROCESS_Double(&C620_up_angle.Angle_pid,
                                        &C620_up_angle.Speed_pid,
                                        C620_up_angle.target_angle,
                                        C620_up_angle.Angle_pid.get,
                                        C620_up_angle.Speed_pid.get);
    }
    else
    {
        output =
            PID_PROCESS_Speed(
                &C620_up_angle.Speed_pid,
                C620_up_angle.target_speed,
                C620_up_angle.Speed_pid.get
            );
    }

    voltage[0] = limit(output, HIT_OUTPUT_LIMIT);

    Set_voltage_up_angle(&hcan2, voltage);
}