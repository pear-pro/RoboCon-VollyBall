#include "FSM.h"
#include "can.h"
#include "debug_uart.h"
#include "motor_can.h"
#include "pid.h"
#include <stdint.h>

//电机结构体变量
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

static const float hit_angle_table[HIT_MOTOR_COUNT][HIT_MOTOR_COUNT] =
{
    {-65.0f  * SCALE,-65.0f  * SCALE, 65.0f  * SCALE},
    {-25.0f * SCALE, -25.0f * SCALE, 25.0f * SCALE},
    {-15.0f * SCALE, -15.0f * SCALE, 15.0f * SCALE},
};

static const float hit_angle_reset[HIT_MOTOR_COUNT] = {-10.0f * SCALE,-10.0f * SCALE,10.0f * SCALE}; 

//限幅函数：将 value 限制在 [-limit, limit] 范围内
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

// 将 target_angle 逐步逼近目标角度 (每次挪动 step_size)
// 目标缓慢变化，PID 自然跟进，回落轨迹更平滑
// 返回值: 1 = 已到达目标, 0 = 正在调节
static uint8_t move_to_angle_smooth(motor_info_t *motor, float target, float step_size)
{
    float diff = target - motor->target_angle;

    if (diff > step_size)
    {
        motor->target_angle += step_size;
        return 0;
    }
    else if (diff < -step_size)
    {
        motor->target_angle -= step_size;
        return 0;
    }
    else
    {
        motor->target_angle = target;
        return 1;
    }
}

//判断三个电机是否在目标角度附近
/*只要有一个没回到数值范围内，就返回 0；全部接近 0 才返回 1，然后状态机进入 HIT_IDLE */
static uint8_t hit_all_near(float target, float threshold)
{
    for (uint8_t i = 0; i < HIT_MOTOR_COUNT; i++)
    {
        float err = C620_hit_angle[i].Angle_pid.get - target;
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

//设置预设角度：设置对应的目标角度
void hit_set_preset(uint8_t preset)
{
    if (preset >= HIT_MOTOR_COUNT)
    {
        preset = HIT_MOTOR_COUNT - 1U;
    }
    hit_preset_index = preset;
}

//外部调用：击球按下
void hit_request_press(void)
{
    hit_stage = HIT_PUT_ANGLE;
}

//外部调用：击球复位释放
void hit_request_release(void)
{
    hit_stage = HIT_RETURN;
}

//判断击球状态是否在活动状态
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
        C620_up_angle.target_angle = 170.0f * SCALE;
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
            C620_hit_angle[i].target_angle = hit_angle_table[hit_preset_index][i];
        }
        break;

    case HIT_RETURN:
    {
        uint8_t all_done = 1;
        for (uint8_t i = 0; i < HIT_MOTOR_COUNT; i++)
        {
            if (!move_to_angle_smooth(&C620_hit_angle[i], hit_angle_reset[i], HIT_RETURN_STEP))
            {
                all_done = 0;
            }
        }
        if (all_done)
        {
            hit_stage = HIT_IDLE;
        }
        break;
    }

    case HIT_IDLE:
    default:
        break;
    }
}

void hit_angle_control(void)    //��PID��ʱ���б� remote_control_hit_update ���ã��������ƻ������Ƕȣ�ֱ���������
{
    int16_t voltage[HIT_MOTOR_COUNT] = {0};

    for (uint8_t i = 0; i < HIT_MOTOR_COUNT; i++)
    {
        int32_t output = PID_PROCESS_Double(&C620_hit_angle[i].Angle_pid,
                                            &C620_hit_angle[i].Speed_pid,
                                            C620_hit_angle[i].target_angle,
                                            C620_hit_angle[i].Angle_pid.get,
                                            C620_hit_angle[i].Speed_pid.get);
        voltage[i] = limit(output, HIT_OUTPUT_LIMIT);
    }

    Set_voltage_hit(&hcan1, voltage);
}