#include "FSM.h"
#include "can.h"
#include "debug_uart.h"
#include <stm32f4xx.h>
#include "motor_can.h"
#include "pid.h"
#include <stdint.h>
#include "t14.h"
#include "jumper_t15_rc.h"

//�����ṹ�����
extern motor_info_t damiao[HIT_MOTOR_COUNT];
extern motor_info_t C620_up_angle;
extern motor_info_t C620_hit_angle[HIT_MOTOR_COUNT];
//����״̬����ر���
static volatile uint8_t serve_active = 0;
static volatile uint8_t serve_armed = 1;
static volatile uint16_t serve_tick = 0;
static volatile serve_stage_t serve_stage = SERVE_STAGE_IDLE;
//����״̬����ر���
static volatile hit_state_t hit_stage = HIT_IDLE;
static uint8_t hit_preset_index = 0;
double set_angle = -170.0f;
volatile uint16_t count = 0;
serve_mode_t serve_mode = SERVE_MODE_ANGLE;

static const float hit_angle_table[HIT_MOTOR_COUNT][HIT_MOTOR_COUNT] =
{
    {-1.25f, 1.25f, 1.25f},
    {-1.0f, 1.0f, 1.0f},
};

static const float hit_angle_reset[HIT_MOTOR_COUNT] =
{
    0,0,0
};

//�޷����������������[-limit, limit]��Χ��
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

//�ж����л������Ƿ���Ŀ��Ƕȸ���
/*ֻҪ��һ����û�ص���ֵ��Χ�ڣ��ͷ��� 0���������ӽ� 0���ŷ��� 1��Ȼ��״̬������ HIT_IDLE */
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

//����Ԥ���������ö�Ӧ��Ŀ��Ƕ�
void hit_set_preset(uint8_t preset)
{
    if (preset >= HIT_MOTOR_COUNT)
    {
        preset = HIT_MOTOR_COUNT - 1U;
    }
    hit_preset_index = preset;
}

//�ⲿ���ã�����������
void hit_request_press(void)
{
    hit_stage = HIT_PUT_ANGLE;
}

//�ⲿ���ã������������
void hit_request_release(void)
{
    hit_stage = HIT_RETURN;
}

//�жϻ���״̬���Ƿ��ڻ״̬
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
        C620_up_angle.target_angle = (set_angle + 360 * count) * 19;
		Pump_On();
        if (++serve_tick >= SERVE_HIT_TICKS)
        {
            serve_stage = SERVE_STAGE_HIT_RETURN;
            serve_tick = 0;
        }
        break;

    case SERVE_STAGE_HIT_RETURN:
    default:
        //C620_up_angle.target_angle = ;
	    Pump_Off();
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
            damiao[i].angle = hit_angle_table[hit_preset_index][i];
        }
        break;

    case HIT_RETURN:
    {
        uint8_t all_done = 1;
        for (uint8_t i = 0; i < HIT_MOTOR_COUNT; i++)
        {
            if (!move_to_angle_smooth(&damiao[i], hit_angle_reset[i], HIT_RETURN_STEP))
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