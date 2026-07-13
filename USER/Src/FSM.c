#include "FSM.h"
#include "can.h"
#include "debug_uart.h"
#include <math.h>
#include <stm32f4xx.h>
#include "heading_hold.h"
#include "motor_can.h"
#include "pid.h"
#include <stdint.h>
#include "t14.h"
#include "jumper_t15_rc.h"

//�����ṹ�����?
extern motor_info_t damiao[HIT_MOTOR_COUNT];
extern motor_info_t C620_up_angle[2];
extern motor_info_t C620_hit_angle[HIT_MOTOR_COUNT];
//����״̬����ر���?
static volatile uint8_t serve_active = 0;
static volatile uint8_t serve_armed = 1;
static volatile uint16_t serve_tick = 0;
static volatile serve_stage_t serve_stage = SERVE_STAGE_IDLE;
//����״̬����ر���?
static volatile hit_state_t hit_stage = HIT_IDLE;
static uint8_t hit_preset_index = 0;
double set_angle = -170.0f;
volatile uint16_t count = 0;
volatile uint16_t count_up = 0;
static volatile float serve_lift_target_angle = 0.0f;
static volatile float serve_hit_target_angle = 0.0f;
static volatile uint8_t serve_up_output_enabled = 1U;

serve_mode_t serve_mode = SERVE_MODE_ANGLE;

#define UP_ANGLE_FEEDBACK_INDEX 1

#define SERVE_BASE_ANGLE        (-3230.0f)
#define SERVE_ANGLE_STEP        (10.0f * SCALE)
#define SERVE_HIT_ANGLE_DELTA   (360.0f * SCALE)

static void pid_clear_output(pid_t *pid)
{
    pid->set = 0.0f;
    pid->error[NOW_ERR] = 0.0f;
    pid->error[LAST_ERR] = 0.0f;
    pid->error[LLAST_ERR] = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->out = 0.0f;
}

static void serve_up_stop_force(void)
{
    int16_t voltage[2] = {0, 0};
    motor_info_t *motor = &C620_up_angle[UP_ANGLE_FEEDBACK_INDEX];

    serve_up_output_enabled = 0;
    motor->target_speed = 0.0f;
    motor->target_angle = motor->Angle_pid.get;
    pid_clear_output(&motor->Angle_pid);
    pid_clear_output(&motor->Speed_pid);
    Set_voltage_up_angle(&hcan2, voltage);
}

static const float hit_angle_table[HIT_MOTOR_COUNT][HIT_MOTOR_COUNT] =
{
    {-1.25f, 1.25f, 1.25f},
    {-1.0f, 1.0f, 1.0f},
};

static const float hit_angle_reset[HIT_MOTOR_COUNT] =
{
    0,0,0
};

static const float up_per_angle[8]=
{
	-220*SCALE,-220*SCALE,-220*SCALE,-220*SCALE,
	-220*SCALE,-220*SCALE,-220*SCALE,-220*SCALE
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
    float diff = target - motor->angle;

    if (diff > step_size)
    {	
        motor->angle += step_size;
        return 0;
    }
    else if (diff < -step_size)
    {
        motor->angle -= step_size;
        return 0;
    }
    else
    {
        motor->angle = target;
        return 1;
    }
}

//�ж����л������Ƿ���Ŀ��Ƕȸ���?
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
        serve_lift_target_angle = up_per_angle[count_up%8];//SERVE_BASE_ANGLE; //- (float)count * SERVE_ANGLE_STEP;
        serve_hit_target_angle = serve_lift_target_angle + SERVE_HIT_ANGLE_DELTA;
        count++;
		count_up++;
        serve_active = 1;
        serve_armed = 0;
        serve_tick = 0;
        serve_up_output_enabled = 1;
        pid_clear_output(&C620_up_angle[UP_ANGLE_FEEDBACK_INDEX].Angle_pid);
        pid_clear_output(&C620_up_angle[UP_ANGLE_FEEDBACK_INDEX].Speed_pid);
        serve_stage = SERVE_STAGE_LIFT;
    }
}

uint8_t serve_is_active(void)
{
    return serve_active;
}

//����Ԥ���������ö�Ӧ��Ŀ��Ƕ�?
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

//�ⲿ���ã������������?
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
    {
        float t = (float)serve_tick / (float)SERVE_LIFT_TICKS;
        float s = 0.5f - 0.5f * cosf(t * 3.14f);
        C620_up_angle[UP_ANGLE_FEEDBACK_INDEX].target_angle = serve_lift_target_angle * s;
        if (++serve_tick >= SERVE_LIFT_TICKS)
        {
            serve_stage = SERVE_STAGE_LIFT_RETURN;
            serve_tick = 0;
        }
        break;
    }

    case SERVE_STAGE_LIFT_RETURN:
		Pump_On();
		if (++serve_tick >= SERVE_RETURN_TICKS)
        {
            serve_stage = SERVE_STAGE_HIT;
            serve_tick = 0;
        }
        break;

    case SERVE_STAGE_HIT:
        C620_up_angle[UP_ANGLE_FEEDBACK_INDEX].target_angle = serve_hit_target_angle;
        if (++serve_tick >= SERVE_HIT_TICKS)
        {
            serve_stage = SERVE_STAGE_HIT_RETURN;
            serve_tick = 0;
        }
        break;

    case SERVE_STAGE_HIT_RETURN:
	    Pump_Off();
        if (++serve_tick >= SERVE_HIT_RETURN_TICKS)
        {
            serve_up_stop_force();
            serve_stage = SERVE_STAGE_CLEAR;
        }
        break;
	case SERVE_STAGE_CLEAR:
	default:
    HeadingHold_AddTargetDeg(180.0f/(float)SERVE_STAGE_CLEAR_TICKS);
        if (++serve_tick >= SERVE_STAGE_CLEAR_TICKS)
        {
            serve_stage = SERVE_STAGE_IDLE;
            serve_tick = 0;
            serve_active = 0;
			C620_up_angle[UP_ANGLE_FEEDBACK_INDEX].FirstEntre = 0;
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
			uint8_t all_done=1;
        for (uint8_t i = 0; i < HIT_MOTOR_COUNT; i++)
        {
            if (!(move_to_angle_smooth(&damiao[i], hit_angle_reset[i], HIT_RETURN_STEP)))
            {
                all_done=0;
            }
        }
				if(all_done)
				{
					hit_stage=HIT_IDLE;
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
    Set_dm_mit(&hcan1, 0);
    Set_dm_mit(&hcan1, 1);
    Set_dm_mit(&hcan1, 2);
}

void up_angle_control(void)
{
    int16_t voltage[2] = {0}; //һ�η������Ŀ��ƣ���ֻ��һ������
    int32_t output = 0;
    motor_info_t *motor = &C620_up_angle[UP_ANGLE_FEEDBACK_INDEX];

    if(serve_up_output_enabled == 0)
    {
        Set_voltage_up_angle(&hcan2, voltage);
        return;
    }

    if(serve_mode == SERVE_MODE_ANGLE)
    {
        output = PID_PROCESS_Double(&motor->Angle_pid,
                                    &motor->Speed_pid,
                                    motor->target_angle,
                                    motor->Angle_pid.get,
                                    motor->Speed_pid.get);
    }
    else
    {
        output = PID_PROCESS_Speed(&motor->Speed_pid,
                                   motor->target_speed,
                                   motor->Speed_pid.get);
    }

    voltage[0] = limit(output, HIT_OUTPUT_LIMIT);
    voltage[1] = voltage[0];
    Set_voltage_up_angle(&hcan2, voltage);
}
