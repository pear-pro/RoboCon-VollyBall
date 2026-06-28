#include "watch_dog.h"
#include "FSM.h"
#include "motor_can.h"
#include "pid.h"
#include <stdint.h>

extern motor_info_t C620[MotorCount];
extern motor_info_t C620_angle;
extern motor_info_t C620_hit_angle[HIT_MOTOR_COUNT];
extern motor_info_t C620_up_angle;
extern motor_info_t damiao[MotorCount];

extern float car_x;
extern float car_y;
extern float car_w;
extern float car_tarx;
extern float car_tary;
extern float car_tarw;

static volatile uint16_t rc_watchdog_tick = 0U;
static volatile uint8_t rc_watchdog_timeout = 1U;

static void watch_dog_pid_clear(pid_t *pid)
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

void remote_control_watchdog_feed(void)
{
    rc_watchdog_tick = 0U;
    rc_watchdog_timeout = 0U;
}

void remote_control_watchdog_update(void)
{
    if (rc_watchdog_tick < RC_WATCHDOG_TIMEOUT_TICKS)
    {
        rc_watchdog_tick++;
    }

    if (rc_watchdog_tick >= RC_WATCHDOG_TIMEOUT_TICKS)
    {
        rc_watchdog_timeout = 1U;
    }
}

uint8_t remote_control_is_timeout(void)
{
    return rc_watchdog_timeout;
}

void remote_control_enter_safe_state(void)
{
    car_x = 0.0f;
    car_y = 0.0f;
    car_w = 0.0f;
    car_tarx = 0.0f;
    car_tary = 0.0f;
    car_tarw = 0.0f;

    for (uint8_t i = 0U; i < MotorCount; i++)
    {
        C620[i].Speed_pid.set = 0.0f;
        watch_dog_pid_clear(&C620[i].Speed_pid);
    }

    C620_angle.Speed_pid.set = 0.0f;
    watch_dog_pid_clear(&C620_angle.Speed_pid);

    C620_up_angle.target_speed = 0.0f;
    C620_up_angle.target_angle = C620_up_angle.Angle_pid.get;
    watch_dog_pid_clear(&C620_up_angle.Speed_pid);
    watch_dog_pid_clear(&C620_up_angle.Angle_pid);

    C620_hit_angle[0].target_angle = -10.0f * SCALE;
    C620_hit_angle[1].target_angle = -10.0f * SCALE;
    C620_hit_angle[2].target_angle =  10.0f * SCALE;
    for (uint8_t i = 0U; i < HIT_MOTOR_COUNT; i++)
    {
        watch_dog_pid_clear(&C620_hit_angle[i].Speed_pid);
        watch_dog_pid_clear(&C620_hit_angle[i].Angle_pid);
    }

    damiao[0].angle = 0.0f;
    damiao[1].angle = 0.0f;
}