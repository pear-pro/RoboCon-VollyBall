#ifndef __OPS_H__
#define __OPS_H__

#include "main.h"
#include "motor_can.h"
#include <stdint.h>

typedef enum
{
    ops_mode_speed = 0,
    ops_mode_position = 1,
} ops_mode_t;

typedef struct
{
    const char *name;
    motor_info_t *motor;
    ops_mode_t mode;
    float ratio;
    void (*set_output)(int16_t output);
    int16_t max_output;
} ops_control_t;

#define OPS_TARGET_MAX 8

void ops_control(void);
ops_control_t *ops_get_current(void);
uint8_t ops_get_current_index(void);
uint8_t ops_get_count(void);
ops_control_t *ops_get_target(uint8_t index);
uint8_t ops_select_by_index(uint8_t index);
uint8_t ops_select_by_name(const char *name);
void ops_set_mode(ops_mode_t mode);
const char *ops_mode_name(ops_mode_t mode);
void ops_reset_pid_state(void);
void ops_set_pid(float akp, float aki, float akd, float skp, float ski, float skd);
void ops_set_limit(int32_t angle_maxout, int32_t speed_maxout);
void ops_set_target_motor(float motor_deg);
void ops_set_target_output(float output_deg);
void ops_set_speed(float rpm);
void ops_zero(void);
void ops_stop(void);

#endif