#ifndef __FSM_H__
#define __FSM_H__

#include <stdint.h>

#define SCALE 19.0f
#define HIT_MOTOR_COUNT 3U
#define HIT_OUTPUT_LIMIT 20000
#define HIT_RETURN_DONE_DEG 20.0f //20/19 ????1.05??????????
#define HIT_RETURN_STEP 0.01f

#define SERVE_LIFT_TICKS        (85U)  //100U //15
#define SERVE_RETURN_TICKS      (9U)
#define SERVE_HIT_TICKS         (100U)
#define SERVE_HIT_RETURN_TICKS  (20U)
#define SERVE_STAGE_CLEAR_TICKS (500U)

typedef enum
{
    HIT_IDLE = 0,
    HIT_PUT_ANGLE,
    HIT_RETURN,
} hit_state_t;

typedef enum
{
    SERVE_STAGE_IDLE = 0,
    SERVE_STAGE_LIFT,
    SERVE_STAGE_LIFT_RETURN,
    SERVE_STAGE_HIT,
    SERVE_STAGE_HIT_RETURN,
	SERVE_STAGE_CLEAR,
} serve_stage_t;


typedef enum
{
    SERVE_MODE_ANGLE = 0,
    SERVE_MODE_SPEED = 1

}serve_mode_t;

extern serve_mode_t serve_mode;

void serve_arm(void);
void serve_request_start(void);
uint8_t serve_is_active(void);

void hit_set_preset(uint8_t preset);
void hit_request_press(void);
void hit_request_release(void);
uint8_t hit_is_active(void);

void remote_control_hit_update(void);
void remote_control_serve_update(void);
void hit_angle_control(void);
void up_angle_control(void);
extern volatile uint16_t count;

#endif
