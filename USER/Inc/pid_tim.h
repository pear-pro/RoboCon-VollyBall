#ifndef __PID_TIM_H__
#define __PID_TIM_H__
#include "main.h"

typedef struct {
    int16_t last_encoder;
    float total_angle;
    int32_t circle_cnt;
} EncoderCircleTypeDef;


#endif
