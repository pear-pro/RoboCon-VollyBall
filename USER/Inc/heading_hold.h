#ifndef __HEADING_HOLD_H
#define __HEADING_HOLD_H

#include <stdint.h>

typedef struct {
    float target_yaw_deg;
    float current_yaw_deg;
    float error_deg;
    float output_wz;
    uint8_t enabled;
    uint8_t initialized;
} HeadingHold_t;

extern HeadingHold_t heading_hold;

void HeadingHold_Init(void);
void HeadingHold_Enable(uint8_t enable);
void HeadingHold_ResetTargetToCurrent(void);
void HeadingHold_AddTargetDeg(float delta_deg);
void HeadingHold_Task(void);
float HeadingHold_Update(float manual_wz);

#endif
