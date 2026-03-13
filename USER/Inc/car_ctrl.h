#ifndef __CAR_CTRL_H
#define __CAR_CTRL_H

#include "pid.h"
extern pid_t car_pid;

void MecanumWheel_Move(float vx, float vy, float wz);
#endif