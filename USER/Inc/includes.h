#ifndef __INCLUDES_H
#define __INCLUDES_H


#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "pid.h"

#include "tim.h"
#include "main.h" 
#include "motor_can.h"
#include "pid_tim.h"
#include "car_ctrl.h"
#include "bsp_rc.h"
#include "Initialize.h"
#include "remote_control.h"
#include "math_utils.h"

extern motor_info_t C620[MotorCount];
extern motor_info_t C6xx[MotorCount];
extern motor_info_t damiao[MotorCount];
extern uint16_t PID_Calc_Flag;
extern int16_t car_x,car_y,car_w;

#define MAX_CAR_SPEED 1000.0f

#endif