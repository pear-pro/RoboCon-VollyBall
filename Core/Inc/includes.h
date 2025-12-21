#ifndef __INCLUDES_H
#define __INCLUDES_H


#include <stdint.h>
#include <math.h>
#include "pid.h"


#include "main.h" 
#include "motor_can.h"
#include "pid_tim.h"
#include "car_ctrl.h"
#include "bsp_rc.h"
#include "Initialize.h"

extern motor_info_t C620[MotorCount];
extern motor_info_t C6xx[MotorCount];
extern uint16_t PID_Calc_Flag;
#endif