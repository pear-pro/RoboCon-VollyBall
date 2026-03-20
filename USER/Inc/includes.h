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
#include "bsp_usart.h"
#include "Initialize.h"
#include "rc_task.h"
#include "math_utils.h"
#include "jy901p.h"
#include "JY901P_Calibrate.h"
extern motor_info_t C620[MotorCount];
extern motor_info_t C6xx[MotorCount];
extern motor_info_t damiao[MotorCount];
extern uint16_t PID_Calc_Flag;
extern int16_t car_x,car_y,car_w;

extern JY901P_DataStruct gyro_data;

enum State{
    faqiuzhunbei=0,
    faqiu=1,
    jiqiu=2,
};

#define MAX_CAR_SPEED 1000.0f

#endif
