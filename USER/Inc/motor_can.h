#ifndef __MOTOR_CAN_H
#define __MOTOR_CAN_H

#include "main.h"
#include "can.h"
#include "pid.h"
/*



*/
#define NORMALIZE_ANGLE180(angle) angle = ((angle) > 180) ? ((angle) - 360) : (((angle) < -180) ? (angle) + 360 : angle)
#define MotorCount 4
extern motor_info_t C620[MotorCount];
extern motor_info_t C6xx[MotorCount];
//角度归一化
typedef struct{
	int16_t Voltage;//电压值
	uint16_t Angle;//机械角度
	int16_t Speed;//转速
	int16_t Torque;//实际扭矩
	uint8_t Temp;//温度
	
}RxMsg_t;

typedef struct{
	pid_t 				Speed_pid;
	pid_t 				Angel_pid;
	
	//控制角度的参数
	uint16_t			FirstEntre;
	uint16_t			Target;//目标角度
	uint16_t 				lastRead;
	uint16_t 				relative;//相对零点转了多少度
	uint16_t 			Zero;//上电后的第一个位置做为零点
	float				encoderAngle;//经过处理的电机角度
	uint16_t			Current;//输出电流
	
	RxMsg_t 			Rxmsg;
}motor_info_t;



void can_filter_init(void);
<<<<<<< HEAD
=======
void can1_filter_init(void);
void can2_fliter_init(void);


void Set_voltagec1(CAN_HandleTypeDef* hcan,int16_t vlotage[]);
void Set_voltagec2(CAN_HandleTypeDef* hcan,int16_t vlotage[]);
#endif

