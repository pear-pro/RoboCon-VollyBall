#ifndef __MOTOR_CAN_H
#define __MOTOR_CAN_H

#include "main.h"
#include "can.h"
#include "pid.h"
#include "stm32f4xx.h"
#include <stdint.h>
/*



*/
#define NORMALIZE_ANGLE180(angle) angle = ((angle) > 180) ? ((angle) - 360) : (((angle) < -180) ? (angle) + 360 : angle)
#define MotorCount 4
#define C620_UP_REDUCTION_RATIO 19.0f
#define DM_MIT_ID_BASE 0x140U
#define DM_MIT_FIRST_ID 1U
#define DM_MIT_DEFAULT_KP 20.0f
#define DM_MIT_DEFAULT_KD 1.0f

//角度归一化
typedef struct{
	int16_t Voltage;//电压值
	float Angle;//机械角度
	float Speed;//转速
	int16_t Torque;//实际扭矩
	uint8_t Temp;//温度
	uint16_t ERR;
}RxMsg_t;

typedef struct{
	pid_t 				Speed_pid;
	pid_t 				Angle_pid;
	//控制角度的参数
	uint16_t			FirstEntre;
	double			Target;//目标角度
	float 				lastRead;//上一次读取值
	float 				currentRead;//当前读取值
	float 				Zero;//上电后的第一个位置做为零点
	float 				totalAngle;//总角度
	float				encoderAngle;//经过处理的电机角度
	int16_t				Current;//输出电流
	float				out;//输出电压

	float               target_angle;
	float               target_speed;
	float				angle;//目标角度
	float				speed;//目标速度
	float            	KP;
	float            	KD;
	float            	tor;
	uint32_t         	ID     ;//电机id

	RxMsg_t 			Rxmsg;
}motor_info_t;



void can_filter_init(void);
void can1_filter_init(void);
void can2_fliter_init(void);


void Set_voltage(CAN_HandleTypeDef* hcan,int16_t vlotage[]);
void Set_voltage_angle(CAN_HandleTypeDef* hcan,int16_t vlotage[]);
void Set_voltage_up_angle(CAN_HandleTypeDef* hcan,int16_t voltage[]);
void Set_dm_zeropoint(CAN_HandleTypeDef* hcan,uint16_t CAN_ID);
void Set_dm_enable(CAN_HandleTypeDef* hcan,uint8_t ID);
void Set_dm_disable(CAN_HandleTypeDef* hcan,uint8_t ID);
void MIT_Calc(motor_info_t *motor,int16_t target_torque,int32_t target_Angle,int16_t target_speed);
void Set_dm_speed(CAN_HandleTypeDef* hcan,int16_t ID,float speed);
void Set_dm_pos(CAN_HandleTypeDef* hcan,uint16_t ID,float pos,float vel);
void Set_dm_mit(CAN_HandleTypeDef* hcan,int16_t ID);
void dm_circle_test();
void dm_motor_fbdata(motor_info_t *motor, uint8_t *rx_data);
#endif
