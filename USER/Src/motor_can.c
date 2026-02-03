/**
  ******************************************************************************
  * File Name          : CANTask.c
  * Description        : CAN通信任务
  ******************************************************************************
  *
  * Copyright (c) 2025 Team TPP-FoShan University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "motor_can.h"
#include "includes.h"
#include "can.h"
#include "main.h"
#include "math_utils.h"
#include "pid.h"
#include "pid.h"
#include "pid_tim.h"
#include "stm32f4xx_hal_can.h"
#include <stdint.h>
motor_info_t C6xx[MotorCount];
motor_info_t damiao[MotorCount];
CAN_RxHeaderTypeDef can1RxMsg,can2RxMsg; //接受消息结构体
uint8_t can1RxData[8],can2RxData[8];     //接受数据缓存
uint8_t isRcan1Started=0,isRcan2Started=0; //标志位，表示 CAN1 和 CAN2 是否已启动接收
uint8_t can1_update = 1;
uint8_t can2_update = 1; //标志位，表示 CAN1 和 CAN2 是否有新的数据需要发送

/********************CAN发送*****************************/
//CAN数据标记发送，保证发送资源正常
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
	if(hcan == &hcan1){
		can1_update = 1;
	} else if(hcan == &hcan2){
		can2_update = 1;
	}
}


/*滤波器配置及can初始化*/
void can1_filter_init(void)
{ 	
	CAN_FilterTypeDef can1_filter_structure;
	can1_filter_structure.FilterActivation = ENABLE;//使能滤波器
	can1_filter_structure.FilterMode = CAN_FILTERMODE_IDMASK;//掩码模式
	can1_filter_structure.FilterScale = CAN_FILTERSCALE_32BIT;
	can1_filter_structure.FilterIdHigh = 0x0000;//下面配置则不筛选ID
	can1_filter_structure.FilterIdLow = 0x0000;
	can1_filter_structure.FilterMaskIdHigh = 0x0000;
	can1_filter_structure.FilterMaskIdLow = 0x0000;
	can1_filter_structure.FilterBank = 0;
	can1_filter_structure.FilterFIFOAssignment = CAN_RX_FIFO0;//使用FIFO0
	HAL_CAN_ConfigFilter(&hcan1, &can1_filter_structure);
	
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//使能中断
    isRcan1Started=1;
}


void can2_fliter_init(void)
{
	CAN_FilterTypeDef can2_filter_structure;
	can2_filter_structure.FilterActivation = ENABLE;//使能滤波器
	can2_filter_structure.FilterMode = CAN_FILTERMODE_IDMASK;//掩码模式
	can2_filter_structure.FilterScale = CAN_FILTERSCALE_32BIT;
	can2_filter_structure.FilterIdHigh = 0x0000;//下面配置则不筛选ID
	can2_filter_structure.FilterIdLow = 0x0000;
	can2_filter_structure.FilterMaskIdHigh = 0x0000;
	can2_filter_structure.FilterMaskIdLow = 0x0000;
	can2_filter_structure.FilterBank = 0;
	can2_filter_structure.FilterFIFOAssignment = CAN_RX_FIFO0;//使用FIFO0
	HAL_CAN_ConfigFilter(&hcan2, &can2_filter_structure);
	HAL_CAN_ConfigFilter(&hcan2, &can2_filter_structure);
	
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//使能中断
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//使能中断
    isRcan2Started=1;
}
/*设置电机电压*/
void Set_voltagec1(CAN_HandleTypeDef* hcan,int16_t voltage[])
{
	uint32_t tx_mailbox;
  CAN_TxHeaderTypeDef can1TxMsg;
  uint8_t             can1TxData[8] = {0};
  can1TxMsg.StdId = 0x200;
  can1TxMsg.IDE   = CAN_ID_STD;//标准ID
  can1TxMsg.RTR   = CAN_RTR_DATA;//数据帧
  can1TxMsg.DLC   = 8;//数据长度
  for(int8_t i=0;i<4;i++)
  {
   can1TxData[2*i]=(voltage[i]>>8)&0xff;
   can1TxData[2*i+1]=(voltage[i])&0xff;
  }
	/* 先检查是否有空的 TX mailbox，只有有空位才发送报文 */
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &can1TxMsg, can1TxData, &tx_mailbox);//发送报文
	}
}

/**************达妙电机******** */

void Set_dm(CAN_HandleTypeDef* hcan,int16_t g)
{
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
  CAN_TxHeaderTypeDef can2TxMsg;
  uint8_t             can2TxData[8] = {0};
  if (g==1){
  can2TxMsg.StdId =  damiao[0].ID +0x000;
  }
  else if(g==2)
  {
  can2TxMsg.StdId =  damiao[1].ID +0x000;
	  
  }
   else if(g==3)
  {
  can2TxMsg.StdId =  damiao[2].ID+0x000 ;
	  
  }
  else if(g==4)
  {
  can2TxMsg.StdId =  damiao[3].ID +0x000;
	  
  }
    pos_tmp = float_to_uint(damiao[g].angle, -12.5, 12.5, 16);
    vel_tmp = float_to_uint(damiao[g].speed, -30, 30, 12);
    tor_tmp = float_to_uint(damiao[g].tor, -10,10, 12);
    kp_tmp  = float_to_uint(damiao[g].KP, 0.0, 500.0, 12);
    kd_tmp  = float_to_uint(damiao[g].KD,  0.0, 5.0, 12);  
  can2TxMsg.IDE   = CAN_ID_STD;//标准ID
  can2TxMsg.RTR   = CAN_RTR_DATA;//数据帧
  can2TxMsg.DLC   = 8;//数据长度
  
    can2TxData[0] = (pos_tmp >> 8);
    can2TxData[1] = pos_tmp;
    can2TxData[2] = (vel_tmp >> 4);
    can2TxData[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    can2TxData[4] = kp_tmp;
    can2TxData[5] = (kd_tmp >> 4);
    can2TxData[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    can2TxData[7] = tor_tmp;
  
	/* 先检查是否有空的 TX mailbox，只有有空位才发送报文 */
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
 			HAL_CAN_AddTxMessage(hcan, &can2TxMsg, can2TxData, (uint32_t*)CAN_TX_MAILBOX0);//发送报文
	}
}
void Set_dm_enable(CAN_HandleTypeDef* hcan,uint8_t ID)
{


  CAN_TxHeaderTypeDef can2TxMsg;
  uint8_t             can2TxData[8] = {0};
  can2TxMsg.StdId = 0x0+ID;
  can2TxMsg.IDE   = CAN_ID_STD;//标准ID
  can2TxMsg.RTR   = CAN_RTR_DATA;//数据帧
  can2TxMsg.DLC   = 8;//数据长度
  
    can2TxData[0] = 0xFF;
    can2TxData[1] = 0xFF;
    can2TxData[2] = 0xFF;
    can2TxData[3] = 0xFF;
    can2TxData[4] = 0xFF;
    can2TxData[5] = 0xFF;
    can2TxData[6] = 0xFF;
    can2TxData[7] = 0xFC;	
		
  
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &can2TxMsg, can2TxData, (uint32_t*)CAN_TX_MAILBOX0);//·￠?í±¨??
	}
}

void Set_dm_zeropoint(CAN_HandleTypeDef* hcan,uint16_t CAN_ID)
{
  CAN_TxHeaderTypeDef can2TxMsg;
  uint8_t             can2TxData[8] = {0};
  can2TxMsg.StdId = 0x7FF;
  can2TxMsg.IDE   = CAN_ID_STD;//标准ID
  can2TxMsg.RTR   = CAN_RTR_DATA;//数据帧
  can2TxMsg.DLC   = 4;//数据长度
  can2TxData[0]=(CAN_ID>>8)&0xff;
  can2TxData[1]=(CAN_ID)&0xff;
  can2TxData[2]=0x55;
  can2TxData[3]=0x50;
	/* 先检查是否有空的 TX mailbox，只有有空位才发送报文 */
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &can2TxMsg, can2TxData, (uint32_t*)CAN_TX_MAILBOX0);//发送报文
	}
}
void MIT_Calc(motor_info_t *motor,int16_t target_torque,int32_t target_Angle,int16_t target_speed)
{
	target_Angle*=19;
	int32_t Angle_delta=target_Angle-motor->totalAngle;
	int16_t speed_delta=target_speed-motor->Rxmsg.Speed;
	float kp=10;
	float kd=5;
	motor->out=clamp_max(target_torque+
				kp*Angle_delta+
				kd*speed_delta, 16000);
}


  



/********************CAN接收*****************************/
//接收中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t flag=0;
  CAN_RxHeaderTypeDef can1RxMsg;
  CAN_RxHeaderTypeDef can2RxMsg;
  if(hcan==&hcan1)
  {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can1RxMsg, can1RxData);
	for(int i=0;i<MotorCount;i++)
	{
		if(can1RxMsg.StdId==0x201+i){
			C620[i].Rxmsg.Angle= ((can1RxData[0] << 8) | can1RxData[1])*360/8192.0f;
			C620[i].Rxmsg.Speed= ((can1RxData[2] << 8) | can1RxData[3]);
			C620[i].Rxmsg.Torque=((can1RxData[4] << 8) | can1RxData[5]);
			C620[i].Rxmsg.Temp=can1RxData[6];
			C620[i].Speed_pid.get=C620[i].Rxmsg.Speed;
			pid_calc(&C620[i].Speed_pid,C620[i].Speed_pid.get,C620[i].Speed_pid.set);
			flag=1;
		}
	}
  }
    if(hcan==&hcan2)
		{
			//HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can2RxMsg, can2RxData);
			//for(int i=0;i<MotorCount;i++)
			//{
			//	if(can2RxMsg.StdId==0x201+i){
			//		C6xx[i].Rxmsg.Angle= ((can2RxData[0] << 8) | can2RxData[1])*360/8192.0f;
			//		C6xx[i].Rxmsg.Speed= ((can2RxData[2] << 8) | can2RxData[3]);
			//		C6xx[i].Rxmsg.Torque=((can2RxData[4] << 8) | can2RxData[5]);
			//		C6xx[i].Rxmsg.Temp=can2RxData[6];
			//		C6xx[i].currentRead=C6xx[i].Rxmsg.Angle;
			//		//计算相对零点转了多少度
			//		if(C6xx[i].FirstEntre==0)
			//		{
			//			C6xx[i].Zero=C6xx[i].currentRead;
			//			C6xx[i].FirstEntre=1;
			//			C6xx[i].lastRead=C6xx[i].currentRead;
			//			C6xx[i].totalAngle=0;
			//		}
			//		int16_t delta=C6xx[i].currentRead-C6xx[i].lastRead;
			//		if(delta>180)
			//		{
			//			delta=delta-360;
			//		}
			//		else if(delta<-180)
			//		{
			//			delta=delta+360;
			//		}
			//		else
			//		{
			//			delta=delta+0;
			//		}
			//		C6xx[i].totalAngle+=delta;
			//		C6xx[i].lastRead=C6xx[i].currentRead;
			//		Vofa_JustFloat((float*)C6xx[i].totalAngle, 1);
			//	}
			//}

		}
	
}



