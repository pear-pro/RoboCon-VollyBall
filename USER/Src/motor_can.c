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
motor_info_t C620_angle;
motor_info_t C620_up_angle;
motor_info_t damiao[MotorCount];
CAN_RxHeaderTypeDef can1RxMsg,can2RxMsg; //接受消息结构体
uint8_t can1RxData[8],can2RxData[8];     //接受数据缓存
uint8_t isRcan1Started=0,isRcan2Started=0; //标志位，表示 CAN1 和 CAN2 是否已启动接收
uint8_t can1_update = 1;
uint8_t can2_update = 1; //标志位，表示 CAN1 和 CAN2 是否有新的数据需要发送

static int16_t dji_motor_decode_int16(uint8_t high, uint8_t low)
{
	return (int16_t)((uint16_t)high << 8 | low);
}

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
	can1_filter_structure.SlaveStartFilterBank=14;
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
	can2_filter_structure.FilterBank = 14;
	can2_filter_structure.FilterFIFOAssignment = CAN_RX_FIFO0;//使用FIFO0
	HAL_CAN_ConfigFilter(&hcan2, &can2_filter_structure);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//使能中断
    isRcan2Started=1;
}
/*设置电机电压*/
void Set_voltage(CAN_HandleTypeDef* hcan,int16_t voltage[])
{
	uint32_t tx_mailbox;
  CAN_TxHeaderTypeDef canTxMsg;
  uint8_t             canTxData[8] = {0};
  canTxMsg.StdId = 0x200;
  canTxMsg.IDE   = CAN_ID_STD;//标准ID
  canTxMsg.RTR   = CAN_RTR_DATA;//数据帧
  canTxMsg.DLC   = 8;//数据长度
  for(int8_t i=0;i<4;i++)
  {
   canTxData[2*i]=(voltage[i]>>8)&0xff;
   canTxData[2*i+1]=(voltage[i])&0xff;
  }
	/* 先检查是否有空的 TX mailbox，只有有空位才发送报文 */
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &canTxMsg, canTxData, &tx_mailbox);//发送报文
	}
}

void Set_voltage_angle(CAN_HandleTypeDef* hcan,int16_t voltage[])
{
	uint32_t tx_mailbox;
  CAN_TxHeaderTypeDef canTxMsg;
  uint8_t             canTxData[8] = {0};
  canTxMsg.StdId = 0x1FF;
  canTxMsg.IDE   = CAN_ID_STD;//标准ID
  canTxMsg.RTR   = CAN_RTR_DATA;//数据帧
  canTxMsg.DLC   = 8;//数据长度
   canTxData[0]=(voltage[0]>>8)&0xff;
   canTxData[1]=(voltage[0])&0xff;
	/* 先检查是否有空的 TX mailbox，只有有空位才发送报文 */
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &canTxMsg, canTxData, &tx_mailbox);//发送报文
	}
}

void Set_voltage_up_angle(CAN_HandleTypeDef* hcan,int16_t voltage[])
{
	uint32_t tx_mailbox;
  CAN_TxHeaderTypeDef canTxMsg;
  uint8_t             canTxData[8] = {0};
  canTxMsg.StdId = 0x1FF;
  canTxMsg.IDE   = CAN_ID_STD;//标准ID
  canTxMsg.RTR   = CAN_RTR_DATA;//数据帧
  canTxMsg.DLC   = 8;//数据长度
   canTxData[2]=(voltage[0]>>8)&0xff;
   canTxData[3]=(voltage[0])&0xff;
	/* 先检查是否有空的 TX mailbox，只有有空位才发送报文 */
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &canTxMsg, canTxData, &tx_mailbox);//发送报文
	}
}

/**************达妙电机******** */

void Set_dm_mit(CAN_HandleTypeDef* hcan,int16_t ID)
{
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
  CAN_TxHeaderTypeDef can1TxMsg;
  uint8_t             can1TxData[8] = {0};
	if (ID==0){
  can1TxMsg.StdId =0x00;
  }
  else if(ID==1)
  {
  can1TxMsg.StdId =0x01;

  }
   else if(ID==2)
  {
  can1TxMsg.StdId =0x02 ;

  }
  else if(ID==3)
  {
  can1TxMsg.StdId =0x03;

  }
    pos_tmp = float_to_uint(damiao[ID].angle, -12.5, 12.5, 16);
    vel_tmp = float_to_uint(damiao[ID].speed, -30, 30, 12);
    tor_tmp = float_to_uint(damiao[ID].tor, -10,10, 12);
    kp_tmp  = float_to_uint(damiao[ID].KP, 0.0, 500.0, 12);
    kd_tmp  = float_to_uint(damiao[ID].KD,  0.0, 5.0, 12);
  can1TxMsg.IDE   = CAN_ID_STD;//标准ID
  can1TxMsg.RTR   = CAN_RTR_DATA;//数据帧
  can1TxMsg.DLC   = 8;//数据长度

    can1TxData[0] = (pos_tmp >> 8);
    can1TxData[1] = pos_tmp;
    can1TxData[2] = (vel_tmp >> 4);
    can1TxData[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    can1TxData[4] = kp_tmp;
    can1TxData[5] = (kd_tmp >> 4);
    can1TxData[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    can1TxData[7] = tor_tmp;

	/* 先检查是否有空的 TX mailbox，只有有空位才发送报文 */
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
 			HAL_CAN_AddTxMessage(hcan, &can1TxMsg, can1TxData, (uint32_t*)CAN_TX_MAILBOX0);//发送报文
	}
}

void Set_dm_speed(CAN_HandleTypeDef* hcan,int16_t ID,float  speed)
{
	uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_msg;
    uint8_t tx_data[8] = {0};

    tx_msg.StdId = 0x200 + ID;  //报文ID 等于设定的 CAN ID 值 + 0x200
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 4;   // 速度模式只需要发送4字节数据

    uint8_t *p = (uint8_t *)&speed;

    tx_data[0] = p[0];
    tx_data[1] = p[1];
    tx_data[2] = p[2];
    tx_data[3] = p[3];

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
    {
        HAL_CAN_AddTxMessage(hcan, &tx_msg, tx_data, &tx_mailbox);
    }
}

void Set_dm_pos(CAN_HandleTypeDef* hcan,uint16_t ID,float pos,float vel)
{
	uint32_t tx_mailbox;
	CAN_TxHeaderTypeDef tx_msg;
	uint8_t tx_data[8] = {0};

	tx_msg.StdId = 0x100 + ID;  //报文ID 等于设定的 CAN ID 值 + 0x100
	tx_msg.IDE = CAN_ID_STD;
	tx_msg.RTR = CAN_RTR_DATA;
	tx_msg.DLC = 8;   // 位置模式需要发送8字节数据

	uint8_t *p_pos = (uint8_t *)&pos;
	uint8_t *p_vel = (uint8_t *)&vel;

	tx_data[0] = p_pos[0];
	tx_data[1] = p_pos[1];
	tx_data[2] = p_pos[2];
	tx_data[3] = p_pos[3];
	tx_data[4] = p_vel[0];
	tx_data[5] = p_vel[1];
	tx_data[6] = p_vel[2];
	tx_data[7] = p_vel[3];

	if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
		HAL_CAN_AddTxMessage(hcan, &tx_msg, tx_data, &tx_mailbox);
	}
}

void Set_dm_enable(CAN_HandleTypeDef* hcan,uint8_t ID)
{
  CAN_TxHeaderTypeDef can1TxMsg;
  uint8_t             can1TxData[8] = {0};

  can1TxMsg.StdId = 0x00+ID;  //模式偏移ID：MIT模式偏移0x00，位置速度模式偏移0x100，速度模式偏移0x200，力位混控模式偏移0x300
  can1TxMsg.IDE   = CAN_ID_STD;//标准ID
  can1TxMsg.RTR   = CAN_RTR_DATA;//数据帧
  can1TxMsg.DLC   = 8;//数据长度

    can1TxData[0] = 0xFF;
    can1TxData[1] = 0xFF;
    can1TxData[2] = 0xFF;
    can1TxData[3] = 0xFF;
    can1TxData[4] = 0xFF;
    can1TxData[5] = 0xFF;
    can1TxData[6] = 0xFF;
    can1TxData[7] = 0xFC;


	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &can1TxMsg, can1TxData, (uint32_t*)CAN_TX_MAILBOX0);//·￠?í±¨??
	}
}

void Set_dm_disable(CAN_HandleTypeDef* hcan,uint8_t ID)
{
  CAN_TxHeaderTypeDef can1TxMsg;
  uint8_t             can1TxData[8] = {0};
  can1TxMsg.StdId = 0x00+ID;
  can1TxMsg.IDE   = CAN_ID_STD;//标准ID
  can1TxMsg.RTR   = CAN_RTR_DATA;//数据帧
  can1TxMsg.DLC   = 8;//数据长度

	can1TxData[0] = 0xFF;
	can1TxData[1] = 0xFF;
	can1TxData[2] = 0xFF;
	can1TxData[3] = 0xFF;
	can1TxData[4] = 0xFF;
	can1TxData[5] = 0xFF;
	can1TxData[6] = 0xFF;
	can1TxData[7] = 0xFD;


	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &can1TxMsg, can1TxData, (uint32_t*)CAN_TX_MAILBOX0);
	}
}

void Set_dm_zeropoint(CAN_HandleTypeDef* hcan,uint16_t CAN_ID)
{
  CAN_TxHeaderTypeDef can1TxMsg;
  uint8_t             can1TxData[8] = {0};
  can1TxMsg.StdId = CAN_ID;
  can1TxMsg.IDE   = CAN_ID_STD;//标准ID
  can1TxMsg.RTR   = CAN_RTR_DATA;//数据帧
  can1TxMsg.DLC   = 8;//数据长度
  can1TxData[0]=0xff;
  can1TxData[1]=0xff;
  can1TxData[2]=0xff;
  can1TxData[3]=0xff;
  can1TxData[4]=0xff;
  can1TxData[5]=0xff;
  can1TxData[6]=0xff;
  can1TxData[7]=0xfe;

	/* 先检查是否有空的 TX mailbox，只有有空位才发送报文 */
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &can1TxMsg, can1TxData, (uint32_t*)CAN_TX_MAILBOX0);//发送报文
	}
}

void dm_motor_fbdata(motor_info_t *motor, uint8_t *rx_data) //master_id默认为0(不影响解析)
{
//    // 解析电机ID和状态（一般用不到，但协议里有）
//    motor->para.id = (rx_data[0]) & 0x0F;
//    motor->para.state = (rx_data[0]) >> 4;

    // 解析位置原始值（高字节+低字节）
    uint16_t p_int = (rx_data[1] << 8) | rx_data[2];
    // 解析速度原始值（跨字节拼接）
    uint16_t v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    // 解析转矩原始值（跨字节拼接）
    uint16_t t_int = ((rx_data[4] & 0x0F) << 8) | rx_data[5];

    // 把原始值转换成实际物理值
    motor->Rxmsg.Angle = uint_to_float(p_int, -12.5,12.5,16);
    motor->Rxmsg.Speed = uint_to_float(v_int, -30,30,12);
    motor->Rxmsg.Torque = uint_to_float(t_int, -10,10,12);
	motor->Angle_pid.get = motor->Rxmsg.Angle;
}



void MIT_Calc(motor_info_t *motor,int16_t target_torque,int32_t target_Angle,int16_t target_speed)
{
	target_Angle*=19;
	float Angle_delta=target_Angle-motor->totalAngle;
	float speed_delta=target_speed-motor->Rxmsg.Speed;
	float kp=18;
	float kd=5;
	motor->out=clamp_max(target_torque+
				kp*Angle_delta+
				kd*speed_delta, 20000);
}

/*0 1.597
  1 1.725 -1.712*/
const float pi=3.1415926;
void dm_circle_test()
{
	static float theta=0.0f;
	static uint32_t k=0;
	const float omega = 0.002f * pi;
	// 电机运动范围
    const float motor0_min = -1.35f;
    const float motor0_max =  1.35f;
    const float motor1_min = 1.35f;
    const float motor1_max =  -1.35f;

    const float center0 = (motor0_max + motor0_min) * 0.5f;
    const float center1 = (motor1_max + motor1_min) * 0.5f;
    const float r = (motor0_max - motor0_min) * 0.5f;
	 damiao[0].angle = center0 + r * cosf(theta+k*pi);
	 damiao[1].angle= center1 + r*sinf(theta+k*pi);
   theta+=omega*pi;
   if(theta==(k+1)*pi){k++;}
}

/********************CAN接收*****************************/
//接收中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t flag=0;
  CAN_RxHeaderTypeDef can1RxMsg;
  CAN_RxHeaderTypeDef can2RxMsg;
//	if(hcan==&hcan1)
//	{
//		 HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can1RxMsg, can1RxData);
//         uint8_t motor_id=can1RxData[0] & 0x0F; //电机ID在第一个字节的低4位
//		if (motor_id>=0 && motor_id < MotorCount)
//		{
//			dm_motor_fbdata(&damiao[motor_id], can1RxData);
//		}
//	}
    if(hcan==&hcan2) //底盘加角度3508
		{

			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can2RxMsg, can2RxData);
			for(int i=0;i<MotorCount;i++)
			{
				if(can2RxMsg.StdId==0x201+i){
					C620[i].Rxmsg.Angle= ((can2RxData[0] << 8) | can2RxData[1])*360/8192.0f;
					C620[i].Rxmsg.Speed= dji_motor_decode_int16(can2RxData[2], can2RxData[3]);
					C620[i].Rxmsg.Torque= dji_motor_decode_int16(can2RxData[4], can2RxData[5]);
					C620[i].Rxmsg.Temp=can2RxData[6];
					C620[i].Speed_pid.get=C620[i].Rxmsg.Speed;
					flag=1;
				}
			}
			//俯仰角3508
				if(can2RxMsg.StdId==0x205){
					C620_angle.Rxmsg.Angle= ((can2RxData[0] << 8) | can2RxData[1])*360/8192.0f;
					C620_angle.Rxmsg.Speed= dji_motor_decode_int16(can2RxData[2], can2RxData[3]);
					C620_angle.Rxmsg.Torque= dji_motor_decode_int16(can2RxData[4], can2RxData[5]);
					C620_angle.Rxmsg.Temp=can2RxData[6];
					C620_angle.Speed_pid.get=C620_angle.Rxmsg.Speed;
				}			//HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can2RxMsg, can2RxData);
			if(can2RxMsg.StdId==0x206){
				C620_up_angle.Rxmsg.Angle= ((can2RxData[0] << 8) | can2RxData[1])*360/8192.0f;
				C620_up_angle.Rxmsg.Speed= dji_motor_decode_int16(can2RxData[2], can2RxData[3]);
				C620_up_angle.Rxmsg.Torque= dji_motor_decode_int16(can2RxData[4], can2RxData[5]);
				C620_up_angle.Rxmsg.Temp=can2RxData[6];
				C620_up_angle.currentRead=C620_up_angle.Rxmsg.Angle;
				C620_up_angle.Speed_pid.get=C620_up_angle.Rxmsg.Speed;
				//计算相对零点转了多少度
				if(C620_up_angle.FirstEntre==0)
				{
					C620_up_angle.Zero=C620_up_angle.currentRead;
					C620_up_angle.FirstEntre=1;
					C620_up_angle.lastRead=C620_up_angle.currentRead;
					C620_up_angle.totalAngle=0;
				}
				float delta=C620_up_angle.currentRead-C620_up_angle.lastRead;
				if(delta>180)
				{
					delta=delta-360;
				}
				else if(delta<-180)
				{
					delta=delta+360;
				}
				else
				{
					delta=delta+0;
				}
				C620_up_angle.totalAngle+=delta;
				C620_up_angle.Angle_pid.get=C620_up_angle.totalAngle;
				C620_up_angle.lastRead=C620_up_angle.currentRead;
				//Vofa_JustFloat(&C620_up_angle.Angle_pid.set, 1);
			}
				}
			}
