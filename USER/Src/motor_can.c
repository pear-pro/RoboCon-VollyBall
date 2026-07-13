/**
  ******************************************************************************
  * File Name          : CANTask.c
  * Description        : CANͨ������
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

motor_info_t DM4310_angle;
motor_info_t DM4310_up_angle;
motor_info_t DM4310_hit_angle[3];
motor_info_t C620_angle;
motor_info_t C620_up_angle[2];
motor_info_t C620_hit_angle[3];
motor_info_t damiao[MotorCount];
CAN_RxHeaderTypeDef can1RxMsg,can2RxMsg; //������Ϣ�ṹ��
uint8_t can1RxData[8],can2RxData[8];     //�������ݻ���
uint8_t isRcan1Started=0,isRcan2Started=0; //��־λ����ʾ CAN1 �� CAN2 �Ƿ�����������
uint8_t can1_update = 1;
uint8_t can2_update = 1;
//反馈超时保护
volatile uint16_t C620_up_angle_feedback_tick[2] = {0xffff, 0xffff}; //��־λ����ʾ CAN1 �� CAN2 �Ƿ����µ�������Ҫ����

static int16_t dji_motor_decode_int16(uint8_t high, uint8_t low)
{
	return (int16_t)((uint16_t)high << 8 | low);
}

static void c620_up_angle_feedback_update(uint8_t index, uint8_t *rx_data)
{
	motor_info_t *motor = &C620_up_angle[index];
	C620_up_angle_feedback_tick[index] = 0;
	motor->Rxmsg.Angle = ((rx_data[0] << 8) | rx_data[1]) * 360 / 8192.0f;
	motor->Rxmsg.Speed = dji_motor_decode_int16(rx_data[2], rx_data[3]);
	motor->Rxmsg.Torque = dji_motor_decode_int16(rx_data[4], rx_data[5]);
	motor->Rxmsg.Temp = rx_data[6];
	motor->currentRead = motor->Rxmsg.Angle;
	motor->Speed_pid.get = motor->Rxmsg.Speed;

	if (motor->FirstEntre == 0)
	{
		motor->Zero = motor->currentRead;
		motor->FirstEntre = 1;
		motor->lastRead = motor->currentRead;
		motor->totalAngle = 0;
	}

	float delta = motor->currentRead - motor->lastRead;
	if (delta > 180)
	{
		delta = delta - 360;
	}
	else if (delta < -180)
	{
		delta = delta + 360;
	}

	motor->totalAngle += delta;
	motor->Angle_pid.get = motor->totalAngle;
	motor->lastRead = motor->currentRead;
}

/********************CAN����*****************************/
//CAN���ݱ�Ƿ��ͣ���֤������Դ����?
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
	if(hcan == &hcan1){
		can1_update = 1;
	} else if(hcan == &hcan2){
		can2_update = 1;
	}
}


/*�˲������ü�can��ʼ��*/
void can1_filter_init(void)
{
	CAN_FilterTypeDef can1_filter_structure = {0};
	can1_filter_structure.SlaveStartFilterBank=14;
	can1_filter_structure.FilterActivation = ENABLE;//ʹ���˲���
	can1_filter_structure.FilterMode = CAN_FILTERMODE_IDMASK;//����ģʽ
	can1_filter_structure.FilterScale = CAN_FILTERSCALE_32BIT;
	can1_filter_structure.FilterIdHigh = 0x0000;//����������ɸѡID
	can1_filter_structure.FilterIdLow = 0x0000;
	can1_filter_structure.FilterMaskIdHigh = 0x0000;
	can1_filter_structure.FilterMaskIdLow = 0x0000;
	can1_filter_structure.FilterBank = 0;
	can1_filter_structure.FilterFIFOAssignment = CAN_RX_FIFO0;//ʹ��FIFO0
	HAL_CAN_ConfigFilter(&hcan1, &can1_filter_structure);

	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//ʹ���ж�
    isRcan1Started=1;
}


void can2_fliter_init(void)
{
	CAN_FilterTypeDef can2_filter_structure = {0};
	can2_filter_structure.SlaveStartFilterBank=14;
	can2_filter_structure.FilterActivation = ENABLE;//ʹ���˲���
	can2_filter_structure.FilterMode = CAN_FILTERMODE_IDMASK;//����ģʽ
	can2_filter_structure.FilterScale = CAN_FILTERSCALE_32BIT;
	can2_filter_structure.FilterIdHigh = 0x0000;//����������ɸѡID
	can2_filter_structure.FilterIdLow = 0x0000;
	can2_filter_structure.FilterMaskIdHigh = 0x0000;
	can2_filter_structure.FilterMaskIdLow = 0x0000;
	can2_filter_structure.FilterBank = 14;
	can2_filter_structure.FilterFIFOAssignment = CAN_RX_FIFO0;//ʹ��FIFO0
	HAL_CAN_ConfigFilter(&hcan2, &can2_filter_structure);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//ʹ���ж�
    isRcan2Started=1;
}
/*���õ�����?*/
void Set_voltage(CAN_HandleTypeDef* hcan,int16_t voltage[])
{
	uint32_t tx_mailbox;
  CAN_TxHeaderTypeDef canTxMsg;
  uint8_t             canTxData[8] = {0};
  canTxMsg.StdId = 0x200;
  canTxMsg.IDE   = CAN_ID_STD;//��׼ID
  canTxMsg.RTR   = CAN_RTR_DATA;//����֡
  canTxMsg.DLC   = 8;//���ݳ���
  for(int8_t i=0;i<4;i++)
  {
   canTxData[2*i]=(voltage[i]>>8)&0xff;
   canTxData[2*i+1]=(voltage[i])&0xff;
  }
	/* �ȼ���Ƿ��пյ�?TX mailbox��ֻ���п�λ�ŷ��ͱ��� */
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &canTxMsg, canTxData, &tx_mailbox);//���ͱ���
	}
}

void Set_voltage_angle(CAN_HandleTypeDef* hcan,int16_t voltage[])
{
	uint32_t tx_mailbox;
  CAN_TxHeaderTypeDef canTxMsg;
  uint8_t             canTxData[8] = {0};
  canTxMsg.StdId = 0x1FF;
  canTxMsg.IDE   = CAN_ID_STD;//��׼ID
  canTxMsg.RTR   = CAN_RTR_DATA;//����֡
  canTxMsg.DLC   = 8;//���ݳ���
   canTxData[0]=(voltage[0]>>8)&0xff;
   canTxData[1]=(voltage[0])&0xff;
	/* �ȼ���Ƿ��пյ�?TX mailbox��ֻ���п�λ�ŷ��ͱ��� */
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &canTxMsg, canTxData, &tx_mailbox);//���ͱ���
	}
}

//�?�?发出�?
void Set_voltage_up_angle(CAN_HandleTypeDef* hcan,int16_t voltage[])
{
	uint32_t tx_mailbox;
  CAN_TxHeaderTypeDef canTxMsg;
  uint8_t             canTxData[8] = {0};
  canTxMsg.StdId = 0x1FF;
  canTxMsg.IDE   = CAN_ID_STD;//��׼ID
  canTxMsg.RTR   = CAN_RTR_DATA;//����֡
  canTxMsg.DLC   = 8;//���ݳ���
  for(uint8_t i=0;i<2;i++)
  {
   canTxData[2*i]=(voltage[i]>>8)&0xff;
   canTxData[2*i+1]=(voltage[i])&0xff;
  }
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &canTxMsg, canTxData, &tx_mailbox);//���ͱ���
	}
}

//����3508  id=6,7,8
void Set_voltage_hit(CAN_HandleTypeDef* hcan,int16_t voltage[])
{
	uint32_t tx_mailbox;
  CAN_TxHeaderTypeDef canTxMsg;
  uint8_t             canTxData[8] = {0};
  canTxMsg.StdId = 0x1FF;
  canTxMsg.IDE   = CAN_ID_STD;//��׼ID
  canTxMsg.RTR   = CAN_RTR_DATA;//����֡
  canTxMsg.DLC   = 8;//���ݳ���
  for(uint8_t i=1;i<4;i++)
  {
   canTxData[2*i]=(voltage[i-1]>>8)&0xff;
   canTxData[2*i+1]=(voltage[i-1])&0xff;
  }
	/* �ȼ���Ƿ��пյ�?TX mailbox��ֻ���п�λ�ŷ��ͱ��� */
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &canTxMsg, canTxData, &tx_mailbox);//���ͱ���
	}
}

/**************������******** */

void Set_dm_mit(CAN_HandleTypeDef* hcan,int16_t ID)
{
	uint32_t tx_mailbox;
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
  can1TxMsg.IDE   = CAN_ID_STD;//��׼ID
  can1TxMsg.RTR   = CAN_RTR_DATA;//����֡
  can1TxMsg.DLC   = 8;//���ݳ���

    can1TxData[0] = (pos_tmp >> 8);
    can1TxData[1] = pos_tmp;
    can1TxData[2] = (vel_tmp >> 4);
    can1TxData[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    can1TxData[4] = kp_tmp;
    can1TxData[5] = (kd_tmp >> 4);
    can1TxData[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    can1TxData[7] = tor_tmp;

	/* �ȼ���Ƿ��пյ�?TX mailbox��ֻ���п�λ�ŷ��ͱ��� */
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &can1TxMsg, can1TxData, &tx_mailbox);//���ͱ���
	}
}

void Set_dm_speed(CAN_HandleTypeDef* hcan,int16_t ID,float  speed)
{
	uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_msg;
    uint8_t tx_data[8] = {0};

    tx_msg.StdId = 0x200 + ID;  //����ID �����趨�� CAN ID ֵ + 0x200
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 4;   // �ٶ�ģʽֻ��Ҫ����4�ֽ�����

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

	tx_msg.StdId = 0x100 + ID;  //����ID �����趨�� CAN ID ֵ + 0x100
	tx_msg.IDE = CAN_ID_STD;
	tx_msg.RTR = CAN_RTR_DATA;
	tx_msg.DLC = 8;   // λ��ģʽ��Ҫ����8�ֽ�����

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
  uint32_t tx_mailbox;
  CAN_TxHeaderTypeDef can1TxMsg;
  uint8_t             can1TxData[8] = {0};

  can1TxMsg.StdId = 0x00+ID;  //ģʽƫ��ID��MITģʽƫ��0x00��λ���ٶ�ģʽƫ��0x100���ٶ�ģʽƫ��0x200����λ���ģʽƫ��?x300
  can1TxMsg.IDE   = CAN_ID_STD;//��׼ID
  can1TxMsg.RTR   = CAN_RTR_DATA;//����֡
  can1TxMsg.DLC   = 8;//���ݳ���

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
			HAL_CAN_AddTxMessage(hcan, &can1TxMsg, can1TxData, &tx_mailbox);//����?������??
	}
}

void Set_dm_disable(CAN_HandleTypeDef* hcan,uint8_t ID)
{
  uint32_t tx_mailbox;
  CAN_TxHeaderTypeDef can1TxMsg;
  uint8_t             can1TxData[8] = {0};
  can1TxMsg.StdId = 0x00+ID;
  can1TxMsg.IDE   = CAN_ID_STD;//��׼ID
  can1TxMsg.RTR   = CAN_RTR_DATA;//����֡
  can1TxMsg.DLC   = 8;//���ݳ���

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
			HAL_CAN_AddTxMessage(hcan, &can1TxMsg, can1TxData, &tx_mailbox);
	}
}

void Set_dm_zeropoint(CAN_HandleTypeDef* hcan,uint16_t CAN_ID)
{
  uint32_t tx_mailbox;
  CAN_TxHeaderTypeDef can1TxMsg;
  uint8_t             can1TxData[8] = {0};
  can1TxMsg.StdId = CAN_ID;
  can1TxMsg.IDE   = CAN_ID_STD;//��׼ID
  can1TxMsg.RTR   = CAN_RTR_DATA;//����֡
  can1TxMsg.DLC   = 8;//���ݳ���
  can1TxData[0]=0xff;
  can1TxData[1]=0xff;
  can1TxData[2]=0xff;
  can1TxData[3]=0xff;
  can1TxData[4]=0xff;
  can1TxData[5]=0xff;
  can1TxData[6]=0xff;
  can1TxData[7]=0xfe;

	/* �ȼ���Ƿ��пյ�?TX mailbox��ֻ���п�λ�ŷ��ͱ��� */
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &can1TxMsg, can1TxData, &tx_mailbox);//���ͱ���
	}
}

void dm_motor_fbdata(motor_info_t *motor, uint8_t *rx_data) //master_idĬ��Ϊ0(��Ӱ�����?
{
//    // �������ID��״̬��һ���ò�������Э�����У�
//    motor->para.id = (rx_data[0]) & 0x0F;
//    motor->para.state = (rx_data[0]) >> 4;

    // ����λ��ԭʼֵ�����ֽ�+���ֽڣ�
    uint16_t p_int = (rx_data[1] << 8) | rx_data[2];
    // �����ٶ�ԭʼֵ�����ֽ�ƴ�ӣ�
    uint16_t v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    // ����ת��ԭʼֵ�����ֽ�ƴ�ӣ�
    uint16_t t_int = ((rx_data[4] & 0x0F) << 8) | rx_data[5];

    // ��ԭʼֵת����ʵ������ֵ
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
	// ����˶����?
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

/********************CAN����*****************************/
//�����жϻص�����
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef can1RxMsg;
  CAN_RxHeaderTypeDef can2RxMsg;
	if(hcan==&hcan1)
	{
		uint8_t message_count = 0;
		while (message_count < 3)
		{
			uint32_t fifo_fill = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
			if (fifo_fill == 0)
			{
				break;
			}

			if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can1RxMsg, can1RxData) != HAL_OK)
			{
				break;
			}

			uint8_t motor_id = can1RxData[0] & 0x0FU;
			if (motor_id < MotorCount)
			{
				dm_motor_fbdata(&damiao[motor_id], can1RxData);
			}

			message_count++;
		}
	}
    if(hcan==&hcan2) //���̼ӽǶ�3508
    {
        uint8_t message_count = 0;

        while (message_count < 3)
        {
            uint32_t fifo_fill = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
            if (fifo_fill == 0)
            {
                break;
            }

            if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can2RxMsg, can2RxData) != HAL_OK)
            {
                break;
            }

            for(int i=0;i<MotorCount;i++)
            {
                if(can2RxMsg.StdId==0x201+i){
                    C620[i].Rxmsg.Angle= ((can2RxData[0] << 8) | can2RxData[1])*360/8192.0f;
                    C620[i].Rxmsg.Speed= dji_motor_decode_int16(can2RxData[2], can2RxData[3]);
                    C620[i].Rxmsg.Torque= dji_motor_decode_int16(can2RxData[4], can2RxData[5]);
                    C620[i].Rxmsg.Temp=can2RxData[6];
                    C620[i].Speed_pid.get=C620[i].Rxmsg.Speed;
                }
            }
            if(can2RxMsg.StdId==0x205){
                c620_up_angle_feedback_update(0, can2RxData);
            }
            else if(can2RxMsg.StdId==0x206){
                c620_up_angle_feedback_update(1, can2RxData);
            }

            message_count++;
        }
    }
}