//
// Created by Troge on 2026/1/22.
//

#ifndef CAN_TEST_DM_DRIVE_PROTOCOL_H
#define CAN_TEST_DM_DRIVE_PROTOCOL_H
#include "CAN.h"
/*MIT模式下各参数等比例转换系数，可以在参数设定页面查询*/
/*KP,KD默认为0.0~500*/
/*Pos、Vel、Torque分别预设为±12.5、±45、±18，这三个参数可以
根据电机的实际参数进行调整。但发送控制命令时，一定要与设定值保持一致，
否则会控制命令会发生等比例缩放。*/
#define P_MIN   -12.5
#define P_MAX   12.5
#define V_MIN    45
#define V_MAX   -45
#define KP_MIN  0.0
#define KP_MAX  500
#define KD_MIN  0.0
#define KD_MAX  500
#define TOR_MIN -18
#define TOR_MAX 18

typedef enum
{
    MIT_Ctl,//MIT模式
    POS__Vel_Ctl,//位置速度模式
    VEL_Ctl,//速度模式
}DM_Motor_Mode_t;

typedef struct
{
    DM_Motor_Mode_t Motor_Mode;
    uint16_t stdid;
    uint16_t ide;
    uint16_t rtr;
    uint8_t dlc;
    uint8_t contol_Frame[8];
}DM_Motor_t;

void DM_MIT_Frame_Load(DM_Motor_t* DM_Motor, float _pos, float _vel, float _KP,
                            float _KD, float _torq);
void DM_Pos__Vel_Frame_Load(DM_Motor_t* DM_Motor, float _pos, float _vel);
void DM_Vel_Frame_Load(DM_Motor_t* DM_Motor, float _vel);
void DM_Enable_Frame_Load(DM_Motor_t* DM_Motor);
void DM_Disable_Frame_Load(DM_Motor_t* DM_Motor);
void DM_Save_Zero_Frame_Load(DM_Motor_t* DM_Motor);
void DM_Clear_Error_Frame_Load(DM_Motor_t* DM_Motor);

#endif //CAN_TEST_DM_DRIVE_PROTOCOL_H