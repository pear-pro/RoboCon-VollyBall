//
// Created by Troge on 2026/1/22.
//

#include "DM_Drive_Protocol.h"

// 这两个转换函数需要首先确定两个等比例转换的最大最小值，这两个值可以
// 在参数设定页面查询，其中KP、KD的最大最小值默认分别为0.0~500.0、0.0~5.0。
// Pos、Vel、Torque如下图所示分别预设为±12.5、±45、±18，这三个参数可以
// 根据电机的实际参数进行调整。但发送控制命令时，一定要与设定值保持一致，
// 否则会控制命令会发生等比例缩放。

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

float float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

void DM_MIT_Frame_Load(DM_Motor_t* DM_Motor, float _pos, float _vel, float _KP,
                            float _KD, float _torq)
{
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    pos_tmp = float_to_uint(_pos,P_MIN,P_MAX,16);
    vel_tmp = float_to_uint(_vel,V_MIN,V_MAX,12);
    kp_tmp = float_to_uint(_KP,KP_MIN,KP_MAX,12);
    kd_tmp = float_to_uint(_KD,KD_MIN,KD_MAX,12);
    tor_tmp = float_to_uint(_torq,TOR_MIN,TOR_MAX,12);
    DM_Motor->contol_Frame[0] = (pos_tmp)>>8;//位置给定高八位
    DM_Motor->contol_Frame[1] = pos_tmp;     //位置给定低八位
    DM_Motor->contol_Frame[2] = (vel_tmp)>>4;//速度给定高八位
    DM_Motor->contol_Frame[3] = (vel_tmp&0x0F)<<4|(kp_tmp>>8);//速度给定低四位&位置比例系数KP高四位
    DM_Motor->contol_Frame[4] = kp_tmp;//位置比例系数KP低八位
    DM_Motor->contol_Frame[5] = (kd_tmp)>>4;//位置微分系数KD高八位
    DM_Motor->contol_Frame[6] = (kd_tmp&0x0F)<<4|(tor_tmp>>8);//位置微分系数KD低四位&扭矩给定高四位
    DM_Motor->contol_Frame[7] = tor_tmp;//扭矩给定低八位
}

void DM_Pos__Vel_Frame_Load(DM_Motor_t* DM_Motor, float _pos, float _vel)
{
    uint8_t *pbuf,*vbuf;
    pbuf = (uint8_t*)&_pos;
    vbuf = (uint8_t*)&_vel;
    DM_Motor->contol_Frame[0] = *pbuf;
    DM_Motor->contol_Frame[1] = *(pbuf+1);
    DM_Motor->contol_Frame[2] = *(pbuf+2);
    DM_Motor->contol_Frame[3] = *(pbuf+3);
    DM_Motor->contol_Frame[4] = *vbuf;
    DM_Motor->contol_Frame[5] = *(vbuf+1);
    DM_Motor->contol_Frame[6] = *(vbuf+2);
    DM_Motor->contol_Frame[7] = *(vbuf+3);
}

void DM_Vel_Frame_Load(DM_Motor_t* DM_Motor, float _vel)
{
    uint8_t *vbuf;
    vbuf = (uint8_t*)&_vel;
    DM_Motor->contol_Frame[0] = *vbuf;
    DM_Motor->contol_Frame[1] = *(vbuf+1);
    DM_Motor->contol_Frame[2] = *(vbuf+2);
    DM_Motor->contol_Frame[3] = *(vbuf+3);
}

void DM_Enable_Frame_Load(DM_Motor_t* DM_Motor)
{
    DM_Motor->contol_Frame[0] = 0xFF;
    DM_Motor->contol_Frame[1] = 0xFF;
    DM_Motor->contol_Frame[2] = 0xFF;
    DM_Motor->contol_Frame[3] = 0xFF;
    DM_Motor->contol_Frame[4] = 0xFF;
    DM_Motor->contol_Frame[5] = 0xFF;
    DM_Motor->contol_Frame[6] = 0xFF;
    DM_Motor->contol_Frame[7] = 0xFC;
}

void DM_Disable_Frame_Load(DM_Motor_t* DM_Motor)
{
    DM_Motor->contol_Frame[0] = 0xFF;
    DM_Motor->contol_Frame[1] = 0xFF;
    DM_Motor->contol_Frame[2] = 0xFF;
    DM_Motor->contol_Frame[3] = 0xFF;
    DM_Motor->contol_Frame[4] = 0xFF;
    DM_Motor->contol_Frame[5] = 0xFF;
    DM_Motor->contol_Frame[6] = 0xFF;
    DM_Motor->contol_Frame[7] = 0xFD;
}

void DM_Save_Zero_Frame_Load(DM_Motor_t* DM_Motor)
{
    DM_Motor->contol_Frame[0] = 0xFF;
    DM_Motor->contol_Frame[1] = 0xFF;
    DM_Motor->contol_Frame[2] = 0xFF;
    DM_Motor->contol_Frame[3] = 0xFF;
    DM_Motor->contol_Frame[4] = 0xFF;
    DM_Motor->contol_Frame[5] = 0xFF;
    DM_Motor->contol_Frame[6] = 0xFF;
    DM_Motor->contol_Frame[7] = 0xFE;
}

void DM_Clear_Error_Frame_Load(DM_Motor_t* DM_Motor)
{
    DM_Motor->contol_Frame[0] = 0xFF;
    DM_Motor->contol_Frame[1] = 0xFF;
    DM_Motor->contol_Frame[2] = 0xFF;
    DM_Motor->contol_Frame[3] = 0xFF;
    DM_Motor->contol_Frame[4] = 0xFF;
    DM_Motor->contol_Frame[5] = 0xFF;
    DM_Motor->contol_Frame[6] = 0xFF;
    DM_Motor->contol_Frame[7] = 0xFB;
}