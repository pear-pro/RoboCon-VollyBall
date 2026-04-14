#include "can.h"
#include "includes.h"
#include "motor_can.h"
#include "pid.h"
#include <stdint.h>
extern motor_info_t damiao[MotorCount];
void All_Init(){

    can1_filter_init();
    can2_fliter_init();
	remote_control_init();
//    JY901P_Unlock();
//    JY901P_Calibrate_Full();
    Set_dm_zeropoint(&hcan1,0X00);
    Set_dm_enable(&hcan1,0X00);
    Set_dm_enable(&hcan1,0X01);
    for(int i=0;i<MotorCount;i++){
        PID_Struct_Init(&C620[i].Speed_pid, 
            2.0f, 
            0.3f,
            0.0f, 
            10000, 
            16000,
            INIT);
        PID_Struct_Init(&C620[i].Angel_pid,
            10.0f,
            0.0f,
            0.0f,
            300,
            300,
            INIT);
    }
	damiao[0].KP = 120.0f;//150.0f;
	damiao[0].KD = 6.0f;
	damiao[0].tor = 0.0f;//-1.65
	damiao[0].angle=0.0f;
    damiao[1].KP = 130.0f;//150.0f;
	damiao[1].KD = 6.0f;
	damiao[1].tor = -0.5f;//-1.65
	damiao[1].angle=-1.9f;
    PID_Struct_Init(&car_pid,
                    10.0f,0.0f, 0.0f,
                    1000, 1000, INIT);
    PID_Struct_Init(&C620_angle.Speed_pid, 10.0f, 0.3f, 0.0f, 10000, 16000, INIT);
    int16_t Z_zeropoint=gyro_data.Angle_Z;    
}

void All_Clear(){
    //Set_dm_disable(&hcan1, 0X00);
    damiao[0].KP = 0.0f;//150.0f;
	damiao[0].KD = 0.0f;
	damiao[0].tor = 0.0f;//-1.65
	damiao[0].angle=0.0f;
    for(int i=0;i<MotorCount;i++){
        pid_reset(&C620[i].Speed_pid, 0.0f, 0.0f, 0.0f);
        pid_reset(&C620[i].Angel_pid, 0.0f, 0.0f, 0.0f);
        pid_reset(&damiao[i].Speed_pid, 0.0f, 0.0f, 0.0f);
    }
    pid_reset(&car_pid, 0.0f, 0.0f, 0.0f);
    pid_reset(&C620_angle.Speed_pid, 0.0f, 0.0f, 0.0f);
}