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
    Set_dm_enable(&hcan1,0X00);
    Set_dm_enable(&hcan1, 0X01);
    Set_dm_zeropoint(&hcan1,0X00);
    Set_dm_zeropoint(&hcan1, 0X01);

    

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
        PID_Struct_Init(&damiao[i].Speed_pid, 
            0.004f, 
            0.000001f,
            0.0f, 
            16000, 
            16000,
            INIT);

    }
	damiao[0].KP = 20.0f;//150.0f;
	damiao[0].KD = 2.0f;
	damiao[0].tor = -1.60f;//-1.65
	damiao[0].angle=0.0f;
	

    PID_Struct_Init(&car_pid,
                    10.0f,0.0f, 0.0f,
                    1000, 1000, INIT);

    int16_t Z_zeropoint=gyro_data.Angle_Z;    
}