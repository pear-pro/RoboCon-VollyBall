#include "includes.h"
#include "pid.h"
extern motor_info_t damiao[MotorCount];
void All_Init(){

    can1_filter_init();
    can2_fliter_init();
	remote_control_init();
    JY901P_Unlock();
    JY901P_Calibrate_Full();

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
       damiao[i].ID=0X001+i;
	   damiao[i].KP=normalize_to_range(55.0, 30.0, 55.0, 1.0, 10.0); 
	   damiao[i].KD=1.0f; 
	   damiao[i].tor=55.0f;
	   
    }
        
}