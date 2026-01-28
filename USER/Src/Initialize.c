#include "includes.h"
#include "pid.h"
extern motor_info_t damiao[MotorCount];
void All_Init(){

    can1_filter_init();
    can2_fliter_init();
	remote_control_init();

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
//		PID_Struct_Init(&damiao[i].Speed_pid,
//            2.0f,
//            1.0f,
//            0.0f,
//            16000,
//            16000,
//            INIT);

    }
        
}