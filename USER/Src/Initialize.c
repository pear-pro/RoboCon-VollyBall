#include "includes.h"
#include "pid.h"
void All_Init(){

    can1_filter_init();
    can2_fliter_init();

    for(int i=0;i<MotorCount;i++){
        PID_Struct_Init(&C620[i].Speed_pid, 
            2.0f, 
            1.0f,
            0.0f, 
            16000, 
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
        
}