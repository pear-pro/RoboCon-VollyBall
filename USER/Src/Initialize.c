#include "can.h"
#include "includes.h"
#include "motor_can.h"
#include "pid.h"
#include <stdint.h>
extern motor_info_t damiao[MotorCount];
void All_Init(){

    can1_filter_init();
    can2_fliter_init();
    sbus_remote_control_init();
	   
       Set_dm_enable(&hcan1,0);

    for(int i=0;i<MotorCount;i++){
			
        PID_Struct_Init(&C620[i].Speed_pid, 
            2.0f, 
            0.3f,
            0.0f, 
            10000, 
            16000,
            INIT);
        PID_Struct_Init(&C620[i].Angle_pid,
            10.0f,
            0.0f,
            0.0f,
            300,
            300,
            INIT);

//            //����3508
//              PID_Struct_Init(&C620_hit_angle[i].Speed_pid, 
//            10.5f, 
//            0.0f,
//            0.15f, 
//            30000, 
//            16000,
//            INIT);
//        PID_Struct_Init(&C620_hit_angle[i].Angle_pid,
//            7.0f,
//            0.0f,
//            0.02f,
//            30000,
//            16000,
//            INIT);
//          //  

    }
	damiao[0].KP = 150.0f;//150.0f;
	damiao[0].KD = 0.2f;
	damiao[0].tor = -0.25f;//-1.65
	damiao[0].angle=0.0f;
  C620_up_angle.target_angle = -3420.0f;
        PID_Struct_Init(&damiao[0].Angle_pid,
            15.0f,
            0.0f,
            80.0f,
            10000,
            500,
            INIT);
		damiao[0].Angle_pid.set = 0.0f;
    damiao[0].target_speed=0.0f;
    

   
        PID_Struct_Init(&C620_angle.Speed_pid, 10.0f, 0.3f, 0.0f, 
            10000, 16000, INIT);
		PID_Struct_Init(&C620_up_angle.Angle_pid, 7.0f, 0.0f, 0.02f, 
            20000, 16000, INIT);
		PID_Struct_Init(&C620_up_angle.Speed_pid, 10.5f, 0.0f, 0.3f, 
            20000, 16000, INIT);
    IMU_UART_Init(&huart7);
		
		HAL_Delay(500);
    HeadingHold_Init();
}

void All_Clear(){
    //Set_dm_disable(&hcan1, 0X00);
    damiao[0].KP = 0.0f;//150.0f;
	damiao[0].KD = 0.0f;
	damiao[0].tor = 0.0f;//-1.65
	damiao[0].angle=0.0f;
    for(int i=0;i<MotorCount;i++){
        pid_reset(&C620[i].Speed_pid, 0.0f, 0.0f, 0.0f);
        pid_reset(&C620[i].Angle_pid, 0.0f, 0.0f, 0.0f);
        pid_reset(&damiao[i].Speed_pid, 0.0f, 0.0f, 0.0f);
    }
    pid_reset(&car_pid, 0.0f, 0.0f, 0.0f);
    pid_reset(&C620_angle.Speed_pid, 0.0f, 0.0f, 0.0f);
}
