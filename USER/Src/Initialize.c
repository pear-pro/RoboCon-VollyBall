#include "can.h"
#include "includes.h"
#include "motor_can.h"
#include "pid.h"
#include <stdint.h>
#include "imu.h"
#include "heading_hold.h"

extern motor_info_t damiao[MotorCount];
void All_Init(){

    can1_filter_init();
    can2_fliter_init();
    sbus_remote_control_init();
//    JY901P_Unlock();
//    JY901P_Calibrate_Full();


       Set_dm_enable(&hcan1,0);
	    HAL_Delay(5);
       Set_dm_zeropoint(&hcan1,0);
	  HAL_Delay(5);
       Set_dm_enable(&hcan1,1);
	  HAL_Delay(5);
	Set_dm_zeropoint(&hcan1,1);
	  HAL_Delay(5);
       Set_dm_enable(&hcan1,2);
	  HAL_Delay(5);
	Set_dm_zeropoint(&hcan1,2);
	

	 PID_Struct_Init(&C620[0].Speed_pid, //右后
            2.0f, 
            0.3f,
            0.0f, 
            10000, 
            16000,
            INIT);
	 PID_Struct_Init(&C620[1].Speed_pid, //右前
            2.0f, 
            0.3f,
            0.0f, 
            10000, 
            16000,
            INIT);
	PID_Struct_Init(&C620[2].Speed_pid, //左前
            2.0f, 
            0.3f,
            0.0f, 
            10000, 
            16000,
            INIT);
	 PID_Struct_Init(&C620[3].Speed_pid, //左后
            2.0f, 
            0.3f,
            0.0f, 
            10000, 
            16000,
            INIT);
    for(int i=0;i<MotorCount;i++){
			
       
        PID_Struct_Init(&C620[i].Angle_pid,
            10.0f,
            0.0f,
            0.0f,
            300,
            300,
            INIT);

    }
	damiao[0].KP = 200.0f;//150.0f;
	damiao[0].KD = 0.2f;
	damiao[0].tor = -1.65f;//-1.65
	damiao[0].angle=0.0f;
			damiao[1].KP = 200.0f;//150.0f;
	damiao[1].KD = 0.2f;
	damiao[1].tor = -1.65f;//-1.65
	damiao[1].angle=0.0f;
			damiao[2].KP = 200.0f;//150.0f;
	damiao[2].KD = 0.2f;
	damiao[2].tor = -1.65f;//-1.65
	damiao[2].angle=0.0f;
	Pump_Off();
        PID_Struct_Init(&damiao[0].Angle_pid,
            15.0f,
            0.0f,
            80.0f,
            10000,
            500,
            INIT);
		damiao[0].Angle_pid.set = 0.0f;
    damiao[0].target_speed=0.0f;
    

   
	PID_Struct_Init(&C620_angle.Speed_pid, 10.5f, 0.0f, 0.3f, 10000, 16000, INIT);
	PID_Struct_Init(&C620_up_angle[1].Angle_pid, 9.0f, 0.0f, 0.3f, 10000, 16000, INIT); //7 0 0  //8.15 0 0.15
	PID_Struct_Init(&C620_up_angle[1].Speed_pid, 10.5f, 0.0f, 0.3f, 10000, 16000, INIT);//10.5 0 0.3
	C620_up_angle[1].target_angle=20*19.0f;
	
	
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

