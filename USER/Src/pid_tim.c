
#include "N630.h"
#include "car_ctrl.h"
#include "includes.h"
#include "can.h"
#include "motor_can.h"
#include <stdint.h>
#include "debug_uart.h"
#include "pid.h"
#include "ops.h"
#include "FSM.h"
#include "watch_dog.h"

// 发球状态机在遥控器模块中推进，这里按固定周期调用
extern void remote_control_serve_update(void);
extern void remote_control_hit_update(void);
uint16_t PID_Calc_Flag = 0;
/************************ 定时器更新中断回调函数 ************************/
/**
 * @brief  定时器更新中断回调函数（HAL库弱函数重写）
 * @note   PID控制逻辑写在此处（原中断服务函数的业务代码）
 * @param  htim: 定时器句柄
 * @retval 无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static int16_t voltages[4];
	 static int16_t can1_pitch_output = 0;
	    if(htim == &htim3)  // 确认是PID定时器的更新中断
    {
		 // 遥控超过 设定时间 未更新时，进入底盘与发球机构安全态
		 remote_control_watchdog_update();
		 if (remote_control_is_timeout())
		 {
		     remote_control_enter_safe_state();
		 }
		 else if (!DebugTune_IsActive())
         {
             remote_control_serve_update();
             remote_control_hit_update();
         }
         /* 底盘控制 */
         // 每10ms让速度加/减
         car_x=remote_control_meanum_update(car_x,car_tarx, SPEED_UP_TICKS, SPEED_DOWN_TICKS, MAX_CAR_SPEED);
         car_y=remote_control_meanum_update(car_y,car_tary, SPEED_UP_TICKS, SPEED_DOWN_TICKS, MAX_CAR_SPEED);
        car_w=HeadingHold_Update(0.0f);
        MecanumWheel_Move(car_x, car_y, car_w);
				 IMU_GetData(&imu);
		
         for(int i=0;i<MotorCount;i++)
         {
		 	      pid_calc(&C620[i].Speed_pid,C620[i].Speed_pid.get,C620[i].Speed_pid.set);
                   voltages[i]=(int16_t)C620[i].Speed_pid.out;
			
         }
         //俯仰角3508的速度环控制，位置环控制在 remote_control_hit_update 中被击球状态机调用
		  pid_calc(&C620_angle.Speed_pid,C620_angle.Speed_pid.get,C620_angle.Speed_pid.set);
          can1_pitch_output=(int16_t)C620_angle.Speed_pid.out;
   
        Set_voltage(&hcan2,voltages);
    }
//	if(hcan1.ErrorCode!=0)//避免can总线错误导致死机
//	{
//		HAL_CAN_DeInit(&hcan1);
//		HAL_CAN_Init(&hcan1);
//		HAL_CAN_Start(&hcan1);
//	}
//    if(hcan2.ErrorCode!=0)//避免can总线错误导致死机
//	{
//		HAL_CAN_DeInit(&hcan2);
//		HAL_CAN_Init(&hcan2);
//		HAL_CAN_Start(&hcan2);
//	
//	
//	}	
    //在这里可以添加角度环的中断处理逻辑
    if(htim == &htim14)  // 确认是PID定时器的更新中断
    {
        //调参模式下由自动调参工具接管控制逻辑，正常模式下执行击球角度控制
        if (DebugTune_IsActive())
        {
            ops_control();
        }
        else
        {
            //击球角度控制，位置环控制逻辑在 remote_control_hit_update 中被击球状态机调用，持续控制直到回零完成
            int16_t can1_cmd[4] = {0};
            can1_cmd[0] = can1_pitch_output;
            hit_angle_control(&can1_cmd[1]);
            Set_Voltage_can1(&hcan1, can1_cmd);
        }
    }	
	
    }


/************************ 错误处理函数（可选） ************************/
#ifdef USE_FULL_ASSERT
void Error_Handler(void)
{
    // 可添加LED闪烁、串口打印等错误提示逻辑
    while(1)
    {
    }
}
#endif


