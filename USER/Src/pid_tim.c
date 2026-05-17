
#include "N630.h"
#include "car_ctrl.h"
#include "includes.h"
#include "can.h"
#include "jy901p.h"
#include "motor_can.h"
#include <stdint.h>
#include "debug_uart.h"
#include "pid.h"
#include "ops.h"

// 发球状态机在遥控器模块中推进，这里按固定周期调用
extern void remote_control_serve_update(void);
extern void remote_control_watchdog_update(void);
extern uint8_t remote_control_is_timeout(void);
extern void remote_control_enter_safe_state(void);

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
	    if(htim == &htim3)  // 确认是PID定时器的更新中断
    {
			 //damiao0_angle_update();
		 // 遥控超过 150ms 未更新时，进入底盘与发球机构安全态
//		 remote_control_watchdog_update();
//         // 达妙电机角度回零
//        
//		 if (remote_control_is_timeout())
//		 {
//		     remote_control_enter_safe_state();
//		 }
      
		 // 每 10ms 更新一次发球动作阶段，串口调参接管时不推进遥控发球状态机
		 if (!DebugTune_IsActive())
         {
             remote_control_serve_update();
         }
         Set_dm_mit(&hcan1,0);
         // 每10ms让速度加/减
         car_x=remote_control_meanum_update(car_x,car_tarx, SPEED_UP_TICKS, SPEED_DOWN_TICKS, MAX_CAR_SPEED);
         car_y=remote_control_meanum_update(car_y,car_tary, SPEED_UP_TICKS, SPEED_DOWN_TICKS, MAX_CAR_SPEED);
        car_w=remote_control_meanum_update(car_w,car_tarw, SPEED_UP_TICKS, SPEED_DOWN_TICKS, MAX_CAR_SPEED);
        MecanumWheel_Move(car_x, car_y, car_w);
		
//        JY901P_ReadAllData(&gyro_data);//读取陀螺仪数据
//        pid_calc(&car_pid, gyro_data.Gyro_Z-Z_zeropoint, 0); // 假设控制角速度为0
//        car_w=car_pid.out;
        // for(int i=0;i<MotorCount;i++)
        // {
		// 	      pid_calc(&C620[i].Speed_pid,C620[i].Speed_pid.get,C620[i].Speed_pid.set);
        //           voltages[i]=(int16_t)C620[i].Speed_pid.out;
			
        // }
//					pid_calc(&C620_angle.Speed_pid,C620_angle.Speed_pid.get,C620_angle.Speed_pid.set);
//          voltage_angle[0]=(int16_t)C620_angle.Speed_pid.out;
//          Set_voltage_angle(&hcan2,voltage_angle);
//        float num[]={//gyro_data.Gyro_X,
//            gyro_data.Gyro_Y,
//            gyro_data.Gyro_Z,
//            gyro_data.Acc_X,
//            gyro_data.Acc_Y,
//            gyro_data.Acc_Z,
//            gyro_data.Angle_X,
//            gyro_data.Angle_Y,
//            gyro_data.Angle_Z
//        };
////        Vofa_JustFloat(num, 3);
       // Set_voltage(&hcan2,voltages);
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
	   ops_control();
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
