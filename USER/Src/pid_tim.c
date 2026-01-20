
#include "includes.h"
#include "can.h"
#include "motor_can.h"
#include <stdint.h>
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
        for(int i=0;i<MotorCount;i++)
        {
            pid_calc_Speed(&C620[i].Speed_pid,C620[i].Speed_pid.get,C620[i].Speed_pid.set);
            voltages[i]=(int16_t)C620[i].Speed_pid.out;
        }
        Set_voltagec1(&hcan1,voltages);
    }
	
    //在这里可以添加角度环的中断处理逻辑
    if(htim == &htim14)  // 确认是PID定时器的更新中断
    {
        for(int i=0;i<MotorCount;i++)
        {
            pid_calc_Angle(&damiao[i].Angel_pid,damiao[i].Angel_pid.get,damiao[i].Angel_pid.set);
            pid_calc_Speed(&damiao[i].Speed_pid,damiao[i].Speed_pid.get,damiao[i].Angel_pid.out);
            voltages[i]=(int16_t)damiao[i].Speed_pid.out;
        }
        Set_dm(&hcan2,voltages);
    }
	if(hcan1.ErrorCode!=0)//避免can总线错误导致死机
	{
		HAL_CAN_DeInit(&hcan1);
		HAL_CAN_Init(&hcan1);
		HAL_CAN_Start(&hcan1);
	
	
	}
    if(hcan2.ErrorCode!=0)//避免can总线错误导致死机
	{
		HAL_CAN_DeInit(&hcan2);
		HAL_CAN_Init(&hcan2);
		HAL_CAN_Start(&hcan2);
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
