
#include "includes.h"
#include "can.h"
#include "motor_can.h"
#include <stdint.h>
uint16_t PID_Calc_Flag = 0;
/************************ 需根据实际硬件修改的宏定义 ************************/
// 定时器选择（示例：TIM3，根据实际使用的定时器修改）
#define PID_TIMx               TIM3
// 定时器时钟使能宏（示例：TIM3属于APB1总线）
#define PID_TIM_RCC_CLK_ENABLE()  __HAL_RCC_TIM3_CLK_ENABLE()
// 定时器中断号（示例：TIM3全局中断）
#define PID_TIM_IRQn           TIM3_IRQn
// 定时器中断服务函数名（需和启动文件中的中断向量表一致）
#define PID_TIM_IRQHandler     TIM3_IRQHandler

/************************ 全局变量（HAL库定时器句柄） ************************/
TIM_HandleTypeDef htim_pid;  // PID定时器句柄


/************************ 定时器更新中断回调函数 ************************/
/**
 * @brief  定时器更新中断回调函数（HAL库弱函数重写）
 * @note   PID控制逻辑写在此处（原中断服务函数的业务代码）
 * @param  htim: 定时器句柄
 * @retval 无
 */
 
 enum State Set = 0;
 
 uint8_t flag;
 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static int16_t voltages[4];
    if (htim == &htim3)  // 确认是PID定时器的更新中断
    {
//        for(int i=0;i<MotorCount;i++)
//        {
//            voltages[i]=(int16_t)C620[i].Speed_pid.out;
//            
//        }
//        Set_voltagec1(&hcan1,voltages);
		switch(Set)
		{
			case 0:
				
				for(int i=0;i<MotorCount;i++)
				{
					MIT_Calc(&C620[i],1000,0,0);  
				} 
				voltages[0]=C620[0].out;
				voltages[1]=C620[1].out;
				voltages[2]=C620[2].out;
				voltages[3]=C620[3].out;
				Set_voltagec1(&hcan1,voltages);
				
			break;
			
			case 1:
				
				for(int i=0;i<MotorCount;i++)
				{
					MIT_Calc(&C620[i],1000,120,0);  
				} 
				voltages[0]=C620[0].out;
				voltages[1]=C620[1].out;
				voltages[2]=C620[2].out;
				voltages[3]=C620[3].out;
				Set_voltagec1(&hcan1,voltages);
				
				flag = 0;
				
				if(flag == 0 /* && 发球按钮按下 */)
				{
					/* 发球 */
				}
			break;
			
			case 2:
				
				for(int i=0;i<MotorCount;i++)
				{
					MIT_Calc(&C620[i],1000,120,0);  
				} 
				voltages[0]=C620[0].out;
				voltages[1]=C620[1].out;
				voltages[2]=C620[2].out;
				voltages[3]=C620[3].out;
				Set_voltagec1(&hcan1,voltages);
				
			break;
		}

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
    //在这里可以添加角度环的中断处理逻辑
    if(htim == &htim14)  // 确认是PID定时器的更新中断
    {
        for(int i=0;i<MotorCount;i++)
        {
			MIT_Calc(&C620[i],100,160,0);  
        } 
		voltages[0]=C620[0].out;
		voltages[1]=C620[1].out;
		voltages[2]=C620[2].out;
		voltages[3]=C620[3].out;
        //Set_voltagec1(&hcan1,voltages);
		
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
