#include "can.h"
#include "includes.h"
#include "motor_can.h"

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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static int16_t voltages[4];
    if (htim == &htim3)  // 确认是PID定时器的更新中断
    {
        for(int i=0;i<MotorCount;i++)
        {
            pid_calc(&C620[i].Speed_pid,&C620[i].Speed_pid.get,&C620[i].Speed_pid.set);
            Set_voltagec1(&hcan1,(int16_t*)C620[i].Speed_pid.out);
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
