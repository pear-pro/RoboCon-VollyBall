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

/************************ 定时器中断初始化函数 ************************/
/**
 * @brief  PID控制定时器中断初始化（HAL库版本）
 * @param  arr: 自动重装值 (TIM_Period)
 * @param  psc: 预分频系数 (TIM_Prescaler)
 * @retval 无
 */
void PID_TIM_Init(uint16_t arr, uint16_t psc)
{
    // 1. 定时器时钟使能
    PID_TIM_RCC_CLK_ENABLE();

    // 2. 配置定时器时基参数
    htim_pid.Instance = PID_TIMx;                     // 定时器实例
    htim_pid.Init.Prescaler = psc;                    // 预分频系数
    htim_pid.Init.CounterMode = TIM_COUNTERMODE_UP;   // 向上计数模式
    htim_pid.Init.Period = arr;                       // 自动重装值
    htim_pid.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  // 时钟分频因子
    htim_pid.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  // 禁用ARR预装载
    if (HAL_TIM_Base_Init(&htim_pid) != HAL_OK)       // 初始化定时器基础模式
    {
        Error_Handler();  // 初始化失败处理（需自行实现或替换为断言）
    }

    // 3. 使能定时器更新中断（Update Interrupt）
    __HAL_TIM_ENABLE_IT(&htim_pid, TIM_IT_UPDATE);

    // 4. 配置中断优先级（抢占优先级2，子优先级0）
    // 注意：优先级分组需在main函数中提前配置（如NVIC_PRIORITYGROUP_2）
    HAL_NVIC_SetPriority(PID_TIM_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(PID_TIM_IRQn);

    // 5. 启动定时器（向上计数模式）
    if (HAL_TIM_Base_Start(&htim_pid) != HAL_OK)
    {
        Error_Handler();  // 启动失败处理
    }
}


void PID_TIM_IRQHandler(void)
{
    // 1. 手动校验更新中断标志（防止虚假中断）
    if (__HAL_TIM_GET_FLAG(&htim_pid, TIM_FLAG_UPDATE) != RESET)
    {
        // 2. 手动清除更新中断标志（HAL库也会清，但双重保障）
        __HAL_TIM_CLEAR_FLAG(&htim_pid, TIM_FLAG_UPDATE);
		//计算完成标志位
        PID_Calc_Flag = 1;
        // 3. HAL库中断公共处理（触发回调函数HAL_TIM_PeriodElapsedCallback）
        HAL_TIM_IRQHandler(&htim_pid);
    }
}
/************************ 定时器更新中断回调函数 ************************/
/**
 * @brief  定时器更新中断回调函数（HAL库弱函数重写）
 * @note   PID控制逻辑写在此处（原中断服务函数的业务代码）
 * @param  htim: 定时器句柄
 * @retval 无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == PID_TIMx)  // 确认是PID定时器的更新中断
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
