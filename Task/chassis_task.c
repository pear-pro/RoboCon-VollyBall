/*  Copyright (s) FOSU
 *  All rights reserved
 * 
 * 文件名称：chassis_task.c
 * 摘要：底盘任务
 *  
 * 修改历史     版本号        Author       修改内容
 *--------------------------------------------------
 * 2026.01.19      v01        FTW      创建文件
 *--------------------------------------------------
*/
#include "chassis_task.h"

/* 任务配置参数 */
#define CHASSIS_TASK_PRIO 26		// 任务优先级
#define CHASSIS_TASK_SIZE 128		// 128 * 4 Byte = 128 * 32 Bit（堆栈）
#define CHASSIS_TASK_PERIOD_MS 1UL  // 调度周期设置：1ms
#define CHASSIS_TASK_PERIOD_TICKS pdMS_TO_TICKS(CHASSIS_TASK_PERIOD_MS) 

TaskHandle_t chassis_Task_Handle;	//任务句柄

/* 队列句柄定义（FreeRTOS进程间通信） */
QueueHandle_t xChassisCtrlQueue;		// 底盘控制指令队列：接收遥控器/上位机的运动指令（vx/vy/wz）
QueueHandle_t xChassisFeedbackQueue;	// 电机反馈队列：接收CAN中断中解析的电机状态（转速、电流等）


/**********************************************************************
 * 函数名称： chassis_task
 * 功能描述： 底盘控制核心任务，1ms周期调度，完成"指令接收→反馈读取→PID计算→CAN发电压"闭环
 * 输入参数： parm - 任务函数入参（本任务未使用，FreeRTOS任务函数固定格式）
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期        版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2026/01/20	     V1.0	  FTW	      创建
 **********************************************************************/
void chassis_task(void *parm)
{
	/* 上次唤醒时间变量：用于vTaskDelayUntil实现精准周期调度（必须初始化） */
	TickType_t xLastWakeTime = xTaskGetTickCount();
		
	RxMsg_t new_feedback[4];					// 临时存储从队列读取的4个电机反馈数据
	Chassis_MoveCmd_t curr_chassis_cmd;			// 临时存储底盘运动指令（vx:前后, vy:左右, wz:旋转）
	static int16_t voltages[4];					// 电机目标电压数组（static保证循环中值不丢失）
   	int16_t l;
	for(;;)
	{
		l=l+1;
		if(l>100)
		{
			HAL_GPIO_TogglePin(led3_GPIO_Port,led3_Pin);
			
			l=0;
		}
		
		/* 1. 读取底盘控制指令队列（非阻塞：超时0，无新指令则沿用上次指令） */
        if(xQueueReceive(xChassisCtrlQueue, &curr_chassis_cmd, 0) == pdTRUE)
        {
			/* 根据新指令更新电机目标转速（麦克纳姆轮运动学解算） */
            MecanumWheel_Move(curr_chassis_cmd.vx,curr_chassis_cmd.vy,curr_chassis_cmd.wz,C620); 
        }

		
		/* 2. 读取电机反馈队列（非阻塞：超时0，无新反馈则沿用上次数据） */
        if(xQueueReceive(xChassisFeedbackQueue, &new_feedback, 0) == pdTRUE) 
		{

            for(int i=0;i<MotorCount;i++)
			{
				C620[i].Rxmsg=new_feedback[i];				
				C620[i].Speed_pid.get=C620[i].Rxmsg.Speed;
				
			}                      
        }
		 
		/* 3. 电机转速PID闭环计算 */
		for(int i=0;i<MotorCount;i++)
		{
			pid_calc(&C620[i].Speed_pid,C620[i].Speed_pid.get,C620[i].Speed_pid.set);
			voltages[i]=(int16_t)C620[i].Speed_pid.out;
			
		}
		/* 4. 通过CAN1发送电机目标电压，驱动M3508电机 */
		Set_voltagec1(&hcan1,voltages);
		
		
		/*延迟并更新上次被唤醒的系统节拍数*/
		vTaskDelayUntil(&xLastWakeTime, (TickType_t)CHASSIS_TASK_PERIOD_TICKS);
	}
}
/**********************************************************************
 * 函数名称： chassis_task_creat
 * 功能描述： 初始化底盘控制相关队列，并创建底盘控制任务
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期        版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2026/01/20	     V1.0	  FTW	      创建
 **********************************************************************/
void chassis_task_creat(void) 
{

	// 1. 创建目标指令队列：长度1（最新指令覆盖旧指令），项大小=MotorCtrlCmd_t
    xChassisCtrlQueue = xQueueCreate(1, sizeof(Chassis_MoveCmd_t));
    configASSERT(xChassisCtrlQueue != NULL); // 队列创建失败则断言
    
    // 2. 创建电机反馈队列：长度1（最新指令覆盖旧指令），项大小为4个RxMsg_t结构体（对应4个电机）
    xChassisFeedbackQueue = xQueueCreate(1, sizeof(RxMsg_t[4]));
    configASSERT(xChassisFeedbackQueue != NULL);
	
	
	/*创建任务 -- 自动分配一个任务控制块*/ 
	xTaskCreate((TaskFunction_t) chassis_task,          		//任务函数
				(const char * ) "chassis_task",		    		//任务名称
				(configSTACK_DEPTH_TYPE) CHASSIS_TASK_SIZE,     //堆栈大小（动态内存申请）
				(void * ) NULL,                         		//传送任务函数的参数
				(UBaseType_t) CHASSIS_TASK_PRIO,       			//优先级
				(TaskHandle_t * ) &chassis_Task_Handle );		//任务句柄
	

}

