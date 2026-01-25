#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"

#include "includes.h"
#include "motor_can.h"
#include <stdint.h>

void chassis_task_creat(void);

extern QueueHandle_t xChassisCtrlQueue;		// 目标指令队列（遥控器）
extern QueueHandle_t xChassisFeedbackQueue;	// 电机反馈队列（CAN中断）

#endif /* __CHASSIS_TASK_H__ */
