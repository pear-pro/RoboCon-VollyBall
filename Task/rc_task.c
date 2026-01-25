/*  Copyright (s) FOSU
 *  All rights reserved
 * 
 * 文件名称：rc_task.c
 * 摘要：遥控器解析任务，实现SBUS协议解析、遥控数据转换为底盘运动指令，并通过队列下发
 *  
 * 修改历史     版本号        Author       修改内容
 *--------------------------------------------------
 * 2026.01.20      v01         FTW      创建文件
 *--------------------------------------------------
*/
#include "rc_task.h"


/* 任务配置参数 */
#define RC_PARSE_TASK_PRIO    24  // 遥控解析任务优先级：低于底盘PID任务，高于普通任务
#define RC_PARSE_TASK_SIZE    128 // 栈大小

TaskHandle_t rc_parse_task_handle = NULL;		//任务句柄

QueueHandle_t xRCRawDataQueue = NULL;			// 遥控原始数据队列句柄：接收中断中读取的SBUS原始数据


uint8_t   dbus_buf[DBUS_BUFLEN];

//remote control data 
//遥控器控制变量
RC_ctrl_t rc_ctrl;
uint16_t RC_CH_VALUE_OFFS111;//receive data, 18 bytes one frame, but set 36 bytes 

/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_init();
}
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl,Chassis_MoveCmd_t *Cmd)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;//数据偏移
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

    Cmd->vx=normalize_to_range(rc_ctrl->rc.ch[1], -1000.0f, 1000.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
    Cmd->vy=-normalize_to_range(rc_ctrl->rc.ch[0], -1000.0f, 1000.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
    Cmd->wz=-normalize_to_range(rc_ctrl->rc.ch[2], -1000.0f, 1000.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
    
}



/**********************************************************************
 * 函数名称： rc_task
 * 功能描述： 遥控解析任务主函数，阻塞读取原始数据队列→解析协议→下发底盘运动指令
 * 输入参数： parm - 任务函数入参（本任务未使用，FreeRTOS任务函数固定格式）
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期        版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2026/01/20	     V1.0	  FTW	      创建
 **********************************************************************/
void rc_task(void *parm)
{
	uint8_t raw_rc_data[SBUS_RX_BUF_NUM];	// 存储从队列读取的遥控原始数据
	Chassis_MoveCmd_t chasiss_goal;			// 存储解析后的底盘运动指令
    for(;;)
    {
         /* 阻塞等待队列数据（超时10ms，避免死等导致任务卡死） */
        if(xQueueReceive(xRCRawDataQueue, raw_rc_data, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            /* 1. 解析遥控原始数据,转换为底盘运动指令 */
			sbus_to_rc(raw_rc_data, &rc_ctrl,&chasiss_goal);

            /* 2. 发送指令到底盘队列（覆盖旧值，保证指令最新） */
            xQueueOverwrite(xChassisCtrlQueue, &chasiss_goal);
        }

        /* 任务退让，主动放弃CPU，提升系统整体调度效率 */
        taskYIELD();
    }
}
/**********************************************************************
 * 函数名称： rc_task_creat
 * 功能描述： 初始化遥控原始数据队列，并创建遥控解析任务
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期        版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2026/01/20	     V1.0	  FTW	      创建
 **********************************************************************/
void rc_task_creat(void)
{
     /* 创建遥控原始数据队列：项大小为SBUS接收缓冲区大小 */
    xRCRawDataQueue = xQueueCreate(1,  sizeof(uint8_t[SBUS_RX_BUF_NUM]));
    configASSERT(xRCRawDataQueue != NULL); // 断言：创建失败则死机
	
	/*创建任务 -- 自动分配一个任务控制块*/ 
	xTaskCreate((TaskFunction_t) rc_task,          					//任务函数
				(const char * ) "rc_task",		    				//任务名称
				(configSTACK_DEPTH_TYPE) RC_PARSE_TASK_SIZE,     	//堆栈大小（动态内存申请）
				(void * ) NULL,                         			//传送任务函数的参数
				(UBaseType_t) RC_PARSE_TASK_PRIO,       			//优先级
				(TaskHandle_t * ) &rc_parse_task_handle );			//任务句柄
}
