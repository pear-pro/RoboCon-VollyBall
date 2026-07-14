
#include "N630.h"
#include "car_ctrl.h"
#include "includes.h"
#include "can.h"
#include "jy901p.h"
#include "motor_can.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "debug_uart.h"
#include "pid.h"
#include "ops.h"
#include "FSM.h"
#include "imu.h"
#include "heading_hold.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_pwr_ex.h"


// ����״̬����ң����ģ�����ƽ������ﰴ�̶����ڵ���
extern void remote_control_serve_update(void);
extern void remote_control_watchdog_update(void);
extern uint8_t remote_control_is_timeout(void);
extern void remote_control_enter_safe_state(void);

uint16_t PID_Calc_Flag = 0;

/* Manual W axis takeover threshold. car_tarw is already set to 0 in remote deadzone. */
#define CAR_W_MANUAL_DEADBAND 1.0f

//发球3508最大允许电�??
#define UP_OVER_CURRENT 13000.0f
#define UP_OVER_TEMP 110.0f
#define UP_FEEDBACK_TIMEOUT_TICKS 5


//过流标志位，0表示没保护，1表示过流保护
static volatile uint8_t up_overcurrent_fault = 0;
static volatile uint8_t up_feedback_lost_fault = 0;


// 过流检测函数，返回1表示过流�??表示正常
static uint8_t up_overcurrent_detected(float overcurrent,float overtemp)
{
    if (abs(C620_up_angle[0].Rxmsg.Torque) >= overcurrent || abs(C620_up_angle[1].Rxmsg.Torque) >= overcurrent
       ||C620_up_angle[0].Rxmsg.Temp>= overtemp ||C620_up_angle[1].Rxmsg.Temp>= overtemp)
    {
        return 1;
    }
    return 0;
}

//做保留，�?前A板先上电会冲�?
static void up_feedback_tick_update(void)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        if (C620_up_angle_feedback_tick[i] < 6)
        {
            C620_up_angle_feedback_tick[i]++;
        }
    }
}

static uint8_t up_feedback_lost_detected(void)
{
    if (C620_up_angle_feedback_tick[0] > UP_FEEDBACK_TIMEOUT_TICKS ||
        C620_up_angle_feedback_tick[1] > UP_FEEDBACK_TIMEOUT_TICKS)
    {
        return 1;
    }
    return 0;
}


/************************ ��ʱ�������жϻص����� ************************/
/**
 * @brief  ��ʱ�������жϻص�������HAL����������д��
 * @note   PID�����߼�д�ڴ˴���ԭ�жϷ�������ҵ�����??
 * @param  htim: ��ʱ�����??
 * @retval ��
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static int16_t voltages[4];
	    if(htim == &htim3)  // ȷ����PID��ʱ���ĸ����ж�
    {

		// ң�س��� 150ms δ����ʱ�����������?��������?̬
		 remote_control_watchdog_update();
		 if (remote_control_is_timeout())
		 {
		     remote_control_enter_safe_state();
		 }
		 // ÿ 10ms ����һ�η������׶Σ����ڵ��νӹ�ʱ���ƽ�ң�ط���״̬��
		 if (!DebugTune_IsActive())
         {
             remote_control_serve_update();
         }
        remote_control_hit_update();

		car_w = HeadingHold_Update(0.0f);
        MecanumWheel_Move(car_x, car_y, car_w);
		 IMU_GetData(&imu);
		 

         for(int i=0;i<MotorCount;i++)
         {
		 	      pid_calc(&C620[i].Speed_pid,C620[i].Speed_pid.get,C620[i].Speed_pid.set);
                   voltages[i]=(int16_t)C620[i].Speed_pid.out;
			
         }
 Set_voltage(&hcan2,voltages);
         //vofa调试接口
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
      
    }
    
        if(hcan1.ErrorCode != 0)
        {
            HAL_CAN_DeInit(&hcan1);
            HAL_CAN_Init(&hcan1);
          
        }
        if(hcan2.ErrorCode != 0)
        {
            HAL_CAN_DeInit(&hcan2);
            HAL_CAN_Init(&hcan2);
            can2_fliter_init();
        }
    //������������ӽǶȻ����жϴ����߼�??
    if(htim == &htim14)  // ȷ����PID��ʱ���ĸ����ж�
    {
        up_feedback_tick_update();
         hit_angle_control();
        if (up_overcurrent_detected(UP_OVER_CURRENT,UP_OVER_TEMP))
        {
            up_overcurrent_fault = 1;
        }
        else
        {
            up_overcurrent_fault = 0;
        }

        if (up_feedback_lost_detected())
        {
            up_feedback_lost_fault = 1;
        }
        else
        {
            up_feedback_lost_fault = 0;
        }

        if(up_overcurrent_fault ||  up_feedback_lost_fault)
        {
        pid_reset(&C620_up_angle[0].Angle_pid,0,0,0);
		pid_reset(&C620_up_angle[1].Angle_pid,0,0,0);
        pid_reset(&C620_up_angle[0].Speed_pid,0,0,0);
		pid_reset(&C620_up_angle[1].Speed_pid,0,0,0);
        }
        if(DebugTune_IsActive())
        {
            ops_control();
        }
        else if(serve_is_active())
        {
           
            up_angle_control();
        }
    }
	

/************************ ��������������ѡ�� ************************/
#ifdef USE_FULL_ASSERT
void Error_Handler(void)
{
    // ������LED��˸�����ڴ�ӡ�ȴ�����ʾ�߼�
    while(1)
    {
    }
}
#endif


	}
