
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
#include "FSM.h"
#include "imu.h"
#include "heading_hold.h"


// ����״̬����ң����ģ�����ƽ������ﰴ�̶����ڵ���
extern void remote_control_serve_update(void);
extern void remote_control_watchdog_update(void);
extern uint8_t remote_control_is_timeout(void);
extern void remote_control_enter_safe_state(void);

uint16_t PID_Calc_Flag = 0;

/* Manual W axis takeover threshold. car_tarw is already set to 0 in remote deadzone. */
#define CAR_W_MANUAL_DEADBAND 1.0f

/************************ ��ʱ�������жϻص����� ************************/
/**
 * @brief  ��ʱ�������жϻص�������HAL����������д��
 * @note   PID�����߼�д�ڴ˴���ԭ�жϷ�������ҵ�����?
 * @param  htim: ��ʱ�����?
 * @retval ��
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static int16_t voltages[4];
	    if(htim == &htim3)  // ȷ����PID��ʱ���ĸ����ж�
    {

		// ң�س��� 150ms δ����ʱ����������뷢�������ȫ̬
		 remote_control_watchdog_update();
		 if (remote_control_is_timeout())
		 {
		     remote_control_enter_safe_state();
		 }

		 // ң�س��� 150ms δ����ʱ����������뷢�������ȫ̬
//		 remote_control_watchdog_update();
//		 if (remote_control_is_timeout())
//		 {
//		     remote_control_enter_safe_state();
//		 }
      
		 // ÿ 10ms ����һ�η������׶Σ����ڵ��νӹ�ʱ���ƽ�ң�ط���״̬��
		 if (!DebugTune_IsActive())
         {
             remote_control_serve_update();
         }
        remote_control_hit_update();
		//car_x=remote_control_meanum_update(car_x,car_tarx, SPEED_UP_TICKS, SPEED_DOWN_TICKS, MAX_CAR_SPEED);
        //car_y=remote_control_meanum_update(car_y,car_tary, SPEED_UP_TICKS, SPEED_DOWN_TICKS, MAX_CAR_SPEED);

        /*
         * W axis control logic:
         * 1) When remote has W input, disable gyro heading hold and use remote W directly.
         * 2) When remote W input ends, reset heading hold target to current yaw.
         *    This makes the adjusted W angle the new software zero point.
         * 3) Do not call JY901P_Calibrate_SetRef() here; it blocks about 2s and
         *    must not run inside this timer interrupt.
         */
        static uint8_t car_w_manual_last = 0U;
        uint8_t car_w_manual_now = ((car_tarw > CAR_W_MANUAL_DEADBAND) ||
                                    (car_tarw < -CAR_W_MANUAL_DEADBAND)) ? 1U : 0U;

//        if (car_w_manual_now)
//        {
//            if (!car_w_manual_last)
//            {
//                HeadingHold_Enable(0U);
//            }
//            car_w = remote_control_meanum_update(car_w, car_tarw,
//                                                 SPEED_UP_TICKS, SPEED_DOWN_TICKS,
//                                                 MAX_CAR_SPEED);
//        }
//        else
//        {
//            if (car_w_manual_last)
//            {
//                car_w = 0.0f;		
//                HeadingHold_ResetTargetToCurrent();
//                 HeadingHold_Enable(1U);
//            }
//            car_w = HeadingHold_Update(0.0f);
//        }
//        car_w_manual_last = car_w_manual_now;
		car_w = HeadingHold_Update(0.0f);
        MecanumWheel_Move(car_x, car_y, car_w);
		 IMU_GetData(&imu);
		 
		 
        //  Set_dm_mit(&hcan1,0);
//        JY901P_ReadAllData(&gyro_data);//��ȡ����������
//        pid_calc(&car_pid, gyro_data.Gyro_Z-Z_zeropoint, 0); // ������ƽ��ٶ��?
//        car_w=car_pid.out;
         for(int i=0;i<MotorCount;i++)
         {
		 	      pid_calc(&C620[i].Speed_pid,C620[i].Speed_pid.get,C620[i].Speed_pid.set);
                   voltages[i]=(int16_t)C620[i].Speed_pid.out;
			
         }

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
       Set_voltage(&hcan2,voltages);
    }
    
	if(hcan1.ErrorCode!=0)//����can���ߴ���������
	{
		HAL_CAN_DeInit(&hcan1);
		HAL_CAN_Init(&hcan1);
		HAL_CAN_Start(&hcan1);
	}
   if(hcan2.ErrorCode!=0)//����can���ߴ���������
	{
		HAL_CAN_DeInit(&hcan2);
		HAL_CAN_Init(&hcan2);
		HAL_CAN_Start(&hcan2);
	
	
	}	
    //������������ӽǶȻ����жϴ����߼�?
    if(htim == &htim14)  // ȷ����PID��ʱ���ĸ����ж�
    {
    if(DebugTune_IsActive())
    {
	   ops_control();
    }else 
    {
       hit_angle_control();
       up_angle_control();
    }	
	
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
