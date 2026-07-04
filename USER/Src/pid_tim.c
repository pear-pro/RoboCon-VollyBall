
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

// ����״̬����ң����ģ�����ƽ������ﰴ�̶����ڵ���
extern void remote_control_serve_update(void);
extern void remote_control_watchdog_update(void);
extern uint8_t remote_control_is_timeout(void);
extern void remote_control_enter_safe_state(void);

uint16_t PID_Calc_Flag = 0;
/************************ ��ʱ�������жϻص����� ************************/
/**
 * @brief  ��ʱ�������жϻص�������HAL����������д��
 * @note   PID�����߼�д�ڴ˴���ԭ�жϷ�������ҵ����룩
 * @param  htim: ��ʱ�����
 * @retval ��
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static int16_t voltages[4];
	    if(htim == &htim3)  // ȷ����PID��ʱ���ĸ����ж�
    {
		 // ң�س��� 150ms δ����ʱ����������뷢�������ȫ̬
//		 remote_control_watchdog_update();
//		 if (remote_control_is_timeout())
//		 {
//		     remote_control_enter_safe_state();
//		 }
      
		 // ÿ 10ms ����һ�η������׶Σ����ڵ��νӹ�ʱ���ƽ�ң�ط���״̬��
	//	 if (!DebugTune_IsActive())
         {
             remote_control_serve_update();
         }
		 remote_control_hit_update();
        //  Set_dm_mit(&hcan1,0);
//        JY901P_ReadAllData(&gyro_data);//��ȡ����������
//        pid_calc(&car_pid, gyro_data.Gyro_Z-Z_zeropoint, 0); // ������ƽ��ٶ�Ϊ0
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
    //������������ӽǶȻ����жϴ����߼�
    if(htim == &htim14)  // ȷ����PID��ʱ���ĸ����ж�
    {
    if(DebugTune_IsActive())
    {
	   ops_control();
    }else 
    {
        Set_dm_mit(&hcan1,0);
        Set_dm_mit(&hcan1,1);
        Set_dm_mit(&hcan1,2);
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
