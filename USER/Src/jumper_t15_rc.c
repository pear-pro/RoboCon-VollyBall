#include "jumper_t15_rc.h"
#include "FSM.h"
#include "includes.h"
#include "main.h"
#include "watch_dog.h"
#include "heading_hold.h"
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t   sbus_buffer[SBUS_BUFLEN];//SBUS���ջ���??

static void sbus_to_remote_control(volatile const uint8_t *sbus_buffer, SBUS_ctrl_t *sbus_ctrl);
static uint8_t sbus_rx_buffer[2][SBUS_RX_BUF_NUM];//DMA˫��??

SBUS_ctrl_t sbus_ctrl;

uint8_t count_flag=0;
static volatile uint8_t count=0;

void sbus_remote_control_init(void)//SBUSң������ʼ��
{
    RC_init(sbus_rx_buffer[0], sbus_rx_buffer[1], SBUS_RX_BUF_NUM);

    
    sbus_ctrl.last_swa_state = POS_UP;  // SWA��???��״̬UP
    sbus_ctrl.last_swb_state = POS_UP;        // SWB��???��״̬UP
    sbus_ctrl.last_swc_state = POS_MID;        // SWC��???��״̬MID
    sbus_ctrl.last_swd_state = POS_MID;  // SWD��???��״̬MID
    sbus_ctrl.last_se_state = POS_UP;  // SE��???��״̬UP
    sbus_ctrl.last_sf_state = POS_UP;  // SF��???��״̬UP
    sbus_ctrl.key_flag = KEY_NONE;       // ��???����־λNONE

}

const SBUS_ctrl_t *get_sbus_remote_control_point(void)//��ȡSBUSң����ָ??
{
    return &sbus_ctrl;
}

//void remote_control_watchdog_update(void)
//{
//    // ���Ź�???����������ֵ����Ϊң����ʧ??
//    if (rc_watchdog_tick < RC_WATCHDOG_TIMEOUT_TICKS)
//    {
//        rc_watchdog_tick++;
//    }

//    // ������ֵ������ʧ��״??
//    if (rc_watchdog_tick >= RC_WATCHDOG_TIMEOUT_TICKS)
//    {
//        rc_watchdog_timeout = 1;
//    }
//}

//uint8_t remote_control_is_timeout(void)
//{
//    return rc_watchdog_timeout;
//}   

//void remote_control_enter_safe_state(void)
//{
//    // ����ͣ???
//    car_x = 0.0f;
//    car_y = 0.0f;
//    car_w = 0.0f;
//    C620_angle.Speed_pid.set = 0.0f;

//    // �����������
//    serve_active = 0;
//    serve_armed = 1;
//    serve_tick = 0;
//    serve_stage = SERVE_STAGE_IDLE;

//    // ��???�����??
//    //damiao[0].angle = 0.0f;
//    //damiao[1].angle = -0.5f;

//    // ������ز�����λ
//    count = 0;
//    returning = 0;
//    return_tick = 0;
//    return_start_angle = 0.0f;
//    damiao0_tarangle = 0.0f;
//}

 float remote_control_meanum_update(float input,float target,float up_ticks,float down_ticks,float max_speed)
 {
     float delat = target - input;
     if (fabsf(delat) < 20.0f) {
         return target; // �Ѿ��ǳ��ӽ�??��ֵ��ֱ�ӷ���??��??
     }
     if (delat > 0) {
         // ��Ҫ��??
         float step = max_speed / up_ticks; // ÿ�����ڵļ���???��
         return input + fminf(step, delat); // ��???����Ŀ��??
     } else {
         // ��Ҫ��??
         float step = max_speed / down_ticks; // ÿ�����ڵļ���???��
         return input + fmaxf(-step, delat); // ��???����Ŀ��??
     }
 }

//����????
void USART1_IRQHandlerCallBack(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//���յ���??
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)//����????
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //�趨����??1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_remote_control(sbus_rx_buffer[0], &sbus_ctrl);
				remote_control_watchdog_feed();
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨����??0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң������??
                sbus_to_remote_control(sbus_rx_buffer[1], &sbus_ctrl);
				remote_control_watchdog_feed();
            }
        }
    }
}

static uint8_t detect_switch_position(int16_t value)
{
    if (value <= SWITCH_SBUS_CH_VALUE_MIN + DEADZONE)
    {
        return POS_UP;  // ??
    } 
    else if (value >= SWITCH_SBUS_CH_VALUE_MAX - DEADZONE) 
    {
        return POS_DOWN;  // ??
    } 
    else 
    {
        return POS_MID;  // ??
    }
}

static void virtual_key_update(SBUS_ctrl_t *sbus_ctrl)
{
    // ���¿���״??
    uint8_t SWA_pos = detect_switch_position(sbus_ctrl -> ch[4]);
    uint8_t SWB_pos = detect_switch_position(sbus_ctrl -> ch[5]);
    uint8_t SWC_pos = detect_switch_position(sbus_ctrl -> ch[6]);
    uint8_t SWD_pos = detect_switch_position(sbus_ctrl -> ch[7]);
    uint8_t SE_pos = detect_switch_position(sbus_ctrl -> ch[8]);
    uint8_t SF_pos = detect_switch_position(sbus_ctrl -> ch[9]);

    //��???������??λ״??
    sbus_ctrl->key_flag = KEY_NONE;

    //SWA����
if(sbus_ctrl -> last_swa_state != SWA_pos)
   {
       if(SWA_pos == POS_UP)
       {
           sbus_ctrl -> key_flag |= KEY_SWA_UP;
       }
       else if(SWA_pos == POS_DOWN)
       {
           sbus_ctrl -> key_flag |= KEY_SWA_DOWN;
       }
       else if(SWA_pos == POS_MID)
       {
           sbus_ctrl->key_flag |= KEY_SWA_MID; 
       }
   }
   else
   {
   // ״̬����ʱ���ö�Ӧλ�õı�־
       if(SWA_pos == POS_UP)
       {
           sbus_ctrl->key_flag |= KEY_SWA_UP;
       }
       else if(SWA_pos == POS_MID)
       {
           sbus_ctrl->key_flag |= KEY_SWA_MID;
       }
       else if(SWA_pos == POS_DOWN)
       {
           sbus_ctrl->key_flag |= KEY_SWA_DOWN;
       }
   }

   //SWB����
   if(sbus_ctrl -> last_swb_state != SWB_pos)
   {
       if(SWB_pos == POS_UP)
       {
           sbus_ctrl -> key_flag |= KEY_SWB_UP;
       }
       else if(SWB_pos == POS_DOWN)
       {
           sbus_ctrl -> key_flag |= KEY_SWB_DOWN;
       }
   }
   else
   {
   // ״̬����ʱ���ö�Ӧλ�õı�־
        if(SWC_pos == POS_UP)
        {
            sbus_ctrl->key_flag |= KEY_SWC_UP;
        }
        else if(SWC_pos == POS_MID)
        {
            sbus_ctrl->key_flag |= KEY_SWC_MID;
        }
        else if(SWC_pos == POS_DOWN)
        {
            sbus_ctrl->key_flag |= KEY_SWC_DOWN;
        }
    }

    //SWD����
    if(sbus_ctrl -> last_swd_state != SWD_pos)
    {
        if(SWD_pos == POS_UP)
        {
            sbus_ctrl -> key_flag |= KEY_SWD_UP;
        }
        else if(SWD_pos == POS_DOWN)
        {
            sbus_ctrl -> key_flag |= KEY_SWD_DOWN;
        }
    }
    else
    {
    // ״̬����ʱ���ö�Ӧλ�õı�??
        if(SWD_pos == POS_UP)
        {
            sbus_ctrl->key_flag |= KEY_SWD_UP;
        }
        else if(SWD_pos == POS_MID)
        {
            sbus_ctrl->key_flag |= KEY_SWD_MID;
        }
        else if(SWD_pos == POS_DOWN)
        {
            sbus_ctrl->key_flag |= KEY_SWD_DOWN;
        }
    }

    //SE����
    if(sbus_ctrl -> last_se_state == POS_UP && SE_pos == POS_DOWN)
    {
        sbus_ctrl->key_flag |= KEY_SE_DOWN;
    }
    else if(sbus_ctrl -> last_se_state == POS_DOWN && SE_pos == POS_UP)
    {
        sbus_ctrl->key_flag |= KEY_SE_UP;
    }
    else
    {
        // ����״̬ʱҲ???�ö�Ӧ״??
        if(SE_pos == POS_UP)
        {
            sbus_ctrl->key_flag |= KEY_SE_UP;
        }
        else if(SE_pos == POS_DOWN)
        {
            sbus_ctrl->key_flag |= KEY_SE_DOWN;
        }
    }

    //SF����
    if(sbus_ctrl -> last_sf_state == POS_UP && SF_pos == POS_DOWN)
    {
        sbus_ctrl->key_flag |= KEY_SF_PRESSED;
    }

    //����״??
    sbus_ctrl->last_swa_state = SWA_pos;
    sbus_ctrl->last_swb_state = SWB_pos;
    sbus_ctrl->last_swc_state = SWC_pos;
    sbus_ctrl->last_swd_state = SWD_pos;
    sbus_ctrl->last_se_state = SE_pos;
    sbus_ctrl->last_sf_state = SF_pos;

}

//void Pump_On(void)
//{
//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);//PE5���� �͵�ƽ����
//}

//void Pump_Off(void)
//{
//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
//}


static void sbus_to_remote_control(volatile const uint8_t *sbus_buffer, SBUS_ctrl_t *sbus_ctrl)
{
    if((sbus_buffer [0] == 0x0f) && (sbus_buffer[24] == 0x00))//�ж�ͷ֡��β??
    {
        sbus_ctrl -> ch[0] = ((sbus_buffer[1] )| (sbus_buffer[2] << 8 )) & 0x07ff;//����??
        sbus_ctrl -> ch[1] = ((sbus_buffer[2] >> 3 )| (sbus_buffer[3] << 5 )) & 0x07ff;//����??
        sbus_ctrl -> ch[2] = ((sbus_buffer[3] >> 6 )| (sbus_buffer[4] << 2 ) | (sbus_buffer[5] << 10)) & 0x07ff;//����??
        sbus_ctrl -> ch[3] = ((sbus_buffer[5] >> 1 )| (sbus_buffer[6] << 7 )) & 0x07ff;//����??
        sbus_ctrl -> ch[4] = ((sbus_buffer[6] >> 4 )| (sbus_buffer[7] << 4 )) & 0x07ff;//SWA
        sbus_ctrl -> ch[5] = ((sbus_buffer[7] >> 7 )| (sbus_buffer[8] << 1 )| (sbus_buffer[9] << 9 )) & 0x07ff;//SWB
        sbus_ctrl -> ch[6] = ((sbus_buffer[9] >> 2 )| (sbus_buffer[10] << 6 )) & 0x07ff;//SWC
        sbus_ctrl-> ch[7] = ((sbus_buffer[10] >> 5 )| (sbus_buffer[11] << 3 )) & 0x07ff;//SWD
        sbus_ctrl ->ch[8] = ((sbus_buffer[12])| (sbus_buffer[13] << 8 )) & 0x07ff;//SE
        sbus_ctrl ->ch[9] = ((sbus_buffer[13] >> 3 )| (sbus_buffer[14] << 5 )) & 0x07ff;//SF

       //����ƫ��
        for(int i = 0;i<10;i++)
        {
            sbus_ctrl->ch[i] = (int16_t)(sbus_ctrl->ch[i] - SBUS_CH_VALUE_OFFSET);
        }
       
        // ��������??λ״??
        virtual_key_update(sbus_ctrl);

        // //��һ����??
//        car_x=normalize_to_range((float)sbus_ctrl -> ch[3], -800.0f, 800.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
//        car_y=-normalize_to_range((float)sbus_ctrl -> ch[1], -800.0f, 800.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
       // car_w=-normalize_to_range((float)sbus_ctrl -> ch[3], -800.0f, 800.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
        
        // //Ӧ������
        // car_x=apply_deadzone(car_x, DEADZONE);
        // car_y=apply_deadzone(car_y, DEADZONE);
        // car_w=apply_deadzone(car_w, DEADZONE);

        if (sbus_ctrl->ch[0] < 100 && sbus_ctrl->ch[0] > -100) {
        car_tarx = 0;
        }
        else {
            car_tarx=normalize_to_range(sbus_ctrl->ch[0], -800.0f, 800.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);  
            //car_x=low_pass(car_tarx, car_x, 0.25);

        }
        if (sbus_ctrl->ch[1] < 100 && sbus_ctrl->ch[1] > -100) {
            car_tary = 0;
        }
        else {
            car_tary=-normalize_to_range(sbus_ctrl->ch[1], -800.0f, 800.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
            //car_y=low_pass(car_tary, car_y, 0.32);
        }
		if (sbus_ctrl->ch[2] < 100 && sbus_ctrl->ch[2] > -100) {
        }
		else {
			heading_hold.target_yaw_deg -= normalize_to_range(sbus_ctrl->ch[2], -800.0f, 800.0f, -1, 1)*0.1;
		}
		
//         MecanumWheel_Move(car_tarx,car_tary,car_tarw);

        //ģʽ�л�
// SWC 三档选择三组击球角度预设
       if (KEY_SWC_UP & sbus_ctrl->key_flag)
       {
           hit_set_preset(0);
       }
       else if (KEY_SWC_MID & sbus_ctrl->key_flag)
       {
           hit_set_preset(1);
       }
       else if (KEY_SWC_DOWN & sbus_ctrl->key_flag)
       {
           hit_set_preset(2);
       }
         // �뿪�µ�������???��һ�η��򴥷���??
//        if (!(KEY_SWD_DOWN & sbus_ctrl->key_flag))
//		{
//			serve_arm();
//		}

//        if (KEY_SWD_UP & sbus_ctrl -> key_flag)
//        {
//            // �ϵ�������ԭ??�Ƕȵ������
//            C620_up_angle.Speed_pid.set = 25000 * (sbus_ctrl->ch[3] / 800.0f);
//        }
//        else if (KEY_SWD_MID & sbus_ctrl -> key_flag)
//        {  
//           //
//        }
//        else
//        {
//           if(serve_mode == SERVE_MODE_ANGLE)
//            {
//                serve_request_start();
//            }
//            
//  
//        }
        
//        if(serve_mode == SERVE_MODE_SPEED)
//        {
//            if(KEY_SE_DOWN & sbus_ctrl -> key_flag)
//            {
//                C620_up_angle.target_speed = 6000.0f;
//            }
//            else if(KEY_SE_UP & sbus_ctrl -> key_flag)
//            {
//                C620_up_angle.target_speed = 0;
//            }
//        }

        // SA ����ѡ�����Ƕ�Ԥ��
// SF 按下击球，松手回零
       if (detect_switch_position(sbus_ctrl->ch[9]) == POS_DOWN)
       {
           hit_request_press();
       }
       else
       {
           hit_request_release();
       }

       // 离开下档后重新装填一次发球触发资格
       if (!(KEY_SWB_UP & sbus_ctrl->key_flag))
       {
           serve_arm();
       }

       if (KEY_SWD_DOWN & sbus_ctrl->key_flag)
       {
           // 上档：保留原有 C620 角度电机控制
           C620_angle.Speed_pid.set = 25000.0f;
       }
       else if (KEY_SWB_MID & sbus_ctrl->key_flag)
       {
           // 中档：击球机构由 SF 单独控制
       }
       else
       {
           // 下档：触发一次自动发球流程
           serve_request_start();

           if (!serve_is_active())
           {
               damiao[0].angle = 0.0f;
               damiao[1].angle = 0.0f;
           }
       }
       }
}

