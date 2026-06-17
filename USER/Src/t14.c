#include "t14.h"
#include "FSM.h"
#include "includes.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t   sbus_buffer[SBUS_BUFLEN];//SBUS接收缓冲�?

static void sbus_to_remote_control(volatile const uint8_t *sbus_buffer, SBUS_ctrl_t *sbus_ctrl);
static uint8_t sbus_rx_buffer[2][SBUS_RX_BUF_NUM];//DMA双缓�?

SBUS_ctrl_t sbus_ctrl;

uint8_t count_flag=0;

void sbus_remote_control_init(void)//SBUS遥控器初始化
{
    RC_init(sbus_rx_buffer[0], sbus_rx_buffer[1], SBUS_RX_BUF_NUM);

    
    sbus_ctrl.last_swa_state = POS_UP;  // SWA初�?�化状态UP
    sbus_ctrl.last_swb_state = POS_UP;        // SWB初�?�化状态UP
    sbus_ctrl.last_swc_state = POS_MID;        // SWC初�?�化状态MID
    sbus_ctrl.last_swd_state = POS_MID;  // SWD初�?�化状态MID
    sbus_ctrl.last_se_state = POS_UP;  // SE初�?�化状态UP
    sbus_ctrl.last_sf_state = POS_UP;  // SF初�?�化状态UP
    sbus_ctrl.key_flag = KEY_NONE;       // 初�?�化标志位NONE

}

const SBUS_ctrl_t *get_sbus_remote_control_point(void)//获取SBUS遥控器指�?
{
    return &sbus_ctrl;
}

//void remote_control_watchdog_update(void)
//{
//    // 看门狗�?�数，超过阈值则认为遥控器失�?
//    if (rc_watchdog_tick < RC_WATCHDOG_TIMEOUT_TICKS)
//    {
//        rc_watchdog_tick++;
//    }

//    // 超过阈值，进入失联状�?
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
//    // 底盘停�??
//    car_x = 0.0f;
//    car_y = 0.0f;
//    car_w = 0.0f;
//    C620_angle.Speed_pid.set = 0.0f;

//    // 发球机构回零
//    serve_active = 0;
//    serve_armed = 1;
//    serve_tick = 0;
//    serve_stage = SERVE_STAGE_IDLE;

//    // 达�?�电机回�?
//    //damiao[0].angle = 0.0f;
//    //damiao[1].angle = -0.5f;

//    // 回零相关参数复位
//    count = 0;
//    returning = 0;
//    return_tick = 0;
//    return_start_angle = 0.0f;
//    damiao0_tarangle = 0.0f;
//}

// float remote_control_meanum_update(float input,float target,float up_ticks,float down_ticks,float max_speed)
// {
//     float delat = target - input;
//     if (fabsf(delat) < 20.0f) {
//         return target; // 已经非常接近�?标值，直接返回�?标�?
//     }
//     if (delat > 0) {
//         // 需要加�?
//         float step = max_speed / up_ticks; // 每个周期的加速�?�长
//         return input + fminf(step, delat); // 不�?�超过目标�?
//     } else {
//         // 需要减�?
//         float step = max_speed / down_ticks; // 每个周期的减速�?�长
//         return input + fmaxf(-step, delat); // 不�?�超过目标�?
//     }
// }

//串口�?�?
void USART1_IRQHandlerCallBack(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数�?
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)//空闲�?�?
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲�?1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_remote_control(sbus_rx_buffer[0], &sbus_ctrl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲�?0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数�?
                sbus_to_remote_control(sbus_rx_buffer[1], &sbus_ctrl);
            }
        }
    }
}

static uint8_t detect_switch_position(int16_t value)
{
    if (value <= SWITCH_SBUS_CH_VALUE_MIN + DEADZONE)
    {
        return POS_UP;  // �?
    } 
    else if (value >= SWITCH_SBUS_CH_VALUE_MAX - DEADZONE) 
    {
        return POS_DOWN;  // �?
    } 
    else 
    {
        return POS_MID;  // �?
    }
}

static void virtual_key_update(SBUS_ctrl_t *sbus_ctrl)
{
    // 更新开关状�?
    uint8_t SWA_pos = detect_switch_position(sbus_ctrl -> ch[4]);
    uint8_t SWB_pos = detect_switch_position(sbus_ctrl -> ch[5]);
    uint8_t SWC_pos = detect_switch_position(sbus_ctrl -> ch[6]);
    uint8_t SWD_pos = detect_switch_position(sbus_ctrl -> ch[7]);
    uint8_t SE_pos = detect_switch_position(sbus_ctrl -> ch[8]);
    uint8_t SF_pos = detect_switch_position(sbus_ctrl -> ch[9]);

    //初�?�化虚拟�?位状�?
    sbus_ctrl->key_flag = KEY_NONE;

    //SWA处理
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
    }
    else
    {
    // 状态不变时设置对应位置的标�?
        if(SWA_pos == POS_UP)
        {
            sbus_ctrl->key_flag |= KEY_SWA_UP;
        }
        else if(SWA_pos == POS_DOWN)
        {
            sbus_ctrl->key_flag |= KEY_SWA_DOWN;
        }
    }

    //SWB处理
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
    // 状态不变时设置对应位置的标�?
        if(SWB_pos == POS_UP)
        {
            sbus_ctrl->key_flag |= KEY_SWB_UP;
        }
        else if(SWB_pos == POS_DOWN)
        {
            sbus_ctrl->key_flag |= KEY_SWB_DOWN;
        }
    }

    //SWC处理
    if(sbus_ctrl -> last_swc_state != SWC_pos)
    {
        if(SWC_pos == POS_UP)
        {
            sbus_ctrl -> key_flag |= KEY_SWC_UP;
        }
        else if(SWC_pos == POS_DOWN)
        {
            sbus_ctrl -> key_flag |= KEY_SWC_DOWN;
        }
    }
    else
    {
    // 状态不变时设置对应位置的标�?
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

    //SWD处理
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
    // 状态不变时设置对应位置的标�?
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

    //SE处理
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
        // 保持状态时也�?�置对应状�?
        if(SE_pos == POS_UP)
        {
            sbus_ctrl->key_flag |= KEY_SE_UP;
        }
        else if(SE_pos == POS_DOWN)
        {
            sbus_ctrl->key_flag |= KEY_SE_DOWN;
        }
    }

    //SF处理
    if(sbus_ctrl -> last_sf_state == POS_UP && SF_pos == POS_DOWN)
    {
        sbus_ctrl->key_flag |= KEY_SF_PRESSED;
    }

    //更新状�?
    sbus_ctrl->last_swa_state = SWA_pos;
    sbus_ctrl->last_swb_state = SWB_pos;
    sbus_ctrl->last_swc_state = SWC_pos;
    sbus_ctrl->last_swd_state = SWD_pos;
    sbus_ctrl->last_se_state = SE_pos;
    sbus_ctrl->last_sf_state = SF_pos;

}

void Pump_On(void)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);//PE5引脚 低电平触发
}

void Pump_Off(void)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
}


static void sbus_to_remote_control(volatile const uint8_t *sbus_buffer, SBUS_ctrl_t *sbus_ctrl)
{
    if((sbus_buffer [0] == 0x0f) && (sbus_buffer[24] == 0x00))//判断头帧和尾�?
    {
        sbus_ctrl -> ch[0] = ((sbus_buffer[1] )| (sbus_buffer[2] << 8 )) & 0x07ff;//右左�?
        sbus_ctrl -> ch[1] = ((sbus_buffer[2] >> 3 )| (sbus_buffer[3] << 5 )) & 0x07ff;//右上�?
        sbus_ctrl -> ch[2] = ((sbus_buffer[3] >> 6 )| (sbus_buffer[4] << 2 ) | (sbus_buffer[5] << 10)) & 0x07ff;//左上�?
        sbus_ctrl -> ch[3] = ((sbus_buffer[5] >> 1 )| (sbus_buffer[6] << 7 )) & 0x07ff;//左左�?
        sbus_ctrl -> ch[4] = ((sbus_buffer[6] >> 4 )| (sbus_buffer[7] << 4 )) & 0x07ff;//SWA
        sbus_ctrl -> ch[5] = ((sbus_buffer[7] >> 7 )| (sbus_buffer[8] << 1 )| (sbus_buffer[9] << 9 )) & 0x07ff;//SWB
        sbus_ctrl -> ch[6] = ((sbus_buffer[9] >> 2 )| (sbus_buffer[10] << 6 )) & 0x07ff;//SWC
        sbus_ctrl-> ch[7] = ((sbus_buffer[10] >> 5 )| (sbus_buffer[11] << 3 )) & 0x07ff;//SWD
        sbus_ctrl ->ch[8] = ((sbus_buffer[12])| (sbus_buffer[13] << 8 )) & 0x07ff;//SE
        sbus_ctrl ->ch[9] = ((sbus_buffer[13] >> 3 )| (sbus_buffer[14] << 5 )) & 0x07ff;//SF

       //数据偏移
        for(int i = 0;i<10;i++)
        {
            sbus_ctrl->ch[i] = (int16_t)(sbus_ctrl->ch[i] - SBUS_CH_VALUE_OFFSET);
        }
       
        // 更新虚拟�?位状�?
        virtual_key_update(sbus_ctrl);

        // //归一化数�?
        car_x=normalize_to_range((float)sbus_ctrl -> ch[1], -800.0f, 800.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
        car_y=-normalize_to_range((float)sbus_ctrl -> ch[0], -800.0f, 800.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
        car_w=-normalize_to_range((float)sbus_ctrl -> ch[3], -800.0f, 800.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
        
        // //应用死区
        // car_x=apply_deadzone(car_x, DEADZONE);
        // car_y=apply_deadzone(car_y, DEADZONE);
        // car_w=apply_deadzone(car_w, DEADZONE);

         //MecanumWheel_Move(car_x,car_y,car_w);

        //模式切换
         if(KEY_SWB_UP & sbus_ctrl -> key_flag)
        {
            //serve_mode = SERVE_MODE_ANGLE;
            hit_set_preset(0);
        }
        else 
        {
            //serve_mode = SERVE_MODE_SPEED;
            hit_set_preset(1);
        }
        
         // 离开下档后重新�?�填一次发球触发资�?
//        if (!(KEY_SWD_DOWN & sbus_ctrl->key_flag))
//		{
//			serve_arm();
//		}

//        if (KEY_SWD_UP & sbus_ctrl -> key_flag)
//        {
//            // 上档：保留原�?角度电机控制
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
        
        if(serve_mode == SERVE_MODE_SPEED)
        {
            if(KEY_SE_DOWN & sbus_ctrl -> key_flag)
            {
                C620_up_angle.target_speed = 6000.0f;
            }
            else if(KEY_SE_UP & sbus_ctrl -> key_flag)
            {
                C620_up_angle.target_speed = 0;
            }
        }

        // SA 三档选择击球角度预设
       if (KEY_SWA_UP & sbus_ctrl->key_flag)
       {
		  // Pump_Off();
		   count_flag = 0;
		   serve_arm();
       }
       else if (KEY_SWA_DOWN & sbus_ctrl->key_flag)
       {
		  // Pump_On();
		   if(!count_flag)
		   {
			   count++;
			   count_flag = 1;
		   }
		   serve_request_start();
       }

       // SF 按下击球，松手回零
       if (detect_switch_position(sbus_ctrl->ch[9]) == POS_DOWN)
       {
           hit_request_press();
       }
       else
       {
           hit_request_release();
       }
    }
}
 
