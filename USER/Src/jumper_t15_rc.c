#include "jumper_t15_rc.h"
#include "includes.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

// 发球动作参数：角度单位沿用当前达妙电机 angle 标定，时间单位为 10ms 控制周期
#define SERVE_LIFT_ANGLE      (-0.10f)
#define SERVE_HIT_ANGLE       (-0.18f)
#define SERVE_LIFT_TICKS      (15u)
#define SERVE_RETURN_TICKS    (36u)
#define SERVE_HIT_TICKS       (20u)
#define SERVE_HIT_RETURN_TICKS (20u)

#define RETURN_TICKS (500u)
#define SET_LIMIT (0.05f)  //滚轮判定松手范围
// 发球状态机：抬球 -> 抬球回零 -> 击球 -> 击球回零
typedef enum
{
    SERVE_STAGE_IDLE = 0,
    SERVE_STAGE_LIFT,
    SERVE_STAGE_LIFT_RETURN,
    SERVE_STAGE_HIT,
    SERVE_STAGE_HIT_RETURN,
} serve_stage_t;

uint8_t   sbus_buffer[SBUS_BUFLEN];//SBUS接收缓冲区

static void sbus_to_remote_control(volatile const uint8_t *sbus_buffer, SBUS_ctrl_t *sbus_ctrl);

static uint8_t sbus_rx_buffer[2][SBUS_RX_BUF_NUM];//DMA双缓冲

SBUS_ctrl_t sbus_ctrl;

// serve_armed 用于保证拨到下档时只触发一次发球流程
static volatile uint8_t serve_active = 0;
static volatile uint8_t serve_armed = 1;
static volatile uint16_t serve_tick = 0;
static volatile serve_stage_t serve_stage = SERVE_STAGE_IDLE; 
static volatile uint16_t rc_watchdog_tick = 0;
static volatile uint8_t rc_watchdog_timeout = 0;
static volatile uint16_t return_tick = 0;
static volatile float damiao0_tarangle=0.0f;//达妙电机0目标角度
static volatile uint16_t count=0; //达妙电机回零计数
static volatile uint8_t returning = 0;
static volatile float return_start_angle = 0.0f;

void sbus_remote_control_init(void)//SBUS遥控器初始化
{
    RC_init(sbus_rx_buffer[0], sbus_rx_buffer[1], SBUS_RX_BUF_NUM);

    
    sbus_ctrl.last_swa_state = POS_MID;  // SWA初始化状态MID
    sbus_ctrl.last_swb_state = POS_MID;        // SWB初始化状态MID
    sbus_ctrl.last_swc_state = POS_MID;        // SWC初始化状态MID
    sbus_ctrl.last_swd_state = POS_MID;  // SWD初始化状态MID
    sbus_ctrl.last_se_state = POS_UP;  // SE初始化状态UP
    sbus_ctrl.last_sf_state = POS_UP;  // SF初始化状态UP
    sbus_ctrl.key_flag = KEY_NONE;       // 初始化标志位NONE
}

const SBUS_ctrl_t *get_sbus_remote_control_point(void)//获取SBUS遥控器指针
{
    return &sbus_ctrl;
}

float remote_control_meanum_update(float input,float target,float up_ticks,float down_ticks,float max_speed)
{
    float delat = target - input;
    if (fabsf(delat) < 20.0f) {
        return target; // 已经非常接近目标值，直接返回目标值
    }
    if (delat > 0) {
        // 需要加速
        float step = max_speed / up_ticks; // 每个周期的加速步长
        return input + fminf(step, delat); // 不要超过目标值
    } else {
        // 需要减速
        float step = max_speed / down_ticks; // 每个周期的减速步长
        return input + fmaxf(-step, delat); // 不要超过目标值
    }
}

////void remote_control_serve_update(void)
//{
//	float progress;
//    if (!serve_active)
//    {
//        return;
//    }

//    switch (serve_stage)
//    {
//    case SERVE_STAGE_LIFT:
//        // damiao[0] 向上抬球，先把球垫起来
//        damiao[0].angle = -0.8f;
//        damiao[1].angle = -0.8f;
//        if (++serve_tick >= SERVE_LIFT_TICKS)
//        {
//            serve_stage = SERVE_STAGE_LIFT_RETURN;
//            serve_tick = 0;
//        }
//        break;

//    case SERVE_STAGE_LIFT_RETURN:
//        // 抬球机构回到零位，为后续击球让出位置
//        damiao[0].angle = -0.25f;
//        damiao[1].angle = -0.8f;
//        if (++serve_tick >= SERVE_RETURN_TICKS)
//        {
//            serve_stage = SERVE_STAGE_HIT;
//            serve_tick = 0;
//        }
//        break;

//    case SERVE_STAGE_HIT:
//        // damiao[1] 向前击球
//        damiao[0].angle = -0.25f;
//        damiao[1].angle = 1.8f;//1.8f;
//        if (++serve_tick >= SERVE_HIT_TICKS)
//        {
//            serve_stage = SERVE_STAGE_HIT_RETURN;
//            serve_tick = 0;
//        }
//        break;

//    case SERVE_STAGE_HIT_RETURN:
//    default:
//       // progress = (float)serve_tick / (float)SERVE_HIT_RETURN_TICKS;
//        //progress = clamp_max(progress, 1.0f);
//        damiao[0].angle = -0.25f; // 保持抬球机构位置不变
//        //damiao[1].angle = 1.0f - 2.6f * progress; // 从 1.8f 平滑过渡回 -0.8f
//		    damiao[1].angle=-0.5f;
//		    //serve_tick++;
//        //if(serve_tick >= SERVE_HIT_RETURN_TICKS)
//        //{
//         //   damiao[1].angle = -0.8f;
//            serve_stage = SERVE_STAGE_IDLE;
//        //    serve_tick = 0;
//            serve_active = 0;
//       // }
//		      break;
//    }
//}

void damiao0_angle_update(void)
{
    float ch3_abs = fabsf((float)sbus_ctrl.ch[3]);
     // 中档处理手动角度回零
    if(ch3_abs > SET_LIMIT){
        count = 0;
        returning = 0;
        return_tick = 0;
        damiao[0].angle=damiao0_tarangle;
        return;
    }else if(ch3_abs<=SET_LIMIT){
        if(count<=3){count++;}
        if(count>3 && !returning){
            returning=1;
            return_tick = 0;
            return_start_angle=damiao[0].angle; // 记录开始回零时的角度
        }
    }
    if(returning){
        float progress = (float)return_tick / (float)RETURN_TICKS;
        progress = clamp_max(progress, 1.0f);
        float s = progress * progress * (3.0f - 2.0f * progress);
        damiao[0].angle = return_start_angle * (1.0f - s);
        //float s = sinf(progress * 1.5707963f);   // pi/2
        //damiao[0].angle = return_start_angle * (1.0f - s);
        if(++return_tick >= RETURN_TICKS)   
        {
            damiao[0].angle=-0.0f;
            returning=0;
            return_tick=0;
            count = 0;
        }
    }
}

//串口中断
void USART1_IRQHandlerCallBack(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)//空闲中断
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
            //设定缓冲区1
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
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_remote_control(sbus_rx_buffer[1], &sbus_ctrl);
            }
        }
    }
}

static uint8_t detect_switch_position(int16_t value)
{
    if (value <= SWITCH_SBUS_CH_VALUE_MIN + DEADZONE)
    {
        return POS_UP;  // 上
    } 
    else if (value >= SWITCH_SBUS_CH_VALUE_MAX - DEADZONE) 
    {
        return POS_DOWN;  // 下
    } 
    else 
    {
        return POS_MID;  // 中
    }
}

static void virtual_key_update(SBUS_ctrl_t *sbus_ctrl)
{
    // 更新开关状态
    uint8_t SWA_pos = detect_switch_position(sbus_ctrl -> ch[4]);
    uint8_t SWB_pos = detect_switch_position(sbus_ctrl -> ch[5]);
    uint8_t SWC_pos = detect_switch_position(sbus_ctrl -> ch[6]);
    uint8_t SWD_pos = detect_switch_position(sbus_ctrl -> ch[7]);
    uint8_t SE_pos = detect_switch_position(sbus_ctrl -> ch[8]);
    uint8_t SF_pos = detect_switch_position(sbus_ctrl -> ch[9]);

    //初始化虚拟键位状态
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
        else if(SWA_pos == POS_MID)
        {
            sbus_ctrl->key_flag |= KEY_SWA_MID; 
        }
    }
    else
    {
    // 状态不变时设置对应位置的标志
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
    // 状态不变时设置对应位置的标志
        if(SWB_pos == POS_UP)
        {
            sbus_ctrl->key_flag |= KEY_SWB_UP;
        }
        else if(SWB_pos == POS_MID)
        {
            sbus_ctrl->key_flag |= KEY_SWB_MID;
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
    // 状态不变时设置对应位置的标志
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
    // 状态不变时设置对应位置的标志
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
        // 保持状态时也设置对应状态
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

    //更新状态
    sbus_ctrl->last_swa_state = SWA_pos;
    sbus_ctrl->last_swb_state = SWB_pos;
    sbus_ctrl->last_swc_state = SWC_pos;
    sbus_ctrl->last_swd_state = SWD_pos;
    sbus_ctrl->last_se_state = SE_pos;
    sbus_ctrl->last_sf_state = SF_pos;

}


static void sbus_to_remote_control(volatile const uint8_t *sbus_buffer, SBUS_ctrl_t *sbus_ctrl)
{
    if((sbus_buffer [0] == 0x0f) && (sbus_buffer[24] == 0x00))//判断头帧和尾帧
    {
        sbus_ctrl -> ch[0] = ((sbus_buffer[1] )| (sbus_buffer[2] << 8 )) & 0x07ff;//右左右
        sbus_ctrl -> ch[1] = ((sbus_buffer[2] >> 3 )| (sbus_buffer[3] << 5 )) & 0x07ff;//右上下
        sbus_ctrl -> ch[2] = ((sbus_buffer[3] >> 6 )| (sbus_buffer[4] << 2 ) | (sbus_buffer[5] << 10)) & 0x07ff;//左上下
        sbus_ctrl -> ch[3] = ((sbus_buffer[5] >> 1 )| (sbus_buffer[6] << 7 )) & 0x07ff;//左左右
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
       
        // 更新虚拟键位状态
        virtual_key_update(sbus_ctrl);

        // //归一化数据
        // car_x=normalize_to_range((float)sbus_ctrl -> ch[1], -800.0f, 800.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
        // car_y=-normalize_to_range((float)sbus_ctrl -> ch[0], -800.0f, 800.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
        // car_w=-normalize_to_range((float)sbus_ctrl -> ch[3], -800.0f, 800.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
        
        // //应用死区
        // car_x=apply_deadzone(car_x, DEADZONE);
        // car_y=apply_deadzone(car_y, DEADZONE);
        // car_w=apply_deadzone(car_w, DEADZONE);

        if (sbus_ctrl->ch[1] < 100 && sbus_ctrl->ch[1] > -100) {
        car_tarx = 0;
        }
        else {
            car_tarx=normalize_to_range(sbus_ctrl->ch[1], -800.0f, 800.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);  
            //car_x=low_pass(car_tarx, car_x, 0.25);

        }
        if (sbus_ctrl->ch[0] < 100 && sbus_ctrl->ch[0] > -100) {
            car_tary = 0;
        }
        else {
            car_tary=-normalize_to_range(sbus_ctrl->ch[0], -800.0f, 800.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
            //car_y=low_pass(car_tary, car_y, 0.32);
        }
        car_tarw=-normalize_to_range(sbus_ctrl->ch[2], -800.0f, 800.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);

        // MecanumWheel_Move(car_x,car_y,car_w);

        // 离开下档后重新装填一次发球触发资格
//        if (KEY_SWB_UP & sbus_ctrl -> key_flag)
//        {
//            serve_armed = 1;
//        }

        if (KEY_SWB_UP & sbus_ctrl -> key_flag)
        {
            // 上档：保留原有 C620 角度电机控制
            C620_angle.Speed_pid.set = 25000 * (sbus_ctrl->ch[3] / 800.0f);
            //if (!serve_active)
            //{
            //  damiao[0].angle = 0.0f;
                //damiao[1].angle = 0.0f;
            //}
        }
//        else if (KEY_SWB_MID & sbus_ctrl -> key_flag)
//        {
//            // 中档：手动调试 damiao[0] 角度
//            //C620_angle.Speed_pid.set = 0.0f;
//            //if (!serve_active)
//            //{  
//           damiao[0].angle = sbus_ctrl->ch[2] * (-1.2f / 800.0f);
//            if (damiao[0].angle > 0)
//            {
//                damiao[0].angle = -damiao[0].angle;
//            }
//            if(damiao[0].angle > -0.05f)
//            {
//                damiao[0].angle = 0.0f;
//            }           //  damiao[1].angle = 0.0f;
            //}
					
//					if(KEY_SWC_MID & sbus_ctrl -> key_flag)
//					{
//						 damiao[0].angle = -0.6f;
//					}
        }
        else
        {
            // 下档：触发一次自动发球流程
            if (serve_armed && !serve_active)
            {
                serve_active = 1;
                serve_armed = 0;
                serve_tick = 0;
                serve_stage = SERVE_STAGE_LIFT;
            }

//            if (!serve_active)
//            {
//                damiao[0].angle = 0.0f;
//                damiao[1].angle = 0.0f;
//            }
        }
        if(detect_switch_position(sbus_ctrl -> ch[9]) ==POS_DOWN  & sbus_ctrl -> key_flag)
        {
       
          damiao[0].angle=-1.2f;
        }
         else{
        
        damiao[0].angle=-0.5f;

    }
}
	
 



