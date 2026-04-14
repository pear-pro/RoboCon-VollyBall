#include "remote_control.h"
#include "can.h"
#include "includes.h"
#include "main.h"
#include "motor_can.h"
#include "math_utils.h"
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t   dbus_buf[DBUS_BUFLEN];

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

// 遥控看门狗参数，TIM3 周期为 10ms，15 个周期即 150ms
#define RC_WATCHDOG_TIMEOUT_TICKS (10u)

// 发球动作参数：角度单位沿用当前达妙电机 angle 标定，时间单位为 10ms 控制周期
#define SERVE_LIFT_TICKS      (15u)
#define SERVE_RETURN_TICKS    (36u)
#define SERVE_HIT_TICKS       (20u)
#define SERVE_HIT_RETURN_TICKS (20u)

//击球回零参数
#define RETURN_TICKS (40u) // 达妙电机回零时间
#define SET_LIMIT (40.0f)  //滚轮判定松手范围

#define DM0_Angle_Scale (-0.6f / 660.0f) // 遥控器输入范围 -660~660 映射到达妙电机 -0.6~0.6 的比例系数
// 发球状态机：抬球 -> 抬球回零 -> 击球 -> 击球回零
typedef enum
{
    SERVE_STAGE_IDLE = 0,
    SERVE_STAGE_LIFT,
    SERVE_STAGE_LIFT_RETURN,
    SERVE_STAGE_HIT,
    SERVE_STAGE_HIT_RETURN,
} serve_stage_t;
  

//遥控器控制变量
RC_ctrl_t rc_ctrl;
uint16_t RC_CH_VALUE_OFFS111;//receive data, 18 bytes one frame, but set 36 bytes 
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
// serve_armed 用于保证拨到下档时只触发一次发球流程
static volatile uint8_t serve_active = 0;
static volatile uint8_t serve_armed = 1;
static volatile uint16_t serve_tick = 0;
static volatile serve_stage_t serve_stage = SERVE_STAGE_IDLE; 
static volatile uint16_t rc_watchdog_tick = 0;
static volatile uint8_t rc_watchdog_timeout = 0;
static volatile uint8_t return_tick = 0;
static volatile float damiao0_tarangle=0.0f;//达妙电机0目标角度
static volatile uint8_t count=0; //达妙电机回零计数
static volatile uint8_t returning = 0;
static volatile float return_start_angle = 0.0f
/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

void remote_control_watchdog_feed(void)
{
    rc_watchdog_tick = 0;
    rc_watchdog_timeout = 0;
}

void remote_control_watchdog_update(void)
{
    // 看门狗计数，超过阈值则认为遥控器失联
    if (rc_watchdog_tick < RC_WATCHDOG_TIMEOUT_TICKS)
    {
        rc_watchdog_tick++;
    }

    // 超过阈值，进入失联状态
    if (rc_watchdog_tick >= RC_WATCHDOG_TIMEOUT_TICKS)
    {
        rc_watchdog_timeout = 1;
    }
}

uint8_t remote_control_is_timeout(void)
{
    return rc_watchdog_timeout;
}

void remote_control_enter_safe_state(void)
{
    // 底盘停止
    car_x = 0.0f;
    car_y = 0.0f;
    car_w = 0.0f;
    C620_angle.Speed_pid.set = 0.0f;

    // 发球机构回零
    serve_active = 0;
    serve_armed = 1;
    serve_tick = 0;
    serve_stage = SERVE_STAGE_IDLE;

    // 达妙电机回零
    damiao[0].angle = 0.0f;
    damiao[1].angle = -0.5f;

    // 回零相关参数复位
    count = 0;
    returning = 0;
    return_tick = 0;
    return_start_angle = 0.0f;
    damiao0_tarangle = 0.0f;
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


// 在 TIM3 的 10ms 周期中推进一次发球状态机
void remote_control_serve_update(void)
{
	float progress;
    if (!serve_active)
    {
        return;
    }

    switch (serve_stage)
    {
    case SERVE_STAGE_LIFT:
        // damiao[0] 向上抬球，先把球垫起来
       // damiao[0].angle = -0.8f;
        damiao[1].angle = -1.0f;
        if (++serve_tick >= SERVE_LIFT_TICKS)
        {
            serve_stage = SERVE_STAGE_LIFT_RETURN;
            serve_tick = 0;
        }
        break;

    case SERVE_STAGE_LIFT_RETURN:
        // 抬球机构回到零位，为后续击球让出位置
       // damiao[0].angle = 0.0f;
        damiao[1].angle = -1.0f;
        if (++serve_tick >= SERVE_RETURN_TICKS)
        {
            serve_stage = SERVE_STAGE_HIT;
            serve_tick = 0;
        }
        break;

    case SERVE_STAGE_HIT:
        // damiao[1] 向前击球
       // damiao[0].angle = 0.0f;
        damiao[1].angle = 2.35f;;
        if (++serve_tick >= SERVE_HIT_TICKS)
        {
            serve_stage = SERVE_STAGE_HIT_RETURN;
            serve_tick = 0;
        }
        break;

    case SERVE_STAGE_HIT_RETURN:
    default:
       // progress = (float)serve_tick / (float)SERVE_HIT_RETURN_TICKS;
        //progress = clamp_max(progress, 1.0f);
       // damiao[0].angle = 0.0f; // 保持抬球机构位置不变
        //damiao[1].angle = 1.0f - 2.6f * progress; // 从 1.8f 平滑过渡回 -0.8f
		    damiao[1].angle=-1.0f;
		    //serve_tick++;
        //if(serve_tick >= SERVE_HIT_RETURN_TICKS)
        //{
         //   damiao[1].angle = -0.8f;
            serve_stage = SERVE_STAGE_IDLE;
        //    serve_tick = 0;
            serve_active = 0;
       // }
		      break;
    }
}

void daimao0_angle_update(void)
{
    float ch4_abs = fabsf((float)rc_ctrl.rc.ch[4]);
     // 中档处理手动角度回零
    if (rc_ctrl.rc.s[0] != 3)
    {
        count = 0;
        returning = 0;
        return_tick = 0;
        return_start_angle = 0.0f;
        return;
    }
    if(ch4_abs > SET_LIMIT){
        count = 0;
        returning = 0;
        return_tick = 0;
        damiao[0].angle=damiao0_tarangle;
        return;
    }else if(ch4_abs<=SET_LIMIT){
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
            damiao[0].angle=0.0f;
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
    else if(USART1->SR & UART_FLAG_IDLE)
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
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                remote_control_watchdog_feed();
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
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
                remote_control_watchdog_feed();
            }
        }
    }
}


/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
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
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);    		//NULL
		
		rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;//数据偏移
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

    if (rc_ctrl->rc.ch[1] < 100 && rc_ctrl->rc.ch[1] > -100) {
        car_tarx = 0;
    }
    else {
        car_tarx=normalize_to_range(rc_ctrl->rc.ch[1], -660.0f, 660.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);  
        //car_x=low_pass(car_tarx, car_x, 0.25);

    }
    if (rc_ctrl->rc.ch[0] < 100 && rc_ctrl->rc.ch[0] > -100) {
          car_tary = 0;
    }
    else {
        car_tary=-normalize_to_range(rc_ctrl->rc.ch[0], -660.0f, 660.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
        //car_y=low_pass(car_tary, car_y, 0.32);
    }
    car_tarw=-normalize_to_range(rc_ctrl->rc.ch[2], -660.0f, 660.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
   // MecanumWheel_Move(car_x,car_y,car_w);
		
    // 离开下档后重新装填一次发球触发资格
    if (rc_ctrl->rc.s[0] != 2)
    {
        serve_armed = 1;
    }

    if (rc_ctrl->rc.s[0] == 1)
    {
        // 上档：保留原有 C620 角度电机控制
        C620_angle.Speed_pid.set = 25000 * (rc_ctrl->rc.ch[4] / 660.0f);

    }
    else if (rc_ctrl->rc.s[0] == 3)
    {
        // 中档：手动调试 damiao[0] 角度
        damiao0_tarangle = rc_ctrl->rc.ch[4] * DM0_Angle_Scale; //亚克力板是-1.2f
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

        if (!serve_active)
        {
            damiao[0].angle = 0.0f;
            damiao[1].angle = 0.0f;
        }
    }
   if(rc_ctrl->rc.s[1]==1)
    {
    //
    }
		//else if(rc_ctrl->rc.s[1]==3){
       
    //}
}
