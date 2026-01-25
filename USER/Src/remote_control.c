// #include "remote_control.h"
// #include "includes.h"
// #include "main.h"

// extern UART_HandleTypeDef huart1;
// extern DMA_HandleTypeDef hdma_usart1_rx;

// uint8_t   dbus_buf[DBUS_BUFLEN];

// /**
//   * @brief          remote control protocol resolution
//   * @param[in]      sbus_buf: raw data point
//   * @param[out]     rc_ctrl: remote control data struct point
//   * @retval         none
//   */
// /**
//   * @brief          遥控器协议解析
//   * @param[in]      sbus_buf: 原生数据指针
//   * @param[out]     rc_ctrl: 遥控器数据指
//   * @retval         none
//   */
// static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

// //remote control data 
// //遥控器控制变量
// RC_ctrl_t rc_ctrl;
// uint16_t RC_CH_VALUE_OFFS111;//receive data, 18 bytes one frame, but set 36 bytes 
// //接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
// static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

// /**
//   * @brief          remote control init
//   * @param[in]      none
//   * @retval         none
//   */
// /**
//   * @brief          遥控器初始化
//   * @param[in]      none
//   * @retval         none
//   */
// void remote_control_init(void)
// {
//     RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);

//     rc_ctrl.rc.last_s1_state = RC_SW_MID;  // S1初始化状态MID
//     rc_ctrl.rc.last_s2_state = RC_SW_MID;        // S2初始化状态MID
//     rc_ctrl.rc.key_flag = KEY_NONE;       // 初始化标志位NONE
// }
// /**
//   * @brief          get remote control data point
//   * @param[in]      none
//   * @retval         remote control data point
//   */
// /**
//   * @brief          获取遥控器数据指针
//   * @param[in]      none
//   * @retval         遥控器数据指针
//   */
// const RC_ctrl_t *get_remote_control_point(void)
// {
//     return &rc_ctrl;
// }


// //串口中断
// void USART1_IRQHandlerCallBack(void)
// {
//     if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
//     {
//         __HAL_UART_CLEAR_PEFLAG(&huart1);
//     }
//     else if(USART1->SR & UART_FLAG_IDLE)
//     {
//         static uint16_t this_time_rx_len = 0;

//         __HAL_UART_CLEAR_PEFLAG(&huart1);

//         if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//         {
//             /* Current memory buffer used is Memory 0 */
    
//             //disable DMA
//             //失效DMA
//             __HAL_DMA_DISABLE(&hdma_usart1_rx);

//             //get receive data length, length = set_data_length - remain_length
//             //获取接收数据长度,长度 = 设定长度 - 剩余长度
//             this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

//             //reset set_data_lenght
//             //重新设定数据长度
//             hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

//             //set memory buffer 1
//             //设定缓冲区1
//             hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            
//             //enable DMA
//             //使能DMA
//             __HAL_DMA_ENABLE(&hdma_usart1_rx);

//             if(this_time_rx_len == RC_FRAME_LENGTH)
//             {
//                 sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
//             }
//         }
//         else
//         {
//             /* Current memory buffer used is Memory 1 */
//             //disable DMA
//             //失效DMA
//             __HAL_DMA_DISABLE(&hdma_usart1_rx);

//             //get receive data length, length = set_data_length - remain_length
//             //获取接收数据长度,长度 = 设定长度 - 剩余长度
//             this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

//             //reset set_data_lenght
//             //重新设定数据长度
//             hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

//             //set memory buffer 0
//             //设定缓冲区0
//             DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
//             //enable DMA
//             //使能DMA
//             __HAL_DMA_ENABLE(&hdma_usart1_rx);

//             if(this_time_rx_len == RC_FRAME_LENGTH)
//             {
//                 //处理遥控器数据
//                 sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
//             }
//         }
//     }
// }


// /**
//   * @brief          remote control protocol resolution
//   * @param[in]      sbus_buf: raw data point
//   * @param[out]     rc_ctrl: remote control data struct point
//   * @retval         none
//   */
// /**
//   * @brief          遥控器协议解析
//   * @param[in]      sbus_buf: 原生数据指针
//   * @param[out]     rc_ctrl: 遥控器数据指
//   * @retval         none
//   */
// static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
// {
//     if (sbus_buf == NULL || rc_ctrl == NULL)
//     {
//         return;
//     }

//     rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0右左右
//     rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1右上下
//     rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2左左右
//                          (sbus_buf[4] << 10)) &0x07ff;
//     rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3左上下
//     rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left右上拨杆
//     rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right左上拨杆
//     rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
//     rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
//     rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
//     rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
//     rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
//     rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
//     rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL左上角滚轮

//     rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;//数据偏移
//     rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
//     rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
//     rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
//     rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

//     car_x=normalize_to_range(rc_ctrl->rc.ch[1], -1000.0f, 1000.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
//     car_y=-normalize_to_range(rc_ctrl->rc.ch[0], -1000.0f, 1000.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
//     car_w=-normalize_to_range(rc_ctrl->rc.ch[2], -1000.0f, 1000.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);

//     // 应用死区
//     car_x = apply_deadzone(car_x, DEADZONE);
//     car_y = apply_deadzone(car_y, DEADZONE);
//     car_w = apply_deadzone(car_w, DEADZONE);

//     MecanumWheel_Move(car_x,car_y,car_w);
// }

// static uint8_t detect_switch_position(int16_t value)
// {
//     if (value == 1) 
//     {
//         return RC_SW_UP;  // 上
//     } 
//     else if (value == 2) 
//     {
//         return RC_SW_DOWN;  // 下
//     } 
//     else 
//     {
//         return RC_SW_MID;  // 中
//     }
// }

// static void virtual_key_update(RC_ctrl_t *rc_ctrl)
// {
//     // 更新开关状态
//     uint8_t S1_pos = detect_switch_position(rc_ctrl->rc.s[0]);
//     uint8_t S2_pos = detect_switch_position(rc_ctrl->rc.s[1]);

//     //初始化虚拟键位状态
//     rc_ctrl->rc.key_flag = KEY_NONE;

//     //S1处理
//     if(rc_ctrl->rc.last_s1_state == RC_SW_MID && S1_pos == RC_SW_UP)
//     {
//         rc_ctrl->rc.key_flag |= KEY_S1_UP;
//     }
//     else if(rc_ctrl->rc.last_s1_state == RC_SW_MID && S1_pos == RC_SW_DOWN)
//     {
//         rc_ctrl->rc.key_flag |= KEY_S1_DOWN;
//     }
//     else if(S1_pos == RC_SW_MID)
//     {
//         rc_ctrl->rc.key_flag |= KEY_S1_MID;
//     }

//     //S2处理
//     if(rc_ctrl->rc.last_s2_state == RC_SW_MID && S2_pos == RC_SW_UP)
//     {
//         rc_ctrl->rc.key_flag |= KEY_S2_UP;
//     }
//     else if(rc_ctrl->rc.last_s2_state == RC_SW_MID && S2_pos == RC_SW_DOWN)
//     {
//         rc_ctrl->rc.key_flag |= KEY_S2_DOWN;
//     }
//     else if(S2_pos == RC_SW_MID)
//     {
//         rc_ctrl->rc.key_flag |= KEY_S2_MID;
//     }


//     //更新状态
//     rc_ctrl->rc.last_s1_state = S1_pos;
//     rc_ctrl->rc.last_s2_state = S2_pos;

// }



