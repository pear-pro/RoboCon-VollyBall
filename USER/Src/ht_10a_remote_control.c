#include "ht_10a_remote_control.h"
#include "includes.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t   sbus_buffer[SBUS_BUFLEN];//SBUS协议数据缓存25bit

static void sbus_to_remote_control(volatile const uint8_t *sbus_buffer, SBUS_ctrl_t *sbus_ctrl);

static uint8_t sbus_rx_buffer[2][SBUS_RX_BUF_NUM];//DMA双缓冲接收数组

SBUS_ctrl_t sbus_ctrl;

void sbus_remote_control_init(void)//初始化SBUS协议接收
{
    RC_init(sbus_rx_buffer[0], sbus_rx_buffer[1], SBUS_RX_BUF_NUM);
}

const SBUS_ctrl_t *get_sbus_remote_control_point(void)//获取SBUS协议遥控器数据指针
{
    return &sbus_ctrl;
}

//串口中断
void USART1_IRQHandlerCallBack(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)//接收完成
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

static void sbus_to_remote_control(volatile const uint8_t *sbus_buffer, SBUS_ctrl_t *sbus_ctrl)
{
    if((sbus_buffer [0] == 0x0f) && (sbus_buffer[24] == 0x00))//判断头帧和尾帧
    {
        sbus_ctrl -> ch[0] = ((sbus_buffer[1] )| (sbus_buffer[2] << 8 )) & 0x07ff;//右摇杆左右
        sbus_ctrl -> ch[1] = ((sbus_buffer[2] >> 3 )| (sbus_buffer[3] << 5 )) & 0x07ff;//右摇杆上下
        sbus_ctrl -> ch[2] = ((sbus_buffer[3] >> 6 )| (sbus_buffer[4] << 2 ) | (sbus_buffer[5] << 10)) & 0x07ff;//左摇杆上下
        sbus_ctrl -> ch[3] = ((sbus_buffer[5] >> 1 )| (sbus_buffer[6] << 7 )) & 0x07ff;//左摇杆左右
        sbus_ctrl -> ch[4] = ((sbus_buffer[6] >> 4 )| (sbus_buffer[7] << 4 )) & 0x07ff;//SWA
        sbus_ctrl -> ch[5] = ((sbus_buffer[7] >> 7 )| (sbus_buffer[8] << 1 )| (sbus_buffer[9] << 9 )) & 0x07ff;//SWB
        sbus_ctrl -> ch[6] = ((sbus_buffer[9] >> 2 )| (sbus_buffer[10] << 6 )) & 0x07ff;//SWC
        sbus_ctrl -> ch[7] = ((sbus_buffer[10] >> 5 )| (sbus_buffer[11] << 3 )) & 0x07ff;//SWD

        //归一化
        car_x=normalize_to_range(sbus_ctrl -> ch[1], 1000.0f, 2000.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
        car_y=-normalize_to_range(sbus_ctrl -> ch[0], 1000.0f, 2000.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
        car_w=-normalize_to_range(sbus_ctrl -> ch[3], 1000.0f, 2000.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
        
        //应用死区处理
        car_x=apply_deadzone(car_x, DEADZONE);
        car_y=apply_deadzone(car_y, DEADZONE);
        car_w=apply_deadzone(car_w, DEADZONE);

        MecanumWheel_Move(car_x,car_y,car_w);
    }



}
