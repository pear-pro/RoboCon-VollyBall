#include "ht_10a_remote_control.h"
#include "includes.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t   sbus_buffer[SBUS_BUFLEN];//SBUSЭ�����ݻ���25bit

static void sbus_to_remote_control(volatile const uint8_t *sbus_buffer, SBUS_ctrl_t *sbus_ctrl);

static uint8_t sbus_rx_buffer[2][SBUS_RX_BUF_NUM];//DMA˫�����������

SBUS_ctrl_t sbus_ctrl;

void sbus_remote_control_init(void)//��ʼ��SBUSЭ�����
{
    RC_init(sbus_rx_buffer[0], sbus_rx_buffer[1], SBUS_RX_BUF_NUM);
}

const SBUS_ctrl_t *get_sbus_remote_control_point(void)//��ȡSBUSЭ��ң��������ָ��
{
    return &sbus_ctrl;
}

//�����ж�
void USART1_IRQHandlerCallBack(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)//�������
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
            //�趨������1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
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
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                sbus_to_remote_control(sbus_rx_buffer[1], &sbus_ctrl);
            }
        }
    }
}

static void sbus_to_remote_control(volatile const uint8_t *sbus_buffer, SBUS_ctrl_t *sbus_ctrl)
{
    if((sbus_buffer [0] == 0x0f) && (sbus_buffer[24] == 0x00))//�ж�ͷ֡��β֡
    {
        sbus_ctrl -> ch[0] = ((sbus_buffer[1] )| (sbus_buffer[2] << 8 )) & 0x07ff;//��ҡ������
        sbus_ctrl -> ch[1] = ((sbus_buffer[2] >> 3 )| (sbus_buffer[3] << 5 )) & 0x07ff;//��ҡ������
        sbus_ctrl -> ch[2] = ((sbus_buffer[3] >> 6 )| (sbus_buffer[4] << 2 ) | (sbus_buffer[5] << 10)) & 0x07ff;//��ҡ������
        sbus_ctrl -> ch[3] = ((sbus_buffer[5] >> 1 )| (sbus_buffer[6] << 7 )) & 0x07ff;//��ҡ������
        sbus_ctrl -> ch[4] = ((sbus_buffer[6] >> 4 )| (sbus_buffer[7] << 4 )) & 0x07ff;//SWA
        sbus_ctrl -> ch[5] = ((sbus_buffer[7] >> 7 )| (sbus_buffer[8] << 1 )| (sbus_buffer[9] << 9 )) & 0x07ff;//SWB
        sbus_ctrl -> ch[6] = ((sbus_buffer[9] >> 2 )| (sbus_buffer[10] << 6 )) & 0x07ff;//SWC
        sbus_ctrl -> ch[7] = ((sbus_buffer[10] >> 5 )| (sbus_buffer[11] << 3 )) & 0x07ff;//SWD

        //��һ��
        car_x=normalize_to_range(sbus_ctrl -> ch[1], 1000.0f, 2000.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
        car_y=-normalize_to_range(sbus_ctrl -> ch[0], 1000.0f, 2000.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
        car_w=-normalize_to_range(sbus_ctrl -> ch[3], 1000.0f, 2000.0f, -MAX_CAR_SPEED, MAX_CAR_SPEED);
        
        //Ӧ����������
        car_x=apply_deadzone(car_x, DEADZONE);
        car_y=apply_deadzone(car_y, DEADZONE);
        car_w=apply_deadzone(car_w, DEADZONE);

        MecanumWheel_Move(car_x,car_y,car_w);
        // 使用USART2打印通道数据

        USART2_Print("Channel Data: Ch0=%d\r\n, Ch1=%d\r\n, Ch2=%d\r\n, Ch3=%d\r\n, Ch4=%d\r\n, Ch5=%d\r\n, Ch6=%d\r\n, Ch7=%d\r\n",
             sbus_ctrl->ch[0],sbus_ctrl->ch[1],sbus_ctrl->ch[2],sbus_ctrl->ch[3],
             sbus_ctrl->ch[4],sbus_ctrl->ch[5],sbus_ctrl->ch[6],sbus_ctrl->ch[7]);

        // 打印归一化后的速度数据

        USART2_Print("Speed Data: car_x=%.2f, car_y=%.2f, car_w=%.2f\r\n", 
             car_x, car_y, car_w);
    }



}
