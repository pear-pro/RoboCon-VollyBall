#include "jy901p.h"
#include "i2c.h"
#include <string.h>
#include "main.h"
#include "includes.h"

extern DMA_HandleTypeDef hdma_i2c2_rx;

// 读取单个16位寄存器
static int16_t JY901P_Read16BitReg(uint8_t reg_addr)
{
    uint8_t temp_buf[2] = {0};
    if (HAL_I2C_Mem_Read(&hi2c2, JY901P_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, temp_buf, 2, 100) != HAL_OK)
    {
        return 0;
    }
    return (int16_t)(temp_buf[1] << 8 | temp_buf[0]);
}

void JY901P_ReadAllData(JY901P_DataStruct *pData)
{
    pData->Acc_X   = JY901P_Read16BitReg(JY901P_REG_AX);
    pData->Acc_Y   = JY901P_Read16BitReg(JY901P_REG_AY);
    pData->Acc_Z   = JY901P_Read16BitReg(JY901P_REG_AZ);
    
    pData->Gyro_X  = JY901P_Read16BitReg(JY901P_REG_GX);
    pData->Gyro_Y  = JY901P_Read16BitReg(JY901P_REG_GY);
    pData->Gyro_Z  = JY901P_Read16BitReg(JY901P_REG_GZ);
    
    pData->Angle_X = JY901P_Read16BitReg(JY901P_REG_ROLL);
    pData->Angle_Y = JY901P_Read16BitReg(JY901P_REG_PITCH);
    pData->Angle_Z = JY901P_Read16BitReg(JY901P_REG_YAW);
}

// 全局DMA接收缓冲区，用于批量接收9个16位寄存器（共18字节）
uint8_t jy901p_i2c2_dma_rx_buf[18] = {0};

// 新增：读取I2C1 DMA接收的当前状态与已接收数据
// pRxData: 输出缓冲区，用于存放DMA接收到的数据
// max_len: 输出缓冲区的最大长度
// 返回值：实际已接收的字节数
uint32_t JY901P_ReadI2C2_DMA_Data(uint8_t *pRxData, uint32_t max_len)
{
    // 1. 获取DMA剩余未传输的数据量
    uint32_t dma_remain = __HAL_DMA_GET_COUNTER(&hdma_i2c2_rx);
    
    // 2. 计算已接收的数据量（你配置的是18字节批量接收）
    uint32_t recv_len = 18 - dma_remain;

    // 3. 安全拷贝数据到用户缓冲区，防止溢出
    if (pRxData != NULL && recv_len > 0)
    {
        uint32_t copy_len = recv_len > max_len ? max_len : recv_len;
        memcpy(pRxData, jy901p_i2c2_dma_rx_buf, copy_len);
    }

    // 4. 返回已接收的有效字节数
    return recv_len;
}

