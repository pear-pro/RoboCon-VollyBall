#ifndef __JY901P_H
#define __JY901P_H

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "i2c.h"

// 设备I2C地址（JY901P默认地址）
#define JY901P_ADDR    (0x50 << 1)

// 寄存器地址宏定义（匹配你的寄存器表）
#define JY901P_REG_AX      0x34  // 加速度X
#define JY901P_REG_AY      0x35  // 加速度Y
#define JY901P_REG_AZ      0x36  // 加速度Z
#define JY901P_REG_GX      0x37  // 角速度X
#define JY901P_REG_GY      0x38  // 角速度Y
#define JY901P_REG_GZ      0x39  // 角速度Z
#define JY901P_REG_ROLL    0x3D  // 横滚角（Angle_X）
#define JY901P_REG_PITCH   0x3E  // 俯仰角（Angle_Y）
#define JY901P_REG_YAW     0x3F  // 航向角（Angle_Z）

// 9轴数据结构体
typedef struct {
    int16_t Angle_X;   // 横滚角（Roll）
    int16_t Angle_Y;   // 俯仰角（Pitch）
    int16_t Angle_Z;   // 航向角（Yaw）
    int16_t Gyro_X;    // 角速度X
    int16_t Gyro_Y;    // 角速度Y
    int16_t Gyro_Z;    // 角速度Z
    int16_t Acc_X;     // 加速度X
    int16_t Acc_Y;     // 加速度Y
    int16_t Acc_Z;     // 加速度Z
} JY901P_DataStruct;


extern I2C_HandleTypeDef hi2c2;

// 核心函数：逐个读取寄存器数据到结构体
void JY901P_ReadAllData(JY901P_DataStruct *pData);

// 声明DMA接收缓冲区和读取函数
extern uint8_t jy901p_i2c2_dma_rx_buf[18];
uint32_t JY901P_ReadI2C2_DMA_Data(uint8_t *pRxData, uint32_t max_len);
//void JY901P_StartI2C2_DMA_Read(void);

#endif

