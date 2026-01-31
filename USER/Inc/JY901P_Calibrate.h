#ifndef __JY901P_CALIBRATE_H
#define __JY901P_CALIBRATE_H

#include "stm32f4xx_hal.h"
#include "JY901P.h"
#include "i2c.h"
// 移除 delay.h，直接使用 HAL_Delay

// 解锁寄存器（16位）
#define JY901P_REG_KEY        0x69
#define JY901P_UNLOCK_CODE    0xB588  // 16位解锁码

// 校准模式寄存器（16位）
#define JY901P_REG_CALSW      0x01
#define JY901P_CAL_LEVEL      0x0001  // 16位水平校准指令
#define JY901P_CAL_SETREF     0x0008  // 16位设置参考指令

// 函数声明
void JY901P_Unlock(void);
void JY901P_Calibrate_SetRef(void);
void JY901P_Calibrate_Level(void);
void JY901P_Calibrate_Full(void);  // 先归零再校准的完整流程

#endif

