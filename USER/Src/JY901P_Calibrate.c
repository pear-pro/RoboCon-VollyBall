#include "JY901P_Calibrate.h"
#include "i2c.h"


// 解锁函数
void JY901P_Unlock(void)
{
    uint8_t unlock_data[2];
    unlock_data[0] = (uint8_t)(JY901P_UNLOCK_CODE & 0xFF);
    unlock_data[1] = (uint8_t)(JY901P_UNLOCK_CODE >> 8);
    
    if (HAL_I2C_Mem_Write(&hi2c1, JY901P_ADDR, JY901P_REG_KEY, I2C_MEMADD_SIZE_8BIT, unlock_data, 2, 100) != HAL_OK)
    {
        return;
    }
}

// 设置当前姿态为0°参考（X/Y/Z全轴归零）
void JY901P_Calibrate_SetRef(void)
{
    uint8_t setref_data[2];
    JY901P_Unlock();
    HAL_Delay(10);  // 使用HAL自带延时
    
    setref_data[0] = (uint8_t)(JY901P_CAL_SETREF & 0xFF);
    setref_data[1] = (uint8_t)(JY901P_CAL_SETREF >> 8);
    
    if (HAL_I2C_Mem_Write(&hi2c1, JY901P_ADDR, JY901P_REG_CALSW, I2C_MEMADD_SIZE_8BIT, setref_data, 2, 100) != HAL_OK)
    {
        return;
    }
    
    HAL_Delay(2000);  // 使用HAL自带延时
}

// 水平校准（建立精准水平基准）
void JY901P_Calibrate_Level(void)
{
    uint8_t level_data[2];
    JY901P_Unlock();
    HAL_Delay(10);  // 使用HAL自带延时
    
    level_data[0] = (uint8_t)(JY901P_CAL_LEVEL & 0xFF);
    level_data[1] = (uint8_t)(JY901P_CAL_LEVEL >> 8);
    
    if (HAL_I2C_Mem_Write(&hi2c1, JY901P_ADDR, JY901P_REG_CALSW, I2C_MEMADD_SIZE_8BIT, level_data, 2, 100) != HAL_OK)
    {
        return;
    }
    
    HAL_Delay(2000);  // 使用HAL自带延时
}

// 完整校准流程：先归零再水平校准
void JY901P_Calibrate_Full(void)
{
    JY901P_Calibrate_SetRef();
    JY901P_Calibrate_Level();
}
