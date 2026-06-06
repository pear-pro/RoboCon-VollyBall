#include "jy901p.h"
#include "i2c.h"
#include "main.h"
#include <string.h>

#define JY901P_I2C_TIMEOUT_MS 2U

static uint8_t JY901P_Read16BitReg(uint8_t reg_addr, int16_t *raw)
{
    uint8_t temp_buf[2] = {0};

    if (raw == NULL) {
        return 0;
    }

    if (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
        return 0;
    }

    if (HAL_I2C_Mem_Read(&hi2c2,
                         JY901P_ADDR,
                         reg_addr,
                         I2C_MEMADD_SIZE_8BIT,
                         temp_buf,
                         2,
                         JY901P_I2C_TIMEOUT_MS) != HAL_OK) {
        JY901P_Recover();
        return 0;
    }

    *raw = (int16_t)(temp_buf[1] << 8 | temp_buf[0]);
    return 1;
}

void JY901P_Recover(void)
{
    HAL_I2C_DeInit(&hi2c2);
    HAL_I2C_Init(&hi2c2);
}

uint8_t JY901P_ReadYawDeg(float *yaw_deg)
{
    int16_t raw_yaw;

    if (yaw_deg == NULL) {
        return 0;
    }

    if (!JY901P_Read16BitReg(JY901P_REG_YAW, &raw_yaw)) {
        return 0;
    }

    *yaw_deg = (float)raw_yaw * 0.0054931640625f;
    return 1;
}

uint8_t JY901P_ReadAllDataSafe(JY901P_DataStruct *pData)
{
    int16_t raw;
    JY901P_DataStruct next;

    if (pData == NULL) {
        return 0;
    }

    next = *pData;

    if (!JY901P_Read16BitReg(JY901P_REG_GX, &raw)) {
        return 0;
    }
    next.Gyro_X = (int16_t)((float)raw * 0.06103515625f);

    if (!JY901P_Read16BitReg(JY901P_REG_GY, &raw)) {
        return 0;
    }
    next.Gyro_Y = (int16_t)((float)raw * 0.06103515625f);

    if (!JY901P_Read16BitReg(JY901P_REG_GZ, &raw)) {
        return 0;
    }
    next.Gyro_Z = (int16_t)((float)raw * 0.06103515625f);

    if (!JY901P_Read16BitReg(JY901P_REG_ROLL, &raw)) {
        return 0;
    }
    next.Angle_X = (int16_t)((float)raw * 0.0054931640625f);

    if (!JY901P_Read16BitReg(JY901P_REG_PITCH, &raw)) {
        return 0;
    }
    next.Angle_Y = (int16_t)((float)raw * 0.0054931640625f);

    if (!JY901P_Read16BitReg(JY901P_REG_YAW, &raw)) {
        return 0;
    }
    next.Angle_Z = (int16_t)((float)raw * 0.0054931640625f);

    *pData = next;
    return 1;
}

void JY901P_ReadAllData(JY901P_DataStruct *pData)
{
    (void)JY901P_ReadAllDataSafe(pData);
}
