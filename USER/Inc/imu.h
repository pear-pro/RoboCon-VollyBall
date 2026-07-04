#ifndef __IMU_H
#define __IMU_H

#include "main.h"
#include <stdint.h>
#include <string.h>

/************************************************************
 * 宏定�?
************************************************************/

#define IMU_FRAME_HEAD              0x55
#define IMU_FRAME_LEN               11

#define IMU_TYPE_TIME               0x50
#define IMU_TYPE_ACC                0x51
#define IMU_TYPE_GYRO               0x52
#define IMU_TYPE_ANGLE              0x53
#define IMU_TYPE_MAG                0x54
#define IMU_TYPE_PORT               0x55
#define IMU_TYPE_PRESSURE_HEIGHT    0x56
#define IMU_TYPE_GPS                0x57
#define IMU_TYPE_GROUND_SPEED       0x58
#define IMU_TYPE_QUAT               0x59
#define IMU_TYPE_GPS_ACCURACY       0x5A
#define IMU_TYPE_READ               0x5F

#define IMU_FLAG_ACC                (1U << 0)
#define IMU_FLAG_GYRO               (1U << 1)
#define IMU_FLAG_ANGLE              (1U << 2)
#define IMU_FLAG_MAG                (1U << 3)
#define IMU_FLAG_QUAT               (1U << 4)

/************************************************************
 * 数据结构
************************************************************/

typedef struct
{
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    int16_t raw_t;
} IMU_Raw3Axis_t;

typedef struct
{
    float x;
    float y;
    float z;
} IMU_Float3Axis_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} IMU_Angle_t;

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} IMU_Quat_t;

typedef struct
{
    IMU_Raw3Axis_t acc_raw;
    IMU_Raw3Axis_t gyro_raw;
    IMU_Raw3Axis_t angle_raw;
    IMU_Raw3Axis_t mag_raw;
    int16_t quat_raw[4];

    IMU_Float3Axis_t acc_g;        /* 加速度，单�?g */
    IMU_Float3Axis_t gyro_dps;     /* 角速度，单�?°/s */
    IMU_Angle_t angle_deg;         /* 欧拉角，单位 ° */
    IMU_Float3Axis_t mag;          /* 磁场，原始�?*/
    IMU_Quat_t quat;               /* 四元�?*/

    uint32_t frame_count;
    uint32_t crc_error_count;
} IMU_Data_t;

/************************************************************
 * 函数声明
************************************************************/

void IMU_UART_Init(UART_HandleTypeDef *huart);
void IMU_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void IMU_MagCalibration(uint32_t rotate_ms);

void IMU_GetData(IMU_Data_t *data);
uint32_t IMU_GetUpdateFlag(void);
uint32_t IMU_GetAndClearUpdateFlag(void);

extern IMU_Data_t imu;

#endif
