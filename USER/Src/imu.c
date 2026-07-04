#include "imu.h"
#include "main.h"
#include <math.h>

/************************************************************
 * ¨¦????????¨¦??
************************************************************/

static UART_HandleTypeDef *s_imu_uart = NULL;

static uint8_t s_rx_byte = 0;
static uint8_t s_rx_buf[IMU_FRAME_LEN];
static uint8_t s_rx_index = 0;

static IMU_Data_t s_imu_data;
static volatile uint32_t s_update_flag = 0;

#define IMU_FUSION_GYRO_LPF_ALPHA        0.35f
#define IMU_FUSION_YAW_CORRECTION_GAIN   0.02f
#define IMU_GYRO_BIAS_ALPHA              0.001f
#define IMU_GYRO_BIAS_STILL_DPS          2.0f
#define IMU_ACC_STILL_MIN_G2             0.64f
#define IMU_ACC_STILL_MAX_G2             1.44f
#define IMU_GYRO_DT_MAX_S                0.2f

static uint8_t s_fusion_ready = 0U;
static uint32_t s_last_gyro_tick = 0U;
IMU_Data_t imu;

/************************************************************
 * ??¡­¨¦?¡§?????¡ã?¡ê¡ã???
************************************************************/

static void IMU_ParseByte(uint8_t byte);
static void IMU_ParseFrame(uint8_t *frame);
static uint8_t IMU_CheckSum(uint8_t *frame);
static int16_t IMU_GetInt16(uint8_t low, uint8_t high);
static float IMU_NormalizeDeg(float angle);
static uint8_t IMU_IsStill(void);
static void IMU_UpdateGyroFusion(void);
static void IMU_UpdateAngleFusion(void);

/************************************************************
 * ?????¡ã??????IMU_UART_Init
 * ???¨¨??  ???????¡ì???¨C¨¦??¨¨?o??a??2??¡ê??£¤?¡±?
 * ??¡é???  ???huart - ??2??¡ê??£¤??????????|?&huart1
 * ¨¨?¡±??????????¡ª?
************************************************************/
void IMU_UART_Init(UART_HandleTypeDef *huart)
{
    s_imu_uart = huart;

    memset(&s_imu_data, 0, sizeof(IMU_Data_t));
    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    s_rx_index = 0;
    s_update_flag = 0;
    s_fusion_ready = 0U;
    s_last_gyro_tick = 0U;

    HAL_UART_Receive_IT(s_imu_uart, &s_rx_byte, 1);
}

/************************************************************
 * ?????¡ã??????IMU_UART_RxCpltCallback
 * ???¨¨??  ?????2??¡ê??£¤?¡±??????????¨¨¡ã?????¡±???¡ã HAL_UART_RxCpltCallback ¨¦??¨¨¡ã??¡±?
 * ??¡é???  ???huart - HAL???¨¨¡ã?????¡­£¤?????2??¡ê??£¤???
 * ¨¨?¡±??????????¡ª?
************************************************************/
void IMU_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_imu_uart)
    {
        IMU_ParseByte(s_rx_byte);

        HAL_UART_Receive_IT(s_imu_uart, &s_rx_byte, 1);
    }
}

/************************************************************
 * ?????¡ã??????IMU_ParseByte
 * ???¨¨??  ???¨¦???-¡ª¨¨??????????o??£¤?¡±??????¡ì??¡ã???
 * ??¡é???  ???byte - ??¡°?¡ë???£¤?¡±???¡ã????-¡ª¨¨??
 * ¨¨?¡±??????????¡ª?
************************************************************/
static void IMU_ParseByte(uint8_t byte)
{
    if (s_rx_index == 0)
    {
        if (byte == IMU_FRAME_HEAD)
        {
            s_rx_buf[0] = byte;
            s_rx_index = 1;
        }
        return;
    }

    s_rx_buf[s_rx_index] = byte;
    s_rx_index++;

    if (s_rx_index >= IMU_FRAME_LEN)
    {
        if (IMU_CheckSum(s_rx_buf) == s_rx_buf[10])
        {
            IMU_ParseFrame(s_rx_buf);
            s_imu_data.frame_count++;
        }
        else
        {
            s_imu_data.crc_error_count++;
        }

        s_rx_index = 0;
    }
}

/************************************************************
 * ?????¡ã??????IMU_CheckSum
 * ???¨¨??  ???¨¨?????SUMCRC
 * ??¡é???  ???frame - 11?-¡ª¨¨????¡ã??????
 * ¨¨?¡±????????????¨¦a??¡¯????8???
************************************************************/
static uint8_t IMU_CheckSum(uint8_t *frame)
{
    uint16_t sum = 0;

    for (uint8_t i = 0; i < 10; i++)
    {
        sum += frame[i];
    }

    return (uint8_t)(sum & 0xFF);
}

/************************************************************
 * ?????¡ã??????IMU_GetInt16
 * ???¨¨??  ???????-¡ª¨¨????¡§?¡ë????¨¦???-¡ª¨¨????¡§????????????int16_t
 * ??¡é???  ???low  - ??????
 *        high - ¨¦?????
 * ¨¨?¡±???????????????????????¡ë??|???6?????¡ã???
************************************************************/
static int16_t IMU_GetInt16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}
static float IMU_NormalizeDeg(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static uint8_t IMU_IsStill(void)
{
    float acc_g2 = s_imu_data.acc_g.x * s_imu_data.acc_g.x +
                   s_imu_data.acc_g.y * s_imu_data.acc_g.y +
                   s_imu_data.acc_g.z * s_imu_data.acc_g.z;

    if (acc_g2 < IMU_ACC_STILL_MIN_G2 || acc_g2 > IMU_ACC_STILL_MAX_G2)
    {
        return 0U;
    }

    if (fabsf(s_imu_data.gyro_dps.x) > IMU_GYRO_BIAS_STILL_DPS ||
        fabsf(s_imu_data.gyro_dps.y) > IMU_GYRO_BIAS_STILL_DPS ||
        fabsf(s_imu_data.gyro_dps.z) > IMU_GYRO_BIAS_STILL_DPS)
    {
        return 0U;
    }

    return 1U;
}

static void IMU_UpdateGyroFusion(void)
{
    uint32_t now = HAL_GetTick();
    float dt = 0.0f;
    uint8_t is_still;

    if (s_last_gyro_tick != 0U)
    {
        dt = (float)(uint32_t)(now - s_last_gyro_tick) * 0.001f;
    }
    s_last_gyro_tick = now;

    is_still = IMU_IsStill();
    if (is_still)
    {
        s_imu_data.gyro_bias_dps.x += (s_imu_data.gyro_dps.x - s_imu_data.gyro_bias_dps.x) * IMU_GYRO_BIAS_ALPHA;
        s_imu_data.gyro_bias_dps.y += (s_imu_data.gyro_dps.y - s_imu_data.gyro_bias_dps.y) * IMU_GYRO_BIAS_ALPHA;
        s_imu_data.gyro_bias_dps.z += (s_imu_data.gyro_dps.z - s_imu_data.gyro_bias_dps.z) * IMU_GYRO_BIAS_ALPHA;
    }

    s_imu_data.gyro_dps_fused.x += ((s_imu_data.gyro_dps.x - s_imu_data.gyro_bias_dps.x) - s_imu_data.gyro_dps_fused.x) * IMU_FUSION_GYRO_LPF_ALPHA;
    s_imu_data.gyro_dps_fused.y += ((s_imu_data.gyro_dps.y - s_imu_data.gyro_bias_dps.y) - s_imu_data.gyro_dps_fused.y) * IMU_FUSION_GYRO_LPF_ALPHA;
    s_imu_data.gyro_dps_fused.z += ((s_imu_data.gyro_dps.z - s_imu_data.gyro_bias_dps.z) - s_imu_data.gyro_dps_fused.z) * IMU_FUSION_GYRO_LPF_ALPHA;

    if (is_still)
    {
        s_imu_data.gyro_dps_fused.x = 0.0f;
        s_imu_data.gyro_dps_fused.y = 0.0f;
        s_imu_data.gyro_dps_fused.z = 0.0f;
    }

    if (s_fusion_ready && dt > 0.0f && dt < IMU_GYRO_DT_MAX_S)
    {
        s_imu_data.angle_fused_deg.yaw = IMU_NormalizeDeg(s_imu_data.angle_fused_deg.yaw +
                                                          s_imu_data.gyro_dps_fused.z * dt);
    }
}

static void IMU_UpdateAngleFusion(void)
{
    if (!s_fusion_ready)
    {
        s_imu_data.angle_fused_deg = s_imu_data.angle_deg;
        s_fusion_ready = 1U;
        return;
    }

    s_imu_data.angle_fused_deg.roll = s_imu_data.angle_deg.roll;
    s_imu_data.angle_fused_deg.pitch = s_imu_data.angle_deg.pitch;

    if (IMU_IsStill())
    {
        return;
    }

    s_imu_data.angle_fused_deg.yaw = IMU_NormalizeDeg(s_imu_data.angle_fused_deg.yaw +
        IMU_NormalizeDeg(s_imu_data.angle_deg.yaw - s_imu_data.angle_fused_deg.yaw) * IMU_FUSION_YAW_CORRECTION_GAIN);
}

/************************************************************
 * ?????¡ã??????IMU_ParseFrame
 * ???¨¨??  ???¨¨¡ì¡ê????????¡ä?????¡ì??¡ã???
 * ??¡é???  ???frame - 11?-¡ª¨¨????¡ã??????
 * ¨¨?¡±??????????¡ª?
************************************************************/
static void IMU_ParseFrame(uint8_t *frame)
{
    uint8_t type = frame[1];

    int16_t data1 = IMU_GetInt16(frame[2], frame[3]);
    int16_t data2 = IMU_GetInt16(frame[4], frame[5]);
    int16_t data3 = IMU_GetInt16(frame[6], frame[7]);
    int16_t data4 = IMU_GetInt16(frame[8], frame[9]);

    switch (type)
    {
        case IMU_TYPE_ACC:
        {
            s_imu_data.acc_raw.raw_x = data1;
            s_imu_data.acc_raw.raw_y = data2;
            s_imu_data.acc_raw.raw_z = data3;
            s_imu_data.acc_raw.raw_t = data4;

            /*
             * ???¨¨¡ì???¡ä?¡ë1/JY901???¨¨?????
             * ???¨¦???o|¨¦???¡§? ?¡À16g????¡¥1?o?int16_t ???¨¦???¡§??¡À32768
             */
            s_imu_data.acc_g.x = (float)data1 / 32768.0f * 16.0f;
            s_imu_data.acc_g.y = (float)data2 / 32768.0f * 16.0f;
            s_imu_data.acc_g.z = (float)data3 / 32768.0f * 16.0f;

            s_update_flag |= IMU_FLAG_ACC;
            break;
        }

        case IMU_TYPE_GYRO:
        {
            s_imu_data.gyro_raw.raw_x = data1;
            s_imu_data.gyro_raw.raw_y = data2;
            s_imu_data.gyro_raw.raw_z = data3;
            s_imu_data.gyro_raw.raw_t = data4;

            /*
             * ???¨¨¡ì???¡ä?¡ë1/JY901???¨¨?????
             * ¨¨¡ì¡¯¨¦???o|¨¦???¡§? ?¡À2000?¡ã/s????¡¥1?o?int16_t ???¨¦???¡§??¡À32768
             */
            s_imu_data.gyro_dps.x = (float)data1 / 32768.0f * 2000.0f;
            s_imu_data.gyro_dps.y = (float)data2 / 32768.0f * 2000.0f;
            s_imu_data.gyro_dps.z = (float)data3 / 32768.0f * 2000.0f;
            IMU_UpdateGyroFusion();

            s_update_flag |= IMU_FLAG_GYRO;
            break;
        }

        case IMU_TYPE_ANGLE:
        {
            s_imu_data.angle_raw.raw_x = data1;
            s_imu_data.angle_raw.raw_y = data2;
            s_imu_data.angle_raw.raw_z = data3;
            s_imu_data.angle_raw.raw_t = data4;

            /*
             * ???¨¨¡ì???¡ä?¡ë1/JY901???¨¨?????
             * ¨¨¡ì¡¯?o|¨¦???¡§? ?¡À180?¡ã????¡¥1?o?int16_t ???¨¦???¡§??¡À32768
             */
            s_imu_data.angle_deg.roll  = (float)data1 / 32768.0f * 180.0f;
            s_imu_data.angle_deg.pitch = (float)data2 / 32768.0f * 180.0f;
            s_imu_data.angle_deg.yaw   = (float)data3 / 32768.0f * 180.0f;
            IMU_UpdateAngleFusion();

            s_update_flag |= IMU_FLAG_ANGLE;
            break;
        }

        case IMU_TYPE_MAG:
        {
            s_imu_data.mag_raw.raw_x = data1;
            s_imu_data.mag_raw.raw_y = data2;
            s_imu_data.mag_raw.raw_z = data3;
            s_imu_data.mag_raw.raw_t = data4;

            s_imu_data.mag.x = (float)data1;
            s_imu_data.mag.y = (float)data2;
            s_imu_data.mag.z = (float)data3;

            s_update_flag |= IMU_FLAG_MAG;
            break;
        }

        case IMU_TYPE_QUAT:
        {
            s_imu_data.quat_raw[0] = data1;
            s_imu_data.quat_raw[1] = data2;
            s_imu_data.quat_raw[2] = data3;
            s_imu_data.quat_raw[3] = data4;

            /*
             * ???¨¨¡ì?????¡­???¡ã????¡±????
             * q = raw / 32768
             */
            s_imu_data.quat.q0 = (float)data1 / 32768.0f;
            s_imu_data.quat.q1 = (float)data2 / 32768.0f;
            s_imu_data.quat.q2 = (float)data3 / 32768.0f;
            s_imu_data.quat.q3 = (float)data4 / 32768.0f;

            s_update_flag |= IMU_FLAG_QUAT;
            break;
        }

        default:
        {
            /*
             * 0x50 ?¡ª?¨¦¡ª¡ä???x55 ??¡¥??¡ê?????????x56 ?¡ã¡±???¨¦???o|???
             * 0x57 ????o??o|???x58 ??¡ã¨¦?????x5A GPS?2??o|?-¡ë???
             * ¨¨??¨¦???¡­????????¡­¡¤??¡°??¡é??¡ª?????a????????£¤?¡±????¨¦a????
             */
            break;
        }
    }
}

void IMU_ResetFusion(void)
{
    __disable_irq();
    s_imu_data.gyro_dps_fused.x = 0.0f;
    s_imu_data.gyro_dps_fused.y = 0.0f;
    s_imu_data.gyro_dps_fused.z = 0.0f;
    s_imu_data.gyro_bias_dps.x = 0.0f;
    s_imu_data.gyro_bias_dps.y = 0.0f;
    s_imu_data.gyro_bias_dps.z = 0.0f;
    s_imu_data.angle_fused_deg = s_imu_data.angle_deg;
    s_fusion_ready = 1U;
    s_last_gyro_tick = HAL_GetTick();
    __enable_irq();
}
/************************************************************
 * ?????¡ã??????IMU_GetData
 * ???¨¨??  ???¨¨?¡¤??¨C??¡°?¡ë?¨¨¡ì¡ê???????????¡ã???
 * ??¡é???  ???data - ??¡ã???¨¨?¡°??o???¨¦¡¯?
 * ¨¨?¡±??????????¡ª?
************************************************************/
void IMU_GetData(IMU_Data_t *data)
{
    if (data == NULL)
    {
        return;
    }

    __disable_irq();
    memcpy(data, &s_imu_data, sizeof(IMU_Data_t));
    __enable_irq();
}

/************************************************************
 * ?????¡ã??????IMU_GetUpdateFlag
 * ???¨¨??  ???¨¨¡¥???¨C??¡ã?????¡ä?¨C¡ã??????
 * ??¡é???  ????¡ª?
 * ¨¨?¡±???????????¡ä?¨C¡ã?????¡ª
************************************************************/
uint32_t IMU_GetUpdateFlag(void)
{
    return s_update_flag;
}

/************************************************************
 * ?????¡ã??????IMU_GetAndClearUpdateFlag
 * ???¨¨??  ???¨¨¡¥???¨C?1???¡­¨¦?¡è??¡ã?????¡ä?¨C¡ã?????¡ª
 * ??¡é???  ????¡ª?
 * ¨¨?¡±???????????¡ä?¨C¡ã?????¡ª
************************************************************/
uint32_t IMU_GetAndClearUpdateFlag(void)
{
    uint32_t flag;

    __disable_irq();
    flag = s_update_flag;
    s_update_flag = 0;
    __enable_irq();

    return flag;
}