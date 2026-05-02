#ifndef __SBUS_H__
#define __SBUS_H__
#include "main.h"
#include <stdint.h>

#define SBUS_BUFLEN 25//SBUS协议数据缓存25bit（定义SBUS协议一帧数据的长度，用于存储）
#define SBUS_HUART       huart1//SBUS协议使用的UART1

#define SBUS_RX_BUF_NUM 50u//DMA双缓冲区
#define RC_FRAME_LENGTH 25u//SBUS协议数据缓存25bit（定义遥控器一帧数据的有效长度，用于解析）

#define SBUS_CH_VALUE_MIN 192
#define SBUS_CH_VALUE_MID 992
#define SBUS_CH_VALUE_MAX 1792
#define SBUS_CH_VALUE_OFFSET 992

#define SWITCH_SBUS_CH_VALUE_MIN -800
#define SWITCH_SBUS_CH_VALUE_MID 0
#define SWITCH_SBUS_CH_VALUE_MAX 800


typedef enum
{
    REMOTE_SW_UP   = 1,
    REMOTE_SW_MID  = 3,
    REMOTE_SW_DOWN = 2,
} remote_switch_pos_t;

typedef struct
{
    int16_t ch[SBUS_MAX_CH_NUM];
    uint8_t ch_count;
} sbus_data_t;

typedef struct
{
  int (*open)(remote_device_t *dev);
  int (*decode)(remote_device_t *dev, const uint8_t *frame,uint16_t len);
  void (*key_update)(remote_device_t *dev);
  void (*apply_control)(remote_device_t *dev);
  void (*close)(remote_device_t *dev);
}remote_ops_t;

typedef struct
{

    const char *name;
    uint8_t frame_len;
    uint8_t dma_buf_len;
    float deadzone;
    sbus_data_t sbus;
       // 按键状态记录
    uint16_t last_swa_state;        // SWA上次状态
    uint16_t last_swb_state;        // SWB上次状态
    uint16_t last_swc_state;        // SWC上次状态
    uint16_t last_swd_state;        // SWD上次状态
    uint16_t last_se_state;        // SE上次状态
    uint16_t last_sf_state;        // SF上次状态
    uint16_t key_flag;              // 当前按键标志
    
    const remote_ops_t *ops; 
}remote_device_t;

int remote_open(remote_device_t *dev);
int remote_decode(remote_device_t *dev, const uint8_t *frame, uint16_t len);
void remote_update_key(remote_device_t *dev);
void remote_apply_control(remote_device_t *dev);
void remote_close(remote_device_t *dev);

#endif