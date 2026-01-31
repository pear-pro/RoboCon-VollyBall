#ifndef __PID_3AXIS_H
#define __PID_3AXIS_H

#include "stm32f4xx_hal.h"
#include "JY901P.h"

// X/Y轴PID结构体（位置式）
typedef struct {
    float SetVal;     // 设定值（水平基准0°）
    float NowVal;     // 当前值（滤波后的角度）
    float Err;        // 本次偏差
    float ErrLast;    // 上一次偏差
    float P;          // 比例系数
    float I;          // 积分系数
    float D;          // 微分系数
    float Iout;       // 积分输出
    float I_Limit;    // 积分限幅阈值
    float PidOut;     // PID总输出
} PID_XY_Typedef;

// Z轴PID结构体（带角度跳变修正和滤波）
typedef struct {
    float P;          // 比例系数
    float I;          // 积分系数
    float D;          // 微分系数
    float Error;      // 当前误差
    float LastError;  // 上一次误差
    float SumError;   // 积分累计误差
    float MaxOutput;  // 输出限幅最大值
    float Output;     // PID输出
} PID_Z_Typedef;

// 全局PID变量声明
extern PID_XY_Typedef PID_X;
extern PID_XY_Typedef PID_Y;
extern PID_Z_Typedef PID_Z;

// Z轴相关全局变量
extern float rz_real;
extern float rz_filtered;
extern float rz_target;

// 函数声明
void PID_X_Init(void);
void PID_Y_Init(void);
void PID_Z_Init(float P, float I, float D, float MaxOutput);
float PID_X_Calc(float NowVal);
float PID_Y_Calc(float NowVal);
void Z_Filter_PID_Calc(void);
void Z_Axis_Cali(void);

#endif

