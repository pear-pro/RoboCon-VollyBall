#include "pid_3axis.h"

// 全局PID变量初始化
PID_XY_Typedef PID_X = {
    .SetVal = 0.0f,
    .NowVal = 0.0f,
    .Err = 0.0f,
    .ErrLast = 0.0f,
    .P = -0.9f,
    .I = -0.00f,
    .D = 0.0f,
    .Iout = 0.0f,
    .I_Limit = 8.0f,
    .PidOut = 0.0f
};

PID_XY_Typedef PID_Y = {
    .SetVal = 0.0f,
    .NowVal = 0.0f,
    .Err = 0.0f,
    .ErrLast = 0.0f,
    .P = -0.9f,
    .I = 0.0f,
    .D = 0.0f,
    .Iout = 0.0f,
    .I_Limit = 8.0f,
    .PidOut = 0.0f
};

PID_Z_Typedef PID_Z = {
    .P = 0.0f,
    .I = 0.0f,
    .D = 0.0f,
    .Error = 0.0f,
    .LastError = 0.0f,
    .SumError = 0.0f,
    .MaxOutput = 0.0f,
    .Output = 0.0f
};

// Z轴全局变量
float rz_real = 0.0f;
float rz_filtered = 0.0f;
float rz_target = 0.0f;
static float rz_last = 0.0f;
static float rz_corr = 0.0f;

// X轴PID初始化
void PID_X_Init(void)
{
    PID_X.SetVal = 0.0f;
    PID_X.NowVal = 0.0f;
    PID_X.Err = 0.0f;
    PID_X.ErrLast = 0.0f;
    PID_X.Iout = 0.0f;
    PID_X.PidOut = 0.0f;
}

// Y轴PID初始化
void PID_Y_Init(void)
{
    PID_Y.SetVal = 0.0f;
    PID_Y.NowVal = 0.0f;
    PID_Y.Err = 0.0f;
    PID_Y.ErrLast = 0.0f;
    PID_Y.Iout = 0.0f;
    PID_Y.PidOut = 0.0f;
}

// Z轴PID初始化
void PID_Z_Init(float P, float I, float D, float MaxOutput)
{
    PID_Z.P = P;
    PID_Z.I = I;
    PID_Z.D = D;
    PID_Z.MaxOutput = MaxOutput;
    PID_Z.Error = 0.0f;
    PID_Z.LastError = 0.0f;
    PID_Z.SumError = 0.0f;
    PID_Z.Output = 0.0f;
}

// X轴PID计算
float PID_X_Calc(float NowVal)
{
    PID_X.NowVal = NowVal;
    PID_X.Err = PID_X.SetVal - PID_X.NowVal;

    // 比例环节
    float Pout = PID_X.P * PID_X.Err;

    // 积分环节+限幅
    PID_X.Iout += PID_X.I * PID_X.Err;
    if (PID_X.Iout > PID_X.I_Limit) PID_X.Iout = PID_X.I_Limit;
    if (PID_X.Iout < -PID_X.I_Limit) PID_X.Iout = -PID_X.I_Limit;

    // 微分环节
    float Dout = PID_X.D * (PID_X.Err - PID_X.ErrLast);

    // 总输出
    PID_X.PidOut = Pout + PID_X.Iout + Dout;

    // 更新上一次偏差
    PID_X.ErrLast = PID_X.Err;

    return PID_X.PidOut;
}

// Y轴PID计算
float PID_Y_Calc(float NowVal)
{
    PID_Y.NowVal = NowVal;
    PID_Y.Err = PID_Y.SetVal - PID_Y.NowVal;

    // 比例环节
    float Pout = PID_Y.P * PID_Y.Err;

    // 积分环节+限幅
    PID_Y.Iout += PID_Y.I * PID_Y.Err;
    if (PID_Y.Iout > PID_Y.I_Limit) PID_Y.Iout = PID_Y.I_Limit;
    if (PID_Y.Iout < -PID_Y.I_Limit) PID_Y.Iout = -PID_Y.I_Limit;

    // 微分环节
    float Dout = PID_Y.D * (PID_Y.Err - PID_Y.ErrLast);

    // 总输出
    PID_Y.PidOut = Pout + PID_Y.Iout + Dout;

    // 更新上一次偏差
    PID_Y.ErrLast = PID_Y.Err;

    return PID_Y.PidOut;
}

// Z轴校准：设置当前角度为目标值
void Z_Axis_Cali(void)
{
    JY901P_DataStruct gyro_data;
    JY901P_ReadAllData(&gyro_data);
    rz_target = (float)gyro_data.Angle_Z / 32768.0f * 180.0f;
    rz_target = -rz_target;
}

// Z轴滤波+PID计算（自动读取角度并处理跳变）
void Z_Filter_PID_Calc(void)
{
    JY901P_DataStruct gyro_data;
    JY901P_ReadAllData(&gyro_data);

    // 1. 原始值转真实角度
    rz_real = (float)gyro_data.Angle_Z / 32768.0f * 180.0f;
    rz_real = -rz_real;

    // 2. ±180°跳变修正
    float angle_diff = rz_real - rz_last;
    if (angle_diff > 180.0f)
    {
        rz_corr = rz_real - 360.0f;
    }
    else if (angle_diff < -180.0f)
    {
        rz_corr = rz_real + 360.0f;
    }
    else
    {
        rz_corr = rz_real;
    }
    rz_last = rz_real;

    // 3. 互补滤波
    rz_filtered = 0.2 * rz_corr + (1 - 0.2) * rz_filtered;

    // 4. 最短路径误差计算
    float raw_error = rz_target - rz_filtered;
    if (raw_error > 180.0f)
    {
        raw_error -= 360.0f;
    }
    else if (raw_error < -180.0f)
    {
        raw_error += 360.0f;
    }

    // 5. PID计算
    PID_Z.Error = raw_error;
    PID_Z.Output = PID_Z.P * raw_error + PID_Z.I * PID_Z.SumError + PID_Z.D * (raw_error - PID_Z.LastError);

    // 6. 输出限幅
    if (PID_Z.Output > PID_Z.MaxOutput) PID_Z.Output = PID_Z.MaxOutput;
    else if (PID_Z.Output < -PID_Z.MaxOutput) PID_Z.Output = -PID_Z.MaxOutput;

    // 7. 积分限幅
    PID_Z.SumError += raw_error;
    if (PID_Z.SumError > 100.0f) PID_Z.SumError = 100.0f;
    else if (PID_Z.SumError < -100.0f) PID_Z.SumError = -100.0f;

    // 8. 更新上一次误差
    PID_Z.LastError = raw_error;
}

