#include "includes.h"
#define POSITION_PID 1 // 位置式
#define DELTA_PID 2	   // 增量式
#define PID_MODE 1

#define ENCODER_PER_CIRCLE 8192    // RM3508编码器每圈脉冲数（核心，适配电机）
#define NOW_ERR     0               // 补充原代码缺失的误差索引宏
#define LAST_ERR    1
#define LLAST_ERR   2

typedef struct {
    int16_t last_encoder;   // 上一次的原始编码器值（0-8191）
    float total_angle;      // 多圈累计角度（编码器值，核心：圈数*8192 + 实时值）
    int32_t circle_cnt;     // 圈数计数（正向+，反向-，方便调试）
} EncoderCircleTypeDef;

/**
 * @brief 将浮点数限制在指定的绝对值范围内
 *
 * @param x 指向要限制的浮点数的指针
 * @param limit 限制的绝对值边界
 *
 * @note 此函数会修改*x的值，使其保持在[-limit, limit]范围内
 */
static void abs_limit(float *x, int32_t limit)
{
	/* 如果值超过正边界，则设置为正边界值 */
	if (*x > limit)
		*x = limit;
	/* 如果值超过负边界，则设置为负边界值 */
	if (*x < -limit)
		*x = -limit;
}

/**
 * @brief 初始化PID控制器参数
 * @param pid 指向PID控制器结构体的指针
 * @param p 比例系数
 * @param i 积分系数
 * @param d 微分系数
 * @param max_out 输出最大值限制
 * @param integral_limit 积分限幅值
 */
void pid_init(pid_t *pid, float p, float i, float d, int32_t max_out, int32_t integral_limit)
{
	/* 设置PID控制器的基本参数 */
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->maxout = max_out;
	pid->integral_limit = integral_limit;

	/* 设置输出死区为5 */
	pid->output_deadband = 5;
}

/**
 * @brief 重置PID控制器参数
 * @param pid 指向PID控制器结构体的指针
 * @param p 比例系数
 * @param i 积分系数
 * @param d 微分系数
 * @return 无返回值
 *
 * 该函数用于初始化或重置PID控制器的参数，将PID系数和输出值清零
 */
void pid_reset(pid_t *pid, float p, float i, float d)
{
	/* 设置PID三个参数 */
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;

	/* 初始化PID各项输出为0 */
	pid->pout = 0;
	pid->iout = 0;
	pid->dout = 0;
	pid->out = 0;
}

/**
 * @brief 处理编码器过零（溢出），计算多圈累计角度
 * @param encoder_data 编码器多圈数据结构体指针
 * @param current_encoder 当前原始编码器值（0-8191）
 * @return 多圈累计角度（编码器值）
 */
float encoder_circle_calc(EncoderCircleTypeDef *encoder_data, int16_t current_encoder)
{
    // 1. 获取上一次编码器值
    int16_t last_encoder = encoder_data->last_encoder;
    // 2. 计算原始角度差（可能溢出）
    int32_t delta = (int32_t)current_encoder - (int32_t)last_encoder;

    // 3. 过零（溢出）修正：以半圈（4096）为阈值
    if (delta > ENCODER_PER_CIRCLE / 2) {
        delta -= ENCODER_PER_CIRCLE;
        encoder_data->circle_cnt--; // 反向转1圈
    } else if (delta < -ENCODER_PER_CIRCLE / 2) {
        delta += ENCODER_PER_CIRCLE;
        encoder_data->circle_cnt++; // 正向转1圈
    }

    // 4. 更新累计总角度（编码器值）
    encoder_data->total_angle += delta;
    // 5. 保存本次编码器值给下次用
    encoder_data->last_encoder = current_encoder;

    return encoder_data->total_angle;
}

/**
 * @brief 角度值（度）转编码器值（适配RM3508：1圈=8192编码器值=360度）
 */
float degree_to_encoder(float degree)
{
    return (degree / 360.0f) * ENCODER_PER_CIRCLE;
}

/**
 * @brief 编码器值转角度值（方便调试查看多圈角度）
 */
float encoder_to_degree(float encoder)
{
    return (encoder / ENCODER_PER_CIRCLE) * 360.0f;
}

/**
 * PID控制器计算函数
 * 根据当前值和设定值计算PID输出
 *
 * @param pid PID控制器结构体指针，包含PID参数和状态信息
 * @param get 当前反馈值（实际测量值）
 * @param set 目标设定值
 * @return PID控制器输出值，如果在死区范围内则返回0
 */
float pid_calc(pid_t *pid, float get, float set)
{
	// 更新当前值和设定值
	pid->get = get;
	pid->set = set;
	// 计算当前误差值
	pid->error[NOW_ERR] = set - get;

#if (PID_MODE == POSITION_PID)
	// 位置式PID计算
	pid->pout = pid->kp * pid->error[NOW_ERR];							// 比例项计算
	pid->iout += pid->ki * pid->error[NOW_ERR];							// 积分项累加计算
	pid->dout = pid->kd * (pid->error[NOW_ERR] - pid->error[LAST_ERR]); // 微分项计算

	abs_limit(&(pid->iout), pid->integral_limit); // 积分项限幅
	pid->out = pid->pout + pid->iout + pid->dout; // PID输出合成
	abs_limit(&(pid->out), pid->maxout);		  // 输出值限幅
#elif (PID_MODE == DELTA_PID)
	// 增量式PID计算
	pid->pout = pid->kp * (pid->error[NOW_ERR] - pid->error[LAST_ERR]);							// 比例项计算
	pid->iout = pid->ki * pid->error[NOW_ERR];													// 积分项计算
	pid->dout = pid->kd * (pid->error[NOW_ERR] * pid->error[LAST_ERR] + pid->error[LLAST_ERR]); // 微分项计算

	pid->out += pid->pout + pid->iout + pid->dout; // 增量式输出累加
	abs_limit(&(pid->out), pid->maxout);		   // 输出值限幅
#endif

	// 更新历史误差值，用于下次计算
	pid->error[LLAST_ERR] = pid->error[LAST_ERR];
	pid->error[LAST_ERR] = pid->error[NOW_ERR];

	// 死区处理：如果输出值在死区范围内则返回0
	if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
		return 0;
	else
		return pid->out;
}



//  if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
//    return 0;
//  else

/**
 * @brief PID结构体初始化函数
 * @param pid 指向PID结构体的指针
 * @param p 比例系数
 * @param i 积分系数
 * @param d 微分系数
 * @param max_out 输出值上限
 * @param integral_limit 积分限幅值
 * @param init_status 初始化状态标识，决定初始化方式
 * @return 无返回值
 *
 * 该函数根据初始化状态标识来初始化PID控制器，支持两种初始化模式：
 * 1. 正常初始化模式：同时初始化和复位PID参数
 * 2. 调试初始化模式：仅初始化PID参数
 */
void PID_Struct_Init(pid_t *pid, float p, float i, float d, int32_t max_out, int32_t integral_limit, INIT_STATUS init_status)
{
	/* 正常初始化模式 */
	if (init_status == INIT)
	{
		pid->f_pid_init = pid_init;
		pid->f_pid_reset = pid_reset;

		pid->f_pid_init(pid, p, i, d, max_out, integral_limit);
		pid->f_pid_reset(pid, p, i, d);
	}
	else
	{
		/* 调试初始化模式 */
		pid->f_pid_init = pid_init;
		pid->f_pid_init(pid, p, i, d, max_out, integral_limit);
	}
}
int16_t PID_PROCESS_Double(pid_t *pid_Angle, pid_t *pid_speed, 
                           EncoderCircleTypeDef *encoder_data, // 新增：编码器多圈数据
                           float target_degree,               // 修改：目标角度（度，多圈）
                           int16_t current_encoder,           // 修改：输入原始编码器值
                           float speed_get)
{
    // 步骤1：处理编码器过零，计算多圈累计角度（编码器值）
    float total_angle_encoder = encoder_circle_calc(encoder_data, current_encoder);

    // 步骤2：将目标角度（度）转为编码器值（适配多圈）
    float target_encoder = degree_to_encoder(target_degree);

    // 步骤3：角度环PID计算（输入多圈累计角度）
    pid_calc(pid_Angle, total_angle_encoder, target_encoder);

    // 步骤4：速度环PID计算（角度环输出作为速度环目标）
    pid_calc(pid_speed, speed_get, pid_Angle->out);

    return (int16_t)pid_speed->out;
}

