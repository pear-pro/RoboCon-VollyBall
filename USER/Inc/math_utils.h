#ifndef MATH_UTILS_H
#define MATH_UTILS_H

// 浮点型极小值，用于判断差值是否接近零（避免除零错误）
#define FLOAT_EPSILON 1e-6f

/**
 * @brief  最大限幅函数（上限位幅）
 * @param  value: 输入需要限幅的数值
 * @param  max_limit: 最大限制阈值（上限）
 * @retval 若value超过max_limit，返回max_limit；否则返回原value
 */
float clamp_max(float value, float max_limit);

/**
 * @brief  最小限幅函数（下限位幅）
 * @param  value: 输入需要限幅的数值
 * @param  min_limit: 最小限制阈值（下限）
 * @retval 若value低于min_limit，返回min_limit；否则返回原value
 */
float clamp_min(float value, float min_limit);

/**
 * @brief  数据归一化函数（映射到[0, 1]区间，保持向下兼容）
 * @param  value: 输入需要归一化的数值
 * @param  data_min: 数据系列的最小值（原始区间下限）
 * @param  data_max: 数据系列的最大值（原始区间上限）
 * @retval 归一化结果；若data_min≈data_max，返回0.0f避免除零错误
 */
float normalize(float value, float data_min, float data_max);

/**
 * @brief  数据归一化函数（映射到任意正负目标区间，新增核心功能）
 * @param  value: 输入需要归一化的数值
 * @param  src_min: 原始数据区间的最小值
 * @param  src_max: 原始数据区间的最大值
 * @param  dst_min: 目标区间的最小值（支持负数，如-1.0f）
 * @param  dst_max: 目标区间的最大值（支持负数，如5.0f）
 * @retval 映射到目标区间的归一化结果；若src_min≈src_max，返回0.0f避免除零错误
 */
float normalize_to_range(float value, float src_min, float src_max, float dst_min, float dst_max);

/**
 * @brief  整型绝对值函数
 * @param  num: 输入整型数值
 * @retval 输入数值的绝对值
 */
int abs_int(int num);

/**
 * @brief  浮点型绝对值函数（适配STM32，无需依赖math.h的fabsf）
 * @param  value: 输入浮点型数值
 * @retval 输入数值的绝对值
 */
float abs_float(float value);

#endif /* MATH_UTILS_H */