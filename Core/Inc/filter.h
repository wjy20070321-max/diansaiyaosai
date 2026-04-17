#ifndef __FILTER_H__
#define __FILTER_H__

/* -------------------- 依赖头文件 -------------------- */
#include "app_config.h"   // 系统配置头文件，提供基础类型和通用宏

/* -------------------- 一阶低通滤波器结构体 -------------------- */
/**
 * @brief 一阶低通滤波器对象
 * @note 用于对输入信号进行平滑处理，抑制突变和高频噪声
 *
 *       常见用途：
 *       - 平滑传感器数据
 *       - 平滑速度估计值
 *       - 平滑控制器输出
 */
typedef struct
{
    float alpha;      // 滤波系数，范围通常在 0~1，越大响应越快，越小越平滑
    float y;          // 当前滤波输出值
    uint8_t inited;   // 初始化标志：1=已初始化，0=未初始化
} LPF1_t;

/* -------------------- 一阶低通滤波器接口 -------------------- */
/**
 * @brief 初始化一阶低通滤波器
 * @param f     滤波器对象指针
 * @param alpha 滤波系数
 *
 * @note 一般在系统启动时调用一次。
 *       alpha 越大，输出越跟随输入；
 *       alpha 越小，输出越平滑但响应更慢。
 */
void LPF1_Init(LPF1_t *f, float alpha);

/**
 * @brief 运行一阶低通滤波器
 * @param f 滤波器对象指针
 * @param x 当前输入值
 * @retval 滤波后的输出值
 *
 * @note 每来一个新采样值，就调用一次这个函数。
 *       它会根据前一次输出值和当前输入值，计算新的平滑输出。
 */
float LPF1_Run(LPF1_t *f, float x);

/* -------------------- 通用浮点工具函数 -------------------- */
/**
 * @brief 浮点数限幅函数
 * @param x    输入值
 * @param minv 最小允许值
 * @param maxv 最大允许值
 * @retval 限幅后的结果
 *
 * @note 如果 x 小于 minv，则返回 minv；
 *       如果 x 大于 maxv，则返回 maxv；
 *       否则返回原值。
 */
float Clampf(float x, float minv, float maxv);

/**
 * @brief 浮点数死区处理函数
 * @param x    输入值
 * @param band 死区带宽
 * @retval 处理后的结果
 *
 * @note 当输入值绝对值小于死区范围时，输出 0；
 *       常用于：
 *       - 去除微小噪声
 *       - 防止控制器对很小误差过于敏感
 *       - 减少抖动
 */
float Deadbandf(float x, float band);

#endif
