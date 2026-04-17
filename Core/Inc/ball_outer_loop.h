#ifndef __BALL_OUTER_LOOP_H__
#define __BALL_OUTER_LOOP_H__

/* -------------------- 依赖头文件 -------------------- */
#include "app_config.h"   // 系统全局配置头文件
#include <stdint.h>       // 标准整型定义

/* -------------------- 外环输出结构体 -------------------- */
/**
 * @brief 小球位置外环控制输出结构体
 * @note 该结构体保存外环控制器计算得到的平台目标倾角
 *
 *       含义是：
 *       - 外环根据“小球当前位置”和“目标位置”
 *       - 计算出平台在 X / Y 两个方向应该倾斜多少度
 *       - 再把这个目标倾角交给内环去继续控制舵机
 */
typedef struct
{
    /**
     * @brief 平台 X 轴目标倾角，单位：度
     */
    float theta_x_ref_deg;

    /**
     * @brief 平台 Y 轴目标倾角，单位：度
     */
    float theta_y_ref_deg;
} BallOuterOutput_t;

/* -------------------- 外环控制模块接口 -------------------- */

/**
 * @brief 小球位置外环控制器初始化函数
 * @note 用于初始化外环控制器内部状态
 */
void BallOuterLoop_Init(void);

/**
 * @brief 小球位置外环控制器重置函数
 * @note 用于清空外环控制器历史状态
 *
 *       常见用途：
 *       - 切换任务时重置
 *       - 球丢失时重置
 *       - 避免旧目标历史导数影响新的控制过程
 */
void BallOuterLoop_Reset(void);

/**
 * @brief 小球位置外环主运行函数
 * @param[in]  x_ref     X 方向目标位置，单位：毫米
 * @param[in]  y_ref     Y 方向目标位置，单位：毫米
 * @param[in]  x_meas    X 方向当前位置测量值，单位：毫米
 * @param[in]  y_meas    Y 方向当前位置测量值，单位：毫米
 * @param[in]  vx_meas   X 方向当前速度测量值，单位：毫米/秒
 * @param[in]  vy_meas   Y 方向当前速度测量值，单位：毫米/秒
 * @param[in]  mode      当前控制模式
 * @param[out] out       输出结构体指针，返回平台目标倾角
 *
 * @note 这个函数是外环控制核心。
 *       它的主要作用是：
 *       1. 根据目标位置和当前小球位置计算位置误差
 *       2. 结合当前小球速度进行动态修正
 *       3. 按控制模式限制平台最大允许倾角
 *       4. 输出平台 X / Y 两个方向的目标倾角
 *
 *       这些目标倾角会继续送给内环控制器，
 *       内环再根据 IMU 姿态去计算舵机命令角。
 */
void BallOuterLoop_Run(float x_ref, float y_ref,
                       float x_meas, float y_meas,
                       float vx_meas, float vy_meas,
                       uint8_t mode,
                       BallOuterOutput_t *out);

#endif /* __BALL_OUTER_LOOP_H__ */
