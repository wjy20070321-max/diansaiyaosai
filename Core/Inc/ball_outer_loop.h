/**
 ******************************************************************************
 * @file    ball_outer_loop.h
 * @brief   球位置外环控制模块头文件 (LQR最优控制版)
 ******************************************************************************
 */

#ifndef __BALL_OUTER_LOOP_H__
#define __BALL_OUTER_LOOP_H__

/* 包含应用配置头文件 */
#include "app_config.h"
#include <stdint.h>

/**
 * @brief 球外环控制输出结构体
 * @details 包含X轴和Y轴的倾斜角度参考值，单位为度(°)
 */
typedef struct
{
    /**
     * @brief X轴倾斜角度参考值（度）
     */
    float theta_x_ref_deg;
    
    /**
     * @brief Y轴倾斜角度参考值（度）
     */
    float theta_y_ref_deg;
} BallOuterOutput_t;

/**
 * @brief 球外环控制初始化函数
 */
void BallOuterLoop_Init(void);

/**
 * @brief 球外环控制重置函数
 */
void BallOuterLoop_Reset(void);

/**
 * @brief 球外环控制主运行函数 (LQR控制核心)
 * @param[in]  x_ref      X轴目标位置（毫米），即激光X坐标
 * @param[in]  y_ref      Y轴目标位置（毫米），即激光Y坐标
 * @param[in]  x_meas     X轴当前位置测量值（毫米），即小球X坐标
 * @param[in]  y_meas     Y轴当前位置测量值（毫米），即小球Y坐标
 * @param[in]  vx_meas    X轴当前速度测量值（毫米/秒）
 * @param[in]  vy_meas    Y轴当前速度测量值（毫米/秒）
 * @param[in]  mode       控制模式
 * @param[out] out        输出结构体指针，包含计算得到的倾斜角度参考值
 */
void BallOuterLoop_Run(float x_ref, float y_ref,
                       float x_meas, float y_meas,
                       float vx_meas, float vy_meas,
                       uint8_t mode,
                       BallOuterOutput_t *out);

#endif /* __BALL_OUTER_LOOP_H__ */
