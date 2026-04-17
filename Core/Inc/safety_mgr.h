#ifndef __SAFETY_MGR_H__
#define __SAFETY_MGR_H__

/* -------------------- 依赖头文件 -------------------- */
#include "app_config.h"   // 系统全局配置头文件，提供超时阈值、板子尺寸、安全边界等宏定义

/* -------------------- 安全检查接口 -------------------- */

/**
 * @brief 检查 IMU 数据是否超时
 * @param now_ms 当前系统时间，单位：毫秒
 * @param imu_ms IMU 最近一次更新时间，单位：毫秒
 * @retval 1=IMU 数据已超时，0=IMU 数据仍有效
 *
 * @note 这个函数用于判断：
 *       “当前系统时间”和“IMU 最近更新时间”之间的差值
 *       是否已经超过配置的 IMU 超时阈值。
 *
 *       常见用途：
 *       - IMU 掉线检测
 *       - 姿态传感器异常保护
 *       - 内环控制前先判断传感器是否可靠
 */
uint8_t SafetyMgr_CheckImuTimeout(uint32_t now_ms, uint32_t imu_ms);

/**
 * @brief 检查树莓派（PI）数据是否超时
 * @param now_ms 当前系统时间，单位：毫秒
 * @param pi_ms  树莓派最近一次更新时间，单位：毫秒
 * @retval 1=树莓派数据已超时，0=树莓派数据仍有效
 *
 * @note 这个函数主要用于视觉数据或命令数据有效性判断。
 *       如果树莓派太久没有发来新数据，
 *       系统就可以判定：
 *       - 小球位置失效
 *       - 目标点失效
 *       - 激光点失效
 *       - 需要进入安全策略
 */
uint8_t SafetyMgr_CheckPiTimeout(uint32_t now_ms, uint32_t pi_ms);

/**
 * @brief 检查小球是否接近平台边缘
 * @param x_mm 小球当前 X 坐标，单位：毫米
 * @param y_mm 小球当前 Y 坐标，单位：毫米
 * @retval 1=小球已接近边缘，0=仍处于安全区域
 *
 * @note 这个函数通常根据：
 *       - 平台总尺寸 BOARD_SIZE_MM
 *       - 边缘安全阈值 BALL_NEAR_EDGE_MM
 *       来判断小球是否已经靠近危险区域。
 *
 *       常见用途：
 *       - 触发减速
 *       - 限制平台倾角
 *       - 触发回中保护
 *       - 防止小球掉出平台
 */
uint8_t SafetyMgr_CheckBallNearEdge(float x_mm, float y_mm);

#endif
