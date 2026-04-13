#include "safety_mgr.h"  // 安全管理模块头文件

/**
 * @brief 检查IMU数据是否超时
 * @param now_ms 当前时间（毫秒）
 * @param imu_ms IMU最后更新时间（毫秒）
 * @return 1表示超时，0表示正常
 * 
 * @note 如果IMU数据长时间未更新，可能表示传感器故障或通信问题
 */
uint8_t SafetyMgr_CheckImuTimeout(uint32_t now_ms, uint32_t imu_ms)
{
    return ((now_ms - imu_ms) > IMU_TIMEOUT_MS) ? 1U : 0U;
}

/**
 * @brief 检查与树莓派(PI)通信是否超时
 * @param now_ms 当前时间（毫秒）
 * @param pi_ms 最后一次与PI通信时间（毫秒）
 * @return 1表示超时，0表示正常
 * 
 * @note 如果与PI通信长时间无响应，可能表示通信链路故障
 */
uint8_t SafetyMgr_CheckPiTimeout(uint32_t now_ms, uint32_t pi_ms)
{
    return ((now_ms - pi_ms) > PI_UART_TIMEOUT_MS) ? 1U : 0U;
}

/**
 * @brief 检查球是否接近板子边缘
 * @param x_mm 球的X坐标（毫米）
 * @param y_mm 球的Y坐标（毫米）
 * @return 1表示接近边缘，0表示在安全区域内
 * 
 * @note 使用BALL_NEAR_EDGE_MM作为判断阈值
 * @note 板子尺寸由BOARD_SIZE_MM定义
 */
uint8_t SafetyMgr_CheckBallNearEdge(float x_mm, float y_mm)
{
    if (x_mm < BALL_NEAR_EDGE_MM) return 1U;
    if (y_mm < BALL_NEAR_EDGE_MM) return 1U;
    if (x_mm > (BOARD_SIZE_MM - BALL_NEAR_EDGE_MM)) return 1U;
    if (y_mm > (BOARD_SIZE_MM - BALL_NEAR_EDGE_MM)) return 1U;
    return 0U;
}
