#include "safety_mgr.h"   // 安全管理模块头文件

/**
 * @brief 检查 IMU 数据是否超时
 * @param now_ms 当前系统时间，单位：毫秒
 * @param imu_ms IMU 最近一次有效更新时间，单位：毫秒
 * @retval 1=IMU 数据已超时，0=IMU 数据仍有效
 *
 * @note 判断逻辑：
 *       如果“当前时间 - IMU 最近更新时间”大于 IMU_TIMEOUT_MS，
 *       就认为 IMU 数据已经失效。
 *
 *       常见使用场景：
 *       - IMU 断线检测
 *       - 姿态数据卡死检测
 *       - 内环控制前的安全判断
 *
 *       一旦 IMU 超时，系统通常会进入保护策略，
 *       例如：
 *       - 舵机回中
 *       - 停止闭环控制
 *       - 保持平台安全姿态
 */
uint8_t SafetyMgr_CheckImuTimeout(uint32_t now_ms, uint32_t imu_ms)
{
    return ((now_ms - imu_ms) > IMU_TIMEOUT_MS) ? 1U : 0U;
}

/**
 * @brief 检查树莓派（PI）数据是否超时
 * @param now_ms 当前系统时间，单位：毫秒
 * @param pi_ms  树莓派最近一次有效更新时间，单位：毫秒
 * @retval 1=树莓派数据已超时，0=树莓派数据仍有效
 *
 * @note 判断逻辑：
 *       如果“当前时间 - 树莓派最近更新时间”大于 PI_UART_TIMEOUT_MS，
 *       就认为树莓派通信已经超时。
 *
 *       常见使用场景：
 *       - 小球位置视觉数据失效检测
 *       - 目标点数据失效检测
 *       - 激光追踪数据失效检测
 *       - 串口通信异常检测
 *
 *       一旦树莓派数据超时，系统通常会采取保护动作，
 *       例如：
 *       - 将球状态标记为无效
 *       - 重置外环
 *       - 平台回中
 */
uint8_t SafetyMgr_CheckPiTimeout(uint32_t now_ms, uint32_t pi_ms)
{
    return ((now_ms - pi_ms) > PI_UART_TIMEOUT_MS) ? 1U : 0U;
}

/**
 * @brief 检查小球是否接近平台边缘
 * @param x_mm 小球当前 X 坐标，单位：毫米
 * @param y_mm 小球当前 Y 坐标，单位：毫米
 * @retval 1=小球已接近边缘，0=小球仍在安全区域
 *
 * @note 判断方法：
 *       只要小球当前位置满足以下任意一个条件，就认为已经接近边缘：
 *       - x < BALL_NEAR_EDGE_MM
 *       - y < BALL_NEAR_EDGE_MM
 *       - x > BOARD_SIZE_MM - BALL_NEAR_EDGE_MM
 *       - y > BOARD_SIZE_MM - BALL_NEAR_EDGE_MM
 *
 *       也就是说，板子四周会留出一个“安全边界带”。
 *       只要球进入这条边界带，就触发边缘保护判定。
 *
 *       典型用途：
 *       - 减小平台目标倾角
 *       - 防止小球继续快速冲向边缘
 *       - 避免小球掉出平台
 */
uint8_t SafetyMgr_CheckBallNearEdge(float x_mm, float y_mm)
{
    /* 靠近左边缘 */
    if (x_mm < BALL_NEAR_EDGE_MM) return 1U;

    /* 靠近下边缘 / 上边缘（取决于你的坐标定义） */
    if (y_mm < BALL_NEAR_EDGE_MM) return 1U;

    /* 靠近右边缘 */
    if (x_mm > (BOARD_SIZE_MM - BALL_NEAR_EDGE_MM)) return 1U;

    /* 靠近另一侧边缘 */
    if (y_mm > (BOARD_SIZE_MM - BALL_NEAR_EDGE_MM)) return 1U;

    /* 否则认为球仍在安全区域内 */
    return 0U;
}
