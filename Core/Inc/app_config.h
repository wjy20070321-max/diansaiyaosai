/**
 ******************************************************************************
 * @file    app_config.h
 * @brief   球盘控制系统应用配置头文件
 * @details 包含系统所有配置参数、常量定义和宏定义
 ******************************************************************************
 */

#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

/* 包含必要的系统头文件 */
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

/* C++兼容性声明 */
#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 物理尺寸配置 ==================== */

/**
 * @brief 平台有效尺寸（毫米）
 * @note  按新题默认使用 600mm × 600mm
 *       如果你们机械实物不是 600，请同步修改 BOARD_CENTER 和 region.c
 */
#define BOARD_SIZE_MM                   600.0f

/**
 * @brief 目标区域直径（毫米）
 * @note  新题要求目标区域外径 4cm
 */
#define REGION_DIAMETER_MM              40.0f

/**
 * @brief 目标区域半径（毫米）
 */
#define REGION_RADIUS_MM                (REGION_DIAMETER_MM * 0.5f)

/**
 * @brief 球的直径（毫米）
 * @note  新题要求小球直径 2cm
 */
#define BALL_DIAMETER_MM                20.0f

/**
 * @brief 球的半径（毫米）
 */
#define BALL_RADIUS_MM                  (BALL_DIAMETER_MM * 0.5f)

/**
 * @brief 进入区域判定半径（毫米）
 * @note  进入判定放宽一些，便于“到达区域”触发
 */
#define REGION_ENTER_RADIUS_MM          (REGION_RADIUS_MM + BALL_RADIUS_MM)

/**
 * @brief 保持区域判定半径（毫米）
 * @note  保持判定更严格，避免停留时边缘抖动
 */
#define REGION_HOLD_RADIUS_MM           (REGION_RADIUS_MM - BALL_RADIUS_MM)

/* ==================== 坐标系统配置 ==================== */

/**
 * @brief 平台中心X坐标（毫米）
 */
#define BOARD_CENTER_X_MM               300.0f

/**
 * @brief 平台中心Y坐标（毫米）
 */
#define BOARD_CENTER_Y_MM               300.0f

/* ==================== 控制周期配置 ==================== */

#define CONTROL_OUTER_DT_S              0.01f
#define CONTROL_INNER_DT_S              0.005f

/* ==================== 通信超时配置 ==================== */

#define PI_UART_TIMEOUT_MS              120U
#define IMU_TIMEOUT_MS                  100U

/* ==================== 任务与路径配置 ==================== */

/**
 * @brief 路径最大点数
 */
#define USER_ROUTE_MAX_LEN              9U

/**
 * @brief 路线步骤最大数
 */
#define TASK_MAX_STEPS                  20U

/**
 * @brief 基础题中心区保持时间（毫秒）
 */
#define CENTER_HOLD_MS                  5000U

/**
 * @brief 普通多点任务每点停留时间（毫秒）
 */
#define ROUTE_HOLD_MS                   2000U

/**
 * @brief 两点往返每次停留时间（毫秒）
 */
#define ROUNDTRIP_HOLD_MS               3000U

/**
 * @brief “经过不停留”任务终点保持时间（毫秒）
 */
#define PASS_END_HOLD_MS                2000U

/**
 * @brief 默认步骤超时时间（毫秒）
 */
#define DEFAULT_STEP_TIMEOUT_MS         12000U

/* ==================== 伺服电机配置 ==================== */

#define SERVO_PWM_MIN_US                500.0f
#define SERVO_PWM_MID_US                1500.0f
#define SERVO_PWM_MAX_US                2500.0f

/**
 * @brief 舵机物理总角度（度）
 * @note  舵机本体是 270°
 */
#define SERVO_PHYS_TOTAL_DEG            270.0f

/**
 * @brief 舵机有效工作区最小角（度）
 * @note  机构单调有效区：0~180°
 */
#define SERVO_WORK_MIN_DEG              0.0f
#define SERVO_WORK_MAX_DEG              180.0f
#define SERVO_WORK_CENTER_DEG           90.0f

#define SERVO_X_CENTER_OFFSET_DEG       0.0f
#define SERVO_Y_CENTER_OFFSET_DEG       0.0f

#define SERVO_X_MAX_CMD_DEG             8.0f
#define SERVO_Y_MAX_CMD_DEG             8.0f

#define SERVO_MAX_CMD_DEG               ((SERVO_X_MAX_CMD_DEG < SERVO_Y_MAX_CMD_DEG) ? SERVO_X_MAX_CMD_DEG : SERVO_Y_MAX_CMD_DEG)

/* ==================== 外环角度限制 ==================== */

#define BOARD_MAX_TILT_REF_DEG          8.0f
#define BOARD_HOLD_TILT_REF_DEG         2.5f
#define BOARD_BRAKE_TILT_REF_DEG        4.0f

/* ==================== 硬件方向配置 ==================== */

#define SERVO_X_REVERSE                 0
#define SERVO_Y_REVERSE                 0

#define IMU_ROLL_REVERSE                0
#define IMU_PITCH_REVERSE               0

/* ==================== 安全边界配置 ==================== */

#define BALL_NEAR_EDGE_MM               40.0f
#define BALL_LOST_SAFE_CENTER           1

/* ==================== 通信协议配置 ==================== */

#define JY61P_FRAME_LEN                 11U
#define PI_FRAME_MAX_LEN                32U

/* ==================== 树莓派命令码定义 ==================== */

#define PI_CMD_BALL_STATE               0x01U
#define PI_CMD_TASK_CTRL                0x02U
#define PI_CMD_SET_ROUTE                0x03U
#define PI_CMD_SET_TARGET               0x04U
#define PI_CMD_LASER_TRACK              0x05U

/* ==================== 任务ID定义 ==================== */

/**
 * @brief 空闲
 */
#define TASK_ID_NONE                    0U

/**
 * @brief 去中心区域（5号区）
 */
#define TASK_ID_GOTO_CENTER             1U

/**
 * @brief 去指定区域
 */
#define TASK_ID_GOTO_REGION             2U

/**
 * @brief 去指定坐标
 */
#define TASK_ID_GOTO_POINT              3U

/**
 * @brief 多区域顺序停留
 */
#define TASK_ID_ROUTE_HOLD              4U

/**
 * @brief 两点往返
 */
#define TASK_ID_ROUND_TRIP              5U

/**
 * @brief 多区域经过不停留，终点停下
 */
#define TASK_ID_ROUTE_PASS              6U

/**
 * @brief 激光追踪
 */
#define TASK_ID_TRACK_LASER             7U

/* ==================== 控制模式定义 ==================== */

#define CTRL_MODE_FAST                  0U
#define CTRL_MODE_BRAKE                 1U
#define CTRL_MODE_HOLD                  2U

/* ==================== 常用宏定义 ==================== */

static inline float Limitf(float val, float minv, float maxv)
{
    if (val < minv) return minv;
    if (val > maxv) return maxv;
    return val;
}

#define ABSF(x)                         ((x) >= 0.0f ? (x) : -(x))
#define SQRF(x)                         ((x) * (x))

#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H__ */
