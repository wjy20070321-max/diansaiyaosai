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
#include "stm32f4xx_hal.h"    // STM32 HAL库头文件
#include <stdint.h>           // 标准整数类型定义
#include <string.h>           // 字符串操作函数
#include <math.h>             // 数学函数库

/* C++兼容性声明 */
#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 物理尺寸配置 ==================== */

/**
 * @brief 平台尺寸（毫米）
 * @note  平台为正方形，边长650毫米
 */
#define BOARD_SIZE_MM                   650.0f

/**
 * @brief 目标区域直径（毫米）
 * @note  球需要到达的目标区域大小
 */
#define REGION_DIAMETER_MM              30.0f

/**
 * @brief 目标区域半径（毫米）
 * @note  目标区域半径 = 直径 / 2
 */
#define REGION_RADIUS_MM                15.0f

/**
 * @brief 球的直径（毫米）
 * @note  使用的标准球直径为25毫米
 */
#define BALL_DIAMETER_MM                25.0f

/**
 * @brief 球的半径（毫米）
 * @note  球半径 = 直径 / 2
 */
#define BALL_RADIUS_MM                  (BALL_DIAMETER_MM * 0.5f)

/**
 * @brief 进入区域判定半径（毫米）
 * @note  当球心距离目标区域中心小于该值时，判定球已进入区域
 *       = 区域半径 + 球半径（考虑球的大小）
 */
#define REGION_ENTER_RADIUS_MM          (REGION_RADIUS_MM + BALL_RADIUS_MM)

/**
 * @brief 保持区域判定半径（毫米）
 * @note  当球心距离目标区域中心小于该值时，判定球稳定保持在区域内
 *       = 区域半径 - 球半径（确保球完全在区域内）
 */
#define REGION_HOLD_RADIUS_MM           (REGION_RADIUS_MM - BALL_RADIUS_MM)

/* ==================== 坐标系统配置 ==================== */

/**
 * @brief 平台中心X坐标（毫米）
 * @note  平台坐标原点在左上角，中心点为(325, 325)
 */
#define BOARD_CENTER_X_MM               325.0f

/**
 * @brief 平台中心Y坐标（毫米）
 * @note  平台坐标原点在左上角，中心点为(325, 325)
 */
#define BOARD_CENTER_Y_MM               325.0f

/* ==================== 控制周期配置 ==================== */

/**
 * @brief 外环控制周期（秒）
 * @note  球位置控制环的采样周期为10ms
 */
#define CONTROL_OUTER_DT_S              0.01f      /* 10ms */

/**
 * @brief 内环控制周期（秒）
 * @note  平台姿态控制环的采样周期为5ms
 */
#define CONTROL_INNER_DT_S              0.005f     /* 5ms  */

/* ==================== 通信超时配置 ==================== */

/**
 * @brief 树莓派通信超时时间（毫秒）
 * @note  超过该时间未收到树莓派数据则判定通信异常
 */
#define PI_UART_TIMEOUT_MS              120U

/**
 * @brief IMU传感器超时时间（毫秒）
 * @note  超过该时间未收到IMU数据则判定传感器异常
 */
#define IMU_TIMEOUT_MS                  100U

/* ==================== 伺服电机配置 ==================== */

/**
 * @brief 伺服电机最小PWM脉宽（微秒）
 * @note  参考你旧项目，同款舵机按 500~2500us 使用
 */
#define SERVO_PWM_MIN_US                500.0f

/**
 * @brief 伺服电机中位PWM脉宽（微秒）
 * @note  舵机电气中位，不一定等于机构有效中位
 */
#define SERVO_PWM_MID_US                1500.0f

/**
 * @brief 伺服电机最大PWM脉宽（微秒）
 */
#define SERVO_PWM_MAX_US                2500.0f

/**
 * @brief 舵机物理总角度（度）
 * @note  舵机本体仍然是 270°
 */
#define SERVO_PHYS_TOTAL_DEG            270.0f

/**
 * @brief 舵机有效工作区最小角（度）
 * @note  机构真正有效的单调区起点
 */
#define SERVO_WORK_MIN_DEG              0.0f

/**
 * @brief 舵机有效工作区最大角（度）
 * @note  超过这个角度机构会反向，因此不能再用
 */
#define SERVO_WORK_MAX_DEG              180.0f

/**
 * @brief 舵机有效工作区中心角（度）
 * @note  当前机构按你的描述，90° 为有效中心
 */
#define SERVO_WORK_CENTER_DEG           90.0f

/**
 * @brief X轴舵机安装补偿角（度）
 * @note  平台真正水平时，X轴舵机相对有效中心角的偏置
 */
#define SERVO_X_CENTER_OFFSET_DEG       0.0f

/**
 * @brief Y轴舵机安装补偿角（度）
 * @note  平台真正水平时，Y轴舵机相对有效中心角的偏置
 */
#define SERVO_Y_CENTER_OFFSET_DEG       0.0f

/**
 * @brief X轴舵机最大相对控制角（度）
 * @note  建议先保守一点，后面再按机构实测放宽
 */
#define SERVO_X_MAX_CMD_DEG             8.0f

/**
 * @brief Y轴舵机最大相对控制角（度）
 */
#define SERVO_Y_MAX_CMD_DEG             8.0f

/**
 * @brief 兼容旧内环代码的统一舵机最大控制角（度）
 * @note  plate_inner_loop.c 仍在使用 SERVO_MAX_CMD_DEG 做限幅
 *       这里取 X/Y 两轴较小值，保证安全
 */
#define SERVO_MAX_CMD_DEG               ((SERVO_X_MAX_CMD_DEG < SERVO_Y_MAX_CMD_DEG) ? SERVO_X_MAX_CMD_DEG : SERVO_Y_MAX_CMD_DEG)

/**
 * @brief 平台最大倾斜参考角度（度）
 * @note  快速运动时平台允许的最大倾斜角度
 *       ball_outer_loop.c 仍在使用这个宏，必须保留
 */
#define BOARD_MAX_TILT_REF_DEG          8.0f

/**
 * @brief 平台保持倾斜参考角度（度）
 * @note  保持球稳定时平台的微小倾斜角度
 *       ball_outer_loop.c 仍在使用这个宏，必须保留
 */
#define BOARD_HOLD_TILT_REF_DEG         2.5f

/**
 * @brief 平台制动倾斜参考角度（度）
 * @note  制动时平台的倾斜角度，用于快速减速
 *       ball_outer_loop.c 仍在使用这个宏，必须保留
 */
#define BOARD_BRAKE_TILT_REF_DEG        4.0f

/* ==================== 硬件方向配置 ==================== */

/**
 * @brief X轴伺服电机方向反转标志
 * @note  0=不反转，1=反转方向
 */
#define SERVO_X_REVERSE                 0

/**
 * @brief Y轴伺服电机方向反转标志
 * @note  0=不反转，1=反转方向
 */
#define SERVO_Y_REVERSE                 0

/**
 * @brief IMU滚转轴方向反转标志
 * @note  0=不反转，1=反转方向
 */
#define IMU_ROLL_REVERSE                0

/**
 * @brief IMU俯仰轴方向反转标志
 * @note  0=不反转，1=反转方向
 */
#define IMU_PITCH_REVERSE               0

/* ==================== 安全边界配置 ==================== */

/**
 * @brief 球接近边缘的安全距离（毫米）
 * @note  当球距离平台边缘小于此距离时，触发安全保护
 */
#define BALL_NEAR_EDGE_MM               40.0f

/**
 * @brief 球丢失时的安全中心模式
 * @note  1=启用，球丢失时将平台归中；0=禁用
 */
#define BALL_LOST_SAFE_CENTER           1

/* ==================== 通信协议配置 ==================== */

/**
 * @brief JY61P传感器数据帧长度（字节）
 * @note  JY61P姿态传感器每帧数据包含11个字节
 */
#define JY61P_FRAME_LEN                 11U

/**
 * @brief 树莓派通信最大帧长度（字节）
 * @note  与树莓派通信的数据帧最大长度
 */
#define PI_FRAME_MAX_LEN                32U

/* ==================== 树莓派命令码定义 ==================== */

/**
 * @brief 球状态上报命令
 * @note  树莓派向STM32发送球的位置和状态信息
 */
#define PI_CMD_BALL_STATE               0x01U

/**
 * @brief 任务控制命令
 * @note  树莓派控制任务的启动、停止等操作
 */
#define PI_CMD_TASK_CTRL                0x02U

/**
 * @brief 设置路径命令
 * @note  树莓派设置球的运动路径
 */
#define PI_CMD_SET_ROUTE                0x03U

/**
 * @brief 设置目标位置命令
 * @note  树莓派设置球的目标位置
 */
#define PI_CMD_SET_TARGET               0x04U

/**
 * @brief 激光坐标下发命令
 * @note  树莓派向STM32发送激光点位置和有效位
 */
#define PI_CMD_LASER_TRACK              0x05U

/* ==================== 任务ID定义 ==================== */

/**
 * @brief 无任务模式
 * @note  系统空闲状态
 */
#define TASK_ID_NONE                    0U

/**
 * @brief 保持2号区域任务
 * @note  控制球保持在2号区域
 */
#define TASK_ID_HOLD_2                  1U

/**
 * @brief 1号到5号区域任务
 * @note  控制球从1号区域移动到5号区域
 */
#define TASK_ID_1_TO_5                  2U

/**
 * @brief 1号到4号再到5号区域任务
 * @note  控制球沿1→4→5路径运动
 */
#define TASK_ID_1_TO_4_TO_5             3U

/**
 * @brief 1号到9号区域任务
 * @note  控制球从1号区域移动到9号区域
 */
#define TASK_ID_1_TO_9                  4U

/**
 * @brief 1-2-6-9区域巡回任务
 * @note  控制球依次经过1、2、6、9号区域
 */
#define TASK_ID_1_2_6_9                 5U

/**
 * @brief 用户自定义ABCD任务
 * @note  用户可自定义的特殊任务模式
 */
#define TASK_ID_USER_ABCD               6U

/**
 * @brief 激光追踪动态任务
 */
#define TASK_ID_TRACK_LASER             7U

/* ==================== 控制模式定义 ==================== */

/**
 * @brief 快速模式
 * @note  球快速运动到目标位置，使用最大倾斜角度
 */
#define CTRL_MODE_FAST                  0U

/**
 * @brief 制动模式
 * @note  球接近目标时减速制动，使用中等倾斜角度
 */
#define CTRL_MODE_BRAKE                 1U

/**
 * @brief 保持模式
 * @note  球已到达目标，保持稳定在目标区域内
 */
#define CTRL_MODE_HOLD                  2U

/* ==================== 常用宏定义 ==================== */

/**
 * @brief 浮点数限幅安全内联函数 (替代原有的 LIMIT 宏)
 * @param val: 待限幅的值
 * @param minv: 最小值
 * @param maxv: 最大值
 * @retval 限幅后的值
 */
static inline float Limitf(float val, float minv, float maxv)
{
    if (val < minv) return minv;
    if (val > maxv) return maxv;
    return val;
}

/**
 * @brief 浮点数绝对值宏
 * @param x: 待求绝对值的浮点数
 * @retval x的绝对值
 */
#define ABSF(x)                         ((x) >= 0.0f ? (x) : -(x))

/**
 * @brief 浮点数平方宏
 * @param x: 待求平方的浮点数
 * @retval x的平方
 */
#define SQRF(x)                         ((x) * (x))

/* C++兼容性结束 */
#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H__ */
