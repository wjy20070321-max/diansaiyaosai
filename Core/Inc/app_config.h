/**
 ******************************************************************************
 * @file    app_config.h
 * @brief   球盘控制系统应用配置头文件
 * @details 包含系统所有配置参数、常量定义和宏定义
 ******************************************************************************
 */

#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

/* -------------------- 必要头文件 -------------------- */
#include "stm32f4xx_hal.h"  // STM32 HAL 库
#include <stdint.h>         // 标准整数类型
#include <string.h>         // 字符串处理函数
#include <math.h>           // 数学函数库

/* -------------------- C++ 兼容声明 -------------------- */
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
 * @brief 小球直径（毫米）
 * @note  新题要求小球直径 2cm
 */
#define BALL_DIAMETER_MM                20.0f

/**
 * @brief 小球半径（毫米）
 */
#define BALL_RADIUS_MM                  (BALL_DIAMETER_MM * 0.5f)

/**
 * @brief 进入区域判定半径（毫米）
 * @note  进入判定适当放宽，便于判断“小球到达目标区域”
 */
#define REGION_ENTER_RADIUS_MM          (REGION_RADIUS_MM + BALL_RADIUS_MM)

/**
 * @brief 保持区域判定半径（毫米）
 * @note  保持判定更严格，避免小球在边缘抖动时误判为稳定停留
 */
#define REGION_HOLD_RADIUS_MM           (REGION_RADIUS_MM - BALL_RADIUS_MM)

/**
 * @brief 任务到达判定半径（毫米）
 * @note  用于 ROUTE / ROUND / 直接去区域 等任务的“算到达/算稳定”判定。
 *        这里按你现在要求，半径 2cm 黄圈内就算到了。
 */
#define TASK_REACHED_RADIUS_MM          20.0f

/* ==================== 坐标系统配置 ==================== */

/**
 * @brief 平台中心 X 坐标（毫米）
 */
#define BOARD_CENTER_X_MM               305.0f

/**
 * @brief 平台中心 Y 坐标（毫米）
 */
#define BOARD_CENTER_Y_MM               305.0f

/* ==================== 长距离任务拆段配置 ==================== */

/**
 * @brief 虚拟过渡点保持时间（毫秒）
 * @note  长距离拆段后，中间虚拟点使用 BRAKE 模式并保持 1 秒。
 */
#define VIRTUAL_POINT_HOLD_MS           100U

/* ==================== 控制周期配置 ==================== */

/**
 * @brief 外环控制周期（秒）
 * @note  外环通常负责小球位置控制
 */
#define CONTROL_OUTER_DT_S              0.01f

/**
 * @brief 内环控制周期（秒）
 * @note  内环通常负责平台姿态控制
 */
#define CONTROL_INNER_DT_S              0.005f

/* ==================== 通信超时配置 ==================== */

/**
 * @brief 树莓派通信超时时间（毫秒）
 * @note  超过该时间没收到视觉数据，则认为视觉数据失效
 */
#define PI_UART_TIMEOUT_MS              120U

/**
 * @brief IMU 通信超时时间（毫秒）
 * @note  超过该时间没收到姿态数据，则认为 IMU 数据失效
 */
#define IMU_TIMEOUT_MS                  100U

/* ==================== 任务与路径配置 ==================== */

/**
 * @brief 用户路径最大点数
 * @note  最多支持 9 个区域点
 */
#define USER_ROUTE_MAX_LEN              9U

/**
 * @brief 任务步骤最大数
 * @note  一个任务最多被拆成 20 个执行步骤
 */
#define TASK_MAX_STEPS                  20U

/**
 * @brief 基础题中心区域保持时间（毫秒）
 */
#define CENTER_HOLD_MS                  6000U

/**
 * @brief 普通多点任务每点停留时间（毫秒）
 */
#define ROUTE_HOLD_MS                   3000U

/**
 * @brief 两点往返任务每次停留时间（毫秒）
 */
#define ROUNDTRIP_HOLD_MS               4000U

/**
 * @brief “经过不停留”任务终点保持时间（毫秒）
 */
#define PASS_END_HOLD_MS                3000U

/**
 * @brief 默认步骤超时时间（毫秒）
 * @note  某一步超过这个时间还没完成，就判定失败
 */
#define DEFAULT_STEP_TIMEOUT_MS         22000U

/* ==================== 伺服电机配置 ==================== */

/**
 * @brief 舵机 PWM 最小脉宽（微秒）
 */
#define SERVO_PWM_MIN_US                500.0f

/**
 * @brief 舵机 PWM 中位脉宽（微秒）
 */
#define SERVO_PWM_MID_US                1500.0f

/**
 * @brief 舵机 PWM 最大脉宽（微秒）
 */
#define SERVO_PWM_MAX_US                2500.0f

/**
 * @brief 舵机物理总角度（度）
 * @note  舵机本体标称总行程为 270°
 */
#define SERVO_PHYS_TOTAL_DEG            180.0f

/* -------------------- X 轴独立工作区配置 -------------------- */

/**
 * @brief X 轴舵机有效工作区最小角（度）
 */
#define SERVO_X_WORK_MIN_DEG            10.0f

/**
 * @brief X 轴舵机有效工作区最大角（度）
 * @note  当前机械实测安全单调区：0~120°
 */
#define SERVO_X_WORK_MAX_DEG            170.0f

/**
 * @brief X 轴舵机工作中心角（度）
 */
#define SERVO_X_WORK_CENTER_DEG         90.0f

/* -------------------- Y 轴独立工作区配置 -------------------- */

/**
 * @brief Y 轴舵机有效工作区最小角（度）
 */
#define SERVO_Y_WORK_MIN_DEG            10.0f

/**
 * @brief Y 轴舵机有效工作区最大角（度）
 * @note  当前先与 X 轴一致，后续若机械实测不同可单独再改
 */
#define SERVO_Y_WORK_MAX_DEG            170.0f

/**
 * @brief Y 轴舵机工作中心角（度）
 * @note  这是这次最关键的修正点。
 *       现在先单独给 Y 轴一套中心位，后续你只调它即可。
 *       如果“Y 轴能上抬下不来”，优先改这个值，不要动 X 轴。
 */
#define SERVO_Y_WORK_CENTER_DEG         90.0f

/**
 * @brief X 轴舵机中心偏移（度）
 * @note  如果安装机械中位不正，可通过这里补偿
 */
#define SERVO_X_CENTER_OFFSET_DEG       -2.5f

/**
 * @brief Y 轴舵机中心偏移（度）
 * @note  一般先保持 0，优先调 SERVO_Y_WORK_CENTER_DEG
 */
#define SERVO_Y_CENTER_OFFSET_DEG       2.3f

/**
 * @brief X 轴舵机负向最大命令角（度）
 */
#define SERVO_X_NEG_MAX_CMD_DEG         15.0f

/**
 * @brief X 轴舵机正向最大命令角（度）
 */
#define SERVO_X_POS_MAX_CMD_DEG         15.0f

/**
 * @brief Y 轴舵机负向最大命令角（度）
 */
#define SERVO_Y_NEG_MAX_CMD_DEG         15.0f

/**
 * @brief Y 轴舵机正向最大命令角（度）
 */
#define SERVO_Y_POS_MAX_CMD_DEG         15.0f

/**
 * @brief X 轴舵机最大命令角（度）
 * @note  【兼容保留】
 */
#define SERVO_X_MAX_CMD_DEG             (SERVO_X_POS_MAX_CMD_DEG)

/**
 * @brief Y 轴舵机最大命令角（度）
 * @note  【兼容保留】
 */
#define SERVO_Y_MAX_CMD_DEG             (SERVO_Y_POS_MAX_CMD_DEG)

/**
 * @brief 舵机最大命令角统一值
 * @note  【兼容保留】
 */
#define SERVO_MAX_CMD_DEG               ((SERVO_X_MAX_CMD_DEG < SERVO_Y_MAX_CMD_DEG) ? SERVO_X_MAX_CMD_DEG : SERVO_Y_MAX_CMD_DEG)

/* -------------------- 旧宏兼容保留 -------------------- */
/* 如果工程里还有其他地方引用旧的统一工作区宏，这里先兼容到 X 轴参数 */
#define SERVO_WORK_MIN_DEG              SERVO_X_WORK_MIN_DEG
#define SERVO_WORK_MAX_DEG              SERVO_X_WORK_MAX_DEG
#define SERVO_WORK_CENTER_DEG           SERVO_X_WORK_CENTER_DEG

/* ==================== 外环角度限制 ==================== */

/**
 * @brief 平台最大倾斜参考角（度）
 * @note  外环输出给内环的最大目标倾角
 */
#define BOARD_MAX_TILT_REF_DEG          8.0f

/**
 * @brief 保持状态下的平台参考倾角（度）
 * @note  用于目标点附近的小范围稳定控制
 */
#define BOARD_HOLD_TILT_REF_DEG         2.5f

/**
 * @brief 刹车状态下的平台参考倾角（度）
 * @note  用于接近目标时减速
 */
#define BOARD_BRAKE_TILT_REF_DEG        8.0f

/* ==================== 硬件方向配置 ==================== */

/**
 * @brief X 轴舵机方向是否反向
 * @note  0=正常，1=反向
 */
#define SERVO_X_REVERSE                 1

/**
 * @brief Y 轴舵机方向是否反向
 */
#define SERVO_Y_REVERSE                 1

/**
 * @brief IMU 横滚角方向是否反向
 */
#define IMU_ROLL_REVERSE                1

/**
 * @brief IMU 俯仰角方向是否反向
 */
#define IMU_PITCH_REVERSE               0

/* ==================== 安全边界配置 ==================== */

/**
 * @brief 小球靠近边缘的判定距离（毫米）
 * @note  小于该值时可触发保护或减速策略
 */
#define BALL_NEAR_EDGE_MM               10.0f

/**
 * @brief 小球丢失时是否强制平台回中
 * @note  1=启用安全回中，0=禁用
 */
#define BALL_LOST_SAFE_CENTER           1

/* ==================== 通信协议配置 ==================== */

/**
 * @brief JY61P 单帧长度（字节）
 */
#define JY61P_FRAME_LEN                 11U

/**
 * @brief 树莓派协议最大帧长度（字节）
 */
#define PI_FRAME_MAX_LEN                32U

/* ==================== 树莓派命令码定义 ==================== */

/**
 * @brief 树莓派发送小球状态
 */
#define PI_CMD_BALL_STATE               0x01U

/**
 * @brief 树莓派发送任务控制命令
 */
#define PI_CMD_TASK_CTRL                0x02U

/**
 * @brief 树莓派发送路径设置命令
 */
#define PI_CMD_SET_ROUTE                0x03U

/**
 * @brief 树莓派发送目标点命令
 */
#define PI_CMD_SET_TARGET               0x04U

/**
 * @brief 树莓派发送激光追踪目标
 */
#define PI_CMD_LASER_TRACK              0x05U

/* ==================== 任务 ID 定义 ==================== */

/**
 * @brief 空闲任务
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
 * @brief 去指定坐标点
 */
#define TASK_ID_GOTO_POINT              3U

/**
 * @brief 多区域顺序停留任务
 */
#define TASK_ID_ROUTE_HOLD              4U

/**
 * @brief 两点往返任务
 */
#define TASK_ID_ROUND_TRIP              5U

/**
 * @brief 多区域经过但不停留，最后在终点停下
 */
#define TASK_ID_ROUTE_PASS              6U

/**
 * @brief 激光追踪任务
 */
#define TASK_ID_TRACK_LASER             7U

/* ==================== 控制模式定义 ==================== */

/**
 * @brief 快速移动模式
 */
#define CTRL_MODE_FAST                  0U

/**
 * @brief 刹车减速模式
 */
#define CTRL_MODE_BRAKE                 1U

/**
 * @brief 保持稳定模式
 */
#define CTRL_MODE_HOLD                  2U

/* ==================== 常用宏定义 ==================== */

/**
 * @brief 浮点限幅函数
 * @param val  输入值
 * @param minv 最小值
 * @param maxv 最大值
 * @retval 限幅后的值
 */
static inline float Limitf(float val, float minv, float maxv)
{
    if (val < minv) return minv;
    if (val > maxv) return maxv;
    return val;
}

/**
 * @brief 浮点绝对值宏
 */
#define ABSF(x)                         ((x) >= 0.0f ? (x) : -(x))

/**
 * @brief 浮点平方宏
 */
#define SQRF(x)                         ((x) * (x))

/* -------------------- C++ 兼容结束 -------------------- */
#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H__ */
