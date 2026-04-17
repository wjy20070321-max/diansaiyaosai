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

/* ==================== 坐标系统配置 ==================== */

/**
 * @brief 平台中心 X 坐标（毫米）
 */
#define BOARD_CENTER_X_MM               300.0f

/**
 * @brief 平台中心 Y 坐标（毫米）
 */
#define BOARD_CENTER_Y_MM               300.0f

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
#define CENTER_HOLD_MS                  5000U

/**
 * @brief 普通多点任务每点停留时间（毫秒）
 */
#define ROUTE_HOLD_MS                   2000U

/**
 * @brief 两点往返任务每次停留时间（毫秒）
 */
#define ROUNDTRIP_HOLD_MS               3000U

/**
 * @brief “经过不停留”任务终点保持时间（毫秒）
 */
#define PASS_END_HOLD_MS                2000U

/**
 * @brief 默认步骤超时时间（毫秒）
 * @note  某一步超过这个时间还没完成，就判定失败
 */
#define DEFAULT_STEP_TIMEOUT_MS         12000U

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
#define SERVO_PHYS_TOTAL_DEG            270.0f

/**
 * @brief 舵机有效工作区最小角（度）
 * @note  当前机械实测安全单调区：0~120°
 */
#define SERVO_WORK_MIN_DEG              0.0f

/**
 * @brief 舵机有效工作区最大角（度）
 * @note  【修改】原来按 180°，现在按你们实物改成 120°
 *       因为超过 120° 会机械卡死
 */
#define SERVO_WORK_MAX_DEG              120.0f

/**
 * @brief 舵机工作中心角（度）
 * @note  以 43° 为中位
 */
#define SERVO_WORK_CENTER_DEG           43.0f

/**
 * @brief X 轴舵机中心偏移（度）
 * @note  如果安装机械中位不正，可通过这里补偿
 */
#define SERVO_X_CENTER_OFFSET_DEG       0.0f

/**
 * @brief Y 轴舵机中心偏移（度）
 */
#define SERVO_Y_CENTER_OFFSET_DEG       0.0f

/**
 * @brief X 轴舵机负向最大命令角（度）
 * @note  【新增】以 90° 为中位时：
 *       - 90 -> 0   可走 90°
 *       因此负向安全范围保留到 -90°
 */
#define SERVO_X_NEG_MAX_CMD_DEG         10.0f

/**
 * @brief X 轴舵机正向最大命令角（度）
 * @note  【新增】以 90° 为中位时：
 *       - 90 -> 120 只可走 30°
 *       因此正向安全范围限制为 +30°
 */
#define SERVO_X_POS_MAX_CMD_DEG         10.0f

/**
 * @brief Y 轴舵机负向最大命令角（度）
 * @note  【新增】当前先按与你描述相同的机械边界处理
 */
#define SERVO_Y_NEG_MAX_CMD_DEG         10.0f

/**
 * @brief Y 轴舵机正向最大命令角（度）
 * @note  【新增】当前先按与你描述相同的机械边界处理
 */
#define SERVO_Y_POS_MAX_CMD_DEG         10.0f

/**
 * @brief X 轴舵机最大命令角（度）
 * @note  【兼容保留】为避免旧代码里仍引用该宏，
 *       这里保守地取“更小的正向极限值”
 */
#define SERVO_X_MAX_CMD_DEG             (SERVO_X_POS_MAX_CMD_DEG)

/**
 * @brief Y 轴舵机最大命令角（度）
 * @note  【兼容保留】为避免旧代码里仍引用该宏，
 *       这里保守地取“更小的正向极限值”
 */
#define SERVO_Y_MAX_CMD_DEG             (SERVO_Y_POS_MAX_CMD_DEG)

/**
 * @brief 舵机最大命令角统一值
 * @note  【兼容保留】旧代码如果仍用统一对称限幅，
 *       则按最保守的正向上限处理，避免机械撞限位
 */
#define SERVO_MAX_CMD_DEG               ((SERVO_X_MAX_CMD_DEG < SERVO_Y_MAX_CMD_DEG) ? SERVO_X_MAX_CMD_DEG : SERVO_Y_MAX_CMD_DEG)

/* ==================== 外环角度限制 ==================== */

/**
 * @brief 平台最大倾斜参考角（度）
 * @note  外环输出给内环的最大目标倾角
 */
#define BOARD_MAX_TILT_REF_DEG          30.0f

/**
 * @brief 保持状态下的平台参考倾角（度）
 * @note  用于目标点附近的小范围稳定控制
 */
#define BOARD_HOLD_TILT_REF_DEG         15.0f

/**
 * @brief 刹车状态下的平台参考倾角（度）
 * @note  用于接近目标时减速
 */
#define BOARD_BRAKE_TILT_REF_DEG        5.0f

/* ==================== 硬件方向配置 ==================== */

/**
 * @brief X 轴舵机方向是否反向
 * @note  0=正常，1=反向
 */
#define SERVO_X_REVERSE                 0

/**
 * @brief Y 轴舵机方向是否反向
 */
#define SERVO_Y_REVERSE                 0

/**
 * @brief IMU 横滚角方向是否反向
 */
#define IMU_ROLL_REVERSE                1

/**
 * @brief IMU 俯仰角方向是否反向
 */
#define IMU_PITCH_REVERSE               1

/* ==================== 安全边界配置 ==================== */

/**
 * @brief 小球靠近边缘的判定距离（毫米）
 * @note  小于该值时可触发保护或减速策略
 */
#define BALL_NEAR_EDGE_MM               40.0f

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
