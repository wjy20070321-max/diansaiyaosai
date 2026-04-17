#ifndef __CONTROLLER_MGR_H__
#define __CONTROLLER_MGR_H__

/* -------------------- 依赖头文件 -------------------- */
#include "app_config.h"        // 系统全局配置参数、通用宏定义
#include "jy61p.h"             // IMU 姿态数据结构定义
#include "protocol_pi.h"       // 树莓派通信数据结构定义
#include "task_mgr.h"          // 任务管理模块接口
#include "ball_outer_loop.h"   // 小球位置外环控制接口
#include "plate_inner_loop.h"  // 平台姿态内环控制接口

/* -------------------- 小球状态结构体 -------------------- */
/**
 * @brief 小球当前状态
 * @note 这个结构体用于保存视觉系统给出的球位置、速度和有效标志
 *
 *       它描述的是“球现在怎么样”，
 *       通常由树莓派视觉系统提供，再由控制器统一写入 g_sys.ball。
 */
typedef struct
{
    float x_mm;         // 小球当前 X 坐标，单位：毫米
    float y_mm;         // 小球当前 Y 坐标，单位：毫米
    float vx_mmps;      // 小球当前 X 方向速度，单位：毫米/秒
    float vy_mmps;      // 小球当前 Y 方向速度，单位：毫米/秒
    uint8_t valid;      // 小球位置是否有效：1=有效，0=无效
    uint32_t tick_ms;   // 该次球数据的时间戳，单位：毫秒
} BallState_t;

/* -------------------- 控制参考量结构体 -------------------- */
/**
 * @brief 控制参考量
 * @note 这个结构体保存“系统当前希望达到的目标”
 *
 *       里面既包含：
 *       - 目标点坐标
 *       - 平台目标倾角
 *       - 舵机最终命令角
 *       - 当前控制模式
 *
 *       也就是说，它描述的是“系统接下来想怎么控制”。
 */
typedef struct
{
    float target_x_mm;       // 当前目标点 X 坐标，单位：毫米
    float target_y_mm;       // 当前目标点 Y 坐标，单位：毫米

    float theta_x_ref_deg;   // 平台 X 轴目标倾角，单位：度
    float theta_y_ref_deg;   // 平台 Y 轴目标倾角，单位：度

    float servo_x_cmd_deg;   // X 轴舵机最终命令角，单位：度
    float servo_y_cmd_deg;   // Y 轴舵机最终命令角，单位：度

    uint8_t mode;            // 当前控制模式，如 FAST / BRAKE / HOLD
} ControlRef_t;

/* -------------------- 系统总上下文结构体 -------------------- */
/**
 * @brief 系统上下文
 * @note 这个结构体相当于整个控制系统的“总状态仓库”
 *
 *       它把控制系统最关键的三类信息统一放在一起：
 *       1. ball：小球当前状态
 *       2. imu ：平台当前姿态状态
 *       3. ref ：当前控制目标和控制输出参考量
 *
 *       这样做的好处是：
 *       整个系统只需要围绕一个 g_sys 结构体，就能统一管理运行状态。
 */
typedef struct
{
    BallState_t ball;       // 小球状态
    JY61P_Data_t imu;       // IMU 当前数据（姿态角、角速度、零偏等）
    ControlRef_t ref;       // 当前控制参考量
    uint8_t emergency_stop; // 紧急停止标志：1=急停，0=正常
} SystemContext_t;

/* -------------------- 全局系统上下文 -------------------- */
/* 在 controller_mgr.c 中定义，在其他文件中通过 extern 引用 */
extern SystemContext_t g_sys;

/* -------------------- 控制器管理模块接口函数 -------------------- */

/**
 * @brief 控制器管理模块初始化
 * @note 用于初始化系统上下文和默认控制目标
 *
 *       一般在系统启动时调用一次。
 */
void ControllerMgr_Init(void);

/**
 * @brief 更新系统输入数据
 * @note 从树莓派、IMU、串口屏等模块读取最新数据，
 *       并同步更新到 g_sys 中
 *
 *       这个函数主要负责把“外部输入”统一收进系统。
 */
void ControllerMgr_UpdateInputs(void);

/**
 * @brief 运行 10ms 控制任务
 * @note 一般用于外环控制：
 *       根据目标位置和当前球位置，计算平台目标倾角
 *
 *       可以理解成：
 *       “球应该往哪滚”这一层控制。
 */
void ControllerMgr_Run10ms(void);

/**
 * @brief 运行 5ms 控制任务
 * @note 一般用于内环控制：
 *       根据平台目标倾角和 IMU 实际姿态，计算舵机输出命令
 *
 *       可以理解成：
 *       “平台要怎么倾，舵机要怎么转”这一层控制。
 */
void ControllerMgr_Run5ms(void);

#endif
