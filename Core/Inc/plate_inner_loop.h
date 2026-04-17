#ifndef __PLATE_INNER_LOOP_H__
#define __PLATE_INNER_LOOP_H__

/* -------------------- 依赖头文件 -------------------- */
#include "app_config.h"   // 系统全局配置头文件，提供类型定义、宏和控制参数

/* -------------------- 平台内环输出结构体 -------------------- */
/**
 * @brief 平台姿态内环控制输出
 * @note 该结构体保存内环控制器最终算出来的两个舵机命令角
 *
 *       这里的含义是：
 *       - 外环先算出“平台应该倾斜多少度”
 *       - 内环再根据 IMU 实际测得的姿态，计算出舵机该转多少度
 *
 *       所以这个结构体里的值，是“送给舵机模块”的最终命令量
 */
typedef struct
{
    float servo_x_cmd_deg;   // X 轴舵机命令角，单位：度
    float servo_y_cmd_deg;   // Y 轴舵机命令角，单位：度
} PlateInnerOutput_t;

/* -------------------- 平台内环模块接口 -------------------- */

/**
 * @brief 平台姿态内环初始化函数
 * @note 用于初始化内环控制器内部状态
 *
 *       如果内环以后加入：
 *       - PID 状态变量
 *       - 滤波器
 *       - 观测器
 *       等内容，通常都在这里初始化。
 */
void PlateInnerLoop_Init(void);

/**
 * @brief 平台姿态内环复位函数
 * @note 用于清空内环控制器历史状态
 *
 *       常见用途：
 *       - 切换任务时复位
 *       - 传感器异常恢复后复位
 *       - 避免旧的历史状态影响新的控制过程
 */
void PlateInnerLoop_Reset(void);

/**
 * @brief 平台姿态内环主运行函数
 * @param theta_x_ref   平台 X 轴目标倾角，单位：度
 * @param theta_y_ref   平台 Y 轴目标倾角，单位：度
 * @param theta_x_meas  平台 X 轴实际测量倾角，单位：度
 * @param theta_y_meas  平台 Y 轴实际测量倾角，单位：度
 * @param gyro_x_meas   X 轴角速度测量值，单位：度/秒
 * @param gyro_y_meas   Y 轴角速度测量值，单位：度/秒
 * @param out           输出结构体指针，用于返回两个舵机命令角
 *
 * @note 这个函数是平台姿态内环控制的核心接口。
 *       它的任务是：
 *       1. 比较目标平台倾角和实际平台倾角
 *       2. 结合角速度信息进行动态修正
 *       3. 算出两个舵机最终应该转到多少度
 *
 *       一般来说：
 *       - 输入来自外环目标 + IMU 实测
 *       - 输出给 Servo_SetXYDeg() 之类的舵机驱动函数
 */
void PlateInnerLoop_Run(float theta_x_ref, float theta_y_ref,
                        float theta_x_meas, float theta_y_meas,
                        float gyro_x_meas, float gyro_y_meas,
                        PlateInnerOutput_t *out);

#endif
