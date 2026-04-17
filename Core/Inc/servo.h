#ifndef __SERVO_H__
#define __SERVO_H__

/* -------------------- 依赖头文件 -------------------- */
#include "app_config.h"   // 系统全局配置头文件，提供舵机参数、角度限制和基础类型

/* -------------------- 舵机控制接口 -------------------- */

/**
 * @brief 舵机模块初始化
 * @note 一般在系统启动时调用。
 *       该函数通常负责：
 *       - 启动舵机对应的 PWM 输出
 *       - 初始化舵机相关状态
 *       - 让舵机先进入安全初始位置
 */
void Servo_Init(void);

/**
 * @brief 设置 X 轴舵机目标角度
 * @param deg 舵机目标角度，单位：度
 *
 * @note 这个函数用于单独控制 X 轴舵机。
 *       一般会在函数内部完成：
 *       - 角度限幅
 *       - 方向反转处理（如果配置了反向）
 *       - 角度到 PWM 脉宽的换算
 *       - 最终写入定时器比较寄存器
 */
void Servo_SetXDeg(float deg);

/**
 * @brief 设置 Y 轴舵机目标角度
 * @param deg 舵机目标角度，单位：度
 *
 * @note 这个函数用于单独控制 Y 轴舵机。
 *       和 Servo_SetXDeg() 类似，只不过作用对象是 Y 轴。
 */
void Servo_SetYDeg(float deg);

/**
 * @brief 同时设置 X / Y 两个舵机目标角度
 * @param x_deg X 轴舵机目标角度，单位：度
 * @param y_deg Y 轴舵机目标角度，单位：度
 *
 * @note 这个接口通常用于双轴平台控制，
 *       当控制器算出两个轴的命令后，
 *       一次性调用这个函数最方便。
 */
void Servo_SetXYDeg(float x_deg, float y_deg);

/**
 * @brief 舵机回中
 * @note 让 X / Y 两个舵机都回到中心位置。
 *
 *       常见用途：
 *       - 上电初始化时先归中
 *       - 急停保护时归中
 *       - 球丢失时平台回到安全姿态
 *       - 调试时恢复默认位置
 */
void Servo_Center(void);

#endif
