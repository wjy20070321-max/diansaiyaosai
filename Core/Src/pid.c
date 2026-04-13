#include "pid.h"  // PID控制器模块头文件

/**
 * @brief PID控制器初始化函数
 * @details 初始化PID控制器的所有参数，包括比例、积分、微分系数以及限制范围
 * 
 * @param pid 指向PID结构体的指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param out_min 输出最小限制值
 * @param out_max 输出最大限制值
 * @param integ_min 积分最小限制值
 * @param integ_max 积分最大限制值
 * 
 * @note 初始化时，积分值和前一次误差值均设置为0
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max,
              float integ_min, float integ_max)
{
    pid->kp = kp;               // 设置比例系数
    pid->ki = ki;               // 设置积分系数
    pid->kd = kd;               // 设置微分系数
    pid->integ = 0.0f;         // 初始化积分值为0
    pid->prev_err = 0.0f;      // 初始化前一次误差为0
    pid->out_min = out_min;    // 设置输出最小限制
    pid->out_max = out_max;    // 设置输出最大限制
    pid->integ_min = integ_min;  // 设置积分最小限制
    pid->integ_max = integ_max;  // 设置积分最大限制
}

/**
 * @brief PID控制器运行函数（基于误差值）
 * @details 根据当前误差值计算PID输出，包含积分和微分计算
 * 
 * @param pid 指向PID结构体的指针
 * @param err 当前误差值
 * @param dt 时间间隔（秒）
 * @return 计算出的PID输出值
 * 
 * @note 计算过程：
 *       1. 更新积分值：integ += err * dt
 *       2. 计算微分值：(err - prev_err) / dt
 *       3. 计算输出值：kp * err + ki * integ + kd * deriv
 *       4. 限制输出值在[out_min, out_max]范围内
 */
float PID_RunErr(PID_t *pid, float err, float dt)
{
    float deriv;
    float out;

    // 更新积分值，并限制在[integ_min, integ_max]范围内
    pid->integ += err * dt;
    pid->integ = Limitf(pid->integ, pid->integ_min, pid->integ_max);

    // 计算微分值
    deriv = (err - pid->prev_err) / dt;
    
    // 计算PID输出值
    out = pid->kp * err + pid->ki * pid->integ + pid->kd * deriv;
    
    // 限制输出值在[out_min, out_max]范围内
    out = Limitf(out, pid->out_min, pid->out_max);
    
    // 保存当前误差值，供下次计算微分使用
    pid->prev_err = err;
    
    return out;
}

/**
 * @brief PID控制器简化运行函数
 * @details 直接使用参考值和反馈值计算误差，然后调用PID_RunErr
 * 
 * @param pid 指向PID结构体的指针
 * @param ref 参考值（期望值）
 * @param fdb 反馈值（实际值）
 * @param dt 时间间隔（秒）
 * @return 计算出的PID输出值
 * 
 * @note 该函数简化了调用过程，自动计算误差值：err = ref - fdb
 */
float PID_Run(PID_t *pid, float ref, float fdb, float dt)
{
    return PID_RunErr(pid, ref - fdb, dt);
}

/**
 * @brief PID控制器重置函数
 * @details 重置PID控制器状态，将积分值和前一次误差值清零
 * 
 * @param pid 指向PID结构体的指针
 * 
 * @note 重置后，PID控制器将从初始状态重新开始计算
 */
void PID_Reset(PID_t *pid)
{
    pid->integ = 0.0f;        // 重置积分值为0
    pid->prev_err = 0.0f;     // 重置前一次误差为0
}
