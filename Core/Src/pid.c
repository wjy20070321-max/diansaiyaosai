#include "pid.h"   // PID 控制器模块头文件

/**
 * @brief PID 控制器初始化函数
 * @param pid       指向 PID 控制器对象的指针
 * @param kp        比例系数
 * @param ki        积分系数
 * @param kd        微分系数
 * @param out_min   控制器输出最小限制值
 * @param out_max   控制器输出最大限制值
 * @param integ_min 积分项最小限制值
 * @param integ_max 积分项最大限制值
 *
 * @note 该函数用于初始化一个 PID 控制器对象。
 *       初始化内容包括：
 *       1. 设置比例 / 积分 / 微分参数
 *       2. 设置输出限幅范围
 *       3. 设置积分限幅范围
 *       4. 清空积分累计值和上一拍误差
 *
 *       一般在系统启动、切换控制器、或者重新配置参数时调用。
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max,
              float integ_min, float integ_max)
{
    pid->kp = kp;               // 保存比例系数
    pid->ki = ki;               // 保存积分系数
    pid->kd = kd;               // 保存微分系数

    pid->integ = 0.0f;          // 积分累计值清零
    pid->prev_err = 0.0f;       // 上一次误差清零

    pid->out_min = out_min;     // 设置输出最小值
    pid->out_max = out_max;     // 设置输出最大值

    pid->integ_min = integ_min; // 设置积分项最小限制
    pid->integ_max = integ_max; // 设置积分项最大限制
}

/**
 * @brief 基于“误差值”运行一次 PID 控制器
 * @param pid 指向 PID 控制器对象的指针
 * @param err 当前误差值
 * @param dt  控制周期，单位：秒
 * @retval 本次 PID 计算输出值
 *
 * @note 该函数是 PID 的核心实现。
 *       它的计算流程是：
 *
 *       1. 积分更新：
 *            integ += err * dt
 *
 *       2. 积分限幅：
 *            防止积分项无限累加，出现积分饱和
 *
 *       3. 微分计算：
 *            deriv = (err - prev_err) / dt
 *
 *       4. PID 输出计算：
 *            out = kp*err + ki*integ + kd*deriv
 *
 *       5. 输出限幅：
 *            把 out 限制在 [out_min, out_max] 范围内
 *
 *       6. 保存当前误差：
 *            为下一次微分计算做准备
 *
 *       注意：
 *       - 这个函数假设 dt 不为 0
 *       - 如果 dt 太小，微分项可能会很大
 */
float PID_RunErr(PID_t *pid, float err, float dt)
{
    float deriv;   // 微分项
    float out;     // PID 总输出

    /* -------------------- 更新积分项 -------------------- */
    /* 积分 = 历史积分 + 当前误差 * 时间 */
    pid->integ += err * dt;

    /* 对积分项做限幅，防止积分累积过大 */
    pid->integ = Limitf(pid->integ, pid->integ_min, pid->integ_max);

    /* -------------------- 计算微分项 -------------------- */
    /* 微分 = 当前误差变化率 */
    deriv = (err - pid->prev_err) / dt;

    /* -------------------- 计算 PID 输出 -------------------- */
    out = pid->kp * err              // 比例项
        + pid->ki * pid->integ       // 积分项
        + pid->kd * deriv;           // 微分项

    /* -------------------- 输出限幅 -------------------- */
    out = Limitf(out, pid->out_min, pid->out_max);

    /* -------------------- 保存历史误差 -------------------- */
    /* 供下一拍微分项计算使用 */
    pid->prev_err = err;

    return out;
}

/**
 * @brief 基于“参考值和反馈值”运行一次 PID 控制器
 * @param pid 指向 PID 控制器对象的指针
 * @param ref 参考值 / 目标值
 * @param fdb 反馈值 / 实际值
 * @param dt  控制周期，单位：秒
 * @retval 本次 PID 计算输出值
 *
 * @note 这是一个更方便的封装函数。
 *       它先自动计算误差：
 *           err = ref - fdb
 *       然后再调用 PID_RunErr() 完成 PID 计算。
 *
 *       适合大多数常规控制场景：
 *       - 已知目标值
 *       - 已知实际反馈值
 *       - 希望直接得到控制器输出
 */
float PID_Run(PID_t *pid, float ref, float fdb, float dt)
{
    return PID_RunErr(pid, ref - fdb, dt);
}

/**
 * @brief 重置 PID 控制器内部状态
 * @param pid 指向 PID 控制器对象的指针
 *
 * @note 该函数只清空“运行状态”，不修改 PID 参数本身。
 *       被清空的内容有：
 *       - 积分项 integ
 *       - 上一次误差 prev_err
 *
 *       常见使用场景：
 *       - 任务切换时
 *       - 控制器重新启动时
 *       - 避免旧积分对新任务造成影响
 */
void PID_Reset(PID_t *pid)
{
    pid->integ = 0.0f;      // 清空积分累计值
    pid->prev_err = 0.0f;   // 清空上一拍误差
}
