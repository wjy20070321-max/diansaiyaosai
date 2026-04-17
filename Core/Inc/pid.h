#ifndef __PID_H__
#define __PID_H__

/* -------------------- 依赖头文件 -------------------- */
#include "app_config.h"   // 系统全局配置头文件，提供基础类型和通用宏

/* -------------------- PID 控制器结构体 -------------------- */
/**
 * @brief PID 控制器对象
 * @note 用于保存一个 PID 控制器运行所需的全部参数和中间状态
 *
 *       PID 的三部分含义：
 *       - kp：比例项，误差越大，输出越大
 *       - ki：积分项，用来消除稳态误差
 *       - kd：微分项，用来抑制变化过快、减少超调
 *
 *       此结构体里除了 PID 参数外，还保存了：
 *       - 积分累计值
 *       - 上一次误差
 *       - 输出限幅范围
 *       - 积分限幅范围
 */
typedef struct
{
    float kp;         // 比例系数 Kp
    float ki;         // 积分系数 Ki
    float kd;         // 微分系数 Kd

    float integ;      // 积分累计值
    float prev_err;   // 上一次误差，用于计算微分项

    float out_min;    // 控制器输出最小值
    float out_max;    // 控制器输出最大值

    float integ_min;  // 积分项最小限制，防止积分过饱和
    float integ_max;  // 积分项最大限制，防止积分过饱和
} PID_t;

/* -------------------- PID 模块接口函数 -------------------- */

/**
 * @brief 初始化 PID 控制器
 * @param pid        PID 控制器对象指针
 * @param kp         比例系数
 * @param ki         积分系数
 * @param kd         微分系数
 * @param out_min    输出最小值
 * @param out_max    输出最大值
 * @param integ_min  积分项最小值
 * @param integ_max  积分项最大值
 *
 * @note 一般在系统启动时调用，用于设置 PID 参数和限幅范围，
 *       同时清空积分项和历史误差。
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max,
              float integ_min, float integ_max);

/**
 * @brief 运行一次 PID 控制（传入参考值和反馈值）
 * @param pid PID 控制器对象指针
 * @param ref 参考目标值
 * @param fdb 当前反馈值
 * @param dt  控制周期，单位：秒
 * @retval PID 输出值
 *
 * @note 该函数内部会先计算误差：
 *           err = ref - fdb
 *       然后根据 PID 公式计算控制输出。
 *
 *       适合常规场景：
 *       - 已知目标值 ref
 *       - 已知实际反馈值 fdb
 *       - 希望自动算出误差并输出控制量
 */
float PID_Run(PID_t *pid, float ref, float fdb, float dt);

/**
 * @brief 运行一次 PID 控制（直接传入误差）
 * @param pid PID 控制器对象指针
 * @param err 当前误差
 * @param dt  控制周期，单位：秒
 * @retval PID 输出值
 *
 * @note 和 PID_Run() 的区别是：
 *       这个函数不再自己计算 ref - fdb，
 *       而是由外部直接把误差 err 传进来。
 *
 *       适合场景：
 *       - 外部已经算好了误差
 *       - 或者误差定义比较特殊，不是简单的 ref - fdb
 */
float PID_RunErr(PID_t *pid, float err, float dt);

/**
 * @brief 重置 PID 控制器内部状态
 * @param pid PID 控制器对象指针
 *
 * @note 调用后通常会清零：
 *       - 积分项 integ
 *       - 上一次误差 prev_err
 *
 *       常用场景：
 *       - 切换控制模式时
 *       - 系统重新开始闭环控制时
 *       - 避免历史积分影响新任务
 */
void PID_Reset(PID_t *pid);

#endif
