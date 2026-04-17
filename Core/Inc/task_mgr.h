#ifndef __TASK_MGR_H__
#define __TASK_MGR_H__

/* -------------------- 依赖头文件 -------------------- */
#include "app_config.h"   // 系统全局配置头文件，提供任务参数、路径长度和基础类型
#include "route_mgr.h"    // 路径构建模块，提供 RouteStep_t 定义和路径生成接口

/* -------------------- 任务上下文结构体 -------------------- */
/**
 * @brief 任务管理器上下文
 * @note 这个结构体用于保存当前任务执行过程中的全部状态信息
 *
 *       它相当于“任务管理器的大脑”，里面记录了：
 *       - 当前任务是否在运行
 *       - 任务是否完成/失败
 *       - 当前任务类型
 *       - 当前执行到第几步
 *       - 当前任务总共有几步
 *       - 已运行了多久
 *       - 当前步骤已运行多久
 *       - 在目标点已经保持了多久
 *       - 用户配置的路径/往返参数
 *       - 当前直接点或直接区域命令
 *       - 实际构建出来的步骤序列
 */
typedef struct
{
    /* -------------------- 任务运行状态标志 -------------------- */
    uint8_t running;       // 任务是否正在运行：1=运行中，0=未运行
    uint8_t finished;      // 任务是否已经完成：1=完成，0=未完成
    uint8_t failed;        // 任务是否执行失败：1=失败，0=未失败

    /* -------------------- 当前任务基本信息 -------------------- */
    uint8_t task_id;       // 当前加载的任务 ID
    uint8_t current_step;  // 当前正在执行的步骤编号
    uint8_t total_steps;   // 当前任务总步骤数

    /* -------------------- 时间统计信息 -------------------- */
    uint32_t total_time_ms; // 整个任务已运行总时间，单位：毫秒
    uint32_t step_time_ms;  // 当前步骤已运行时间，单位：毫秒
    uint32_t hold_count_ms; // 当前在目标区域/目标点已持续保持的时间，单位：毫秒

    /* -------------------- 路径任务参数 -------------------- */
    uint8_t route_len;                       // 当前路径任务的长度
    uint8_t route_pass_mode;                // 路径模式：是否为“经过不停留”模式
    uint8_t route_region[USER_ROUTE_MAX_LEN]; // 当前路径点数组，保存区域编号序列

    /* -------------------- 两点往返任务参数 -------------------- */
    uint8_t roundtrip_a;       // 往返点 A 的区域编号
    uint8_t roundtrip_b;       // 往返点 B 的区域编号
    uint8_t roundtrip_cycles;  // 往返循环次数

    /* -------------------- 直接区域命令参数 -------------------- */
    uint8_t current_region_cmd; // 当前直接指定的目标区域编号

    /* -------------------- 直接点命令参数 -------------------- */
    float point_x_mm;          // 当前直接指定目标点 X 坐标，单位：毫米
    float point_y_mm;          // 当前直接指定目标点 Y 坐标，单位：毫米

    /* -------------------- 任务步骤数组 -------------------- */
    RouteStep_t steps[TASK_MAX_STEPS]; // 当前任务被拆解后的步骤序列
} TaskContext_t;

/* -------------------- 任务管理模块接口 -------------------- */

/**
 * @brief 任务管理器初始化
 * @note 用于初始化任务上下文、清空状态和设置默认值
 */
void TaskMgr_Init(void);

/**
 * @brief 加载指定任务
 * @param task_id 任务 ID
 *
 * @note 该函数通常只负责“装载任务类型”，
 *       后续再根据任务参数生成具体步骤。
 */
void TaskMgr_LoadTask(uint8_t task_id);

/* -------------------- 设置任务参数接口 -------------------- */

/**
 * @brief 设置直接目标点任务参数
 * @param x_mm 目标点 X 坐标，单位：毫米
 * @param y_mm 目标点 Y 坐标，单位：毫米
 *
 * @note 用于“直接去某个坐标点”的任务。
 */
void TaskMgr_SetDirectPoint(float x_mm, float y_mm);

/**
 * @brief 设置直接目标区域任务参数
 * @param region_id 目标区域编号
 *
 * @note 用于“直接去某个区域中心”的任务。
 */
void TaskMgr_SetDirectRegion(uint8_t region_id);

/**
 * @brief 设置路径序列任务参数
 * @param route     路径点数组（区域编号序列）
 * @param len       路径长度
 * @param pass_mode 路径模式：0=逐点停留，1=经过不停留
 *
 * @note 用于配置多区域路径任务。
 */
void TaskMgr_SetRouteSequence(const uint8_t *route, uint8_t len, uint8_t pass_mode);

/**
 * @brief 设置两点往返任务参数
 * @param a      往返点 A
 * @param b      往返点 B
 * @param cycles 往返次数
 *
 * @note 用于配置 A、B 两点之间的往返任务。
 */
void TaskMgr_SetRoundTrip(uint8_t a, uint8_t b, uint8_t cycles);

/* -------------------- 任务运行控制接口 -------------------- */

/**
 * @brief 启动当前任务
 * @note 启动后任务状态机会开始运行
 */
void TaskMgr_Start(void);

/**
 * @brief 停止当前任务
 * @note 停止后任务不再继续执行
 */
void TaskMgr_Stop(void);

/**
 * @brief 任务管理器 1ms 更新函数
 * @param ball_x     当前小球 X 坐标，单位：毫米
 * @param ball_y     当前小球 Y 坐标，单位：毫米
 * @param ball_valid 当前小球位置是否有效：1=有效，0=无效
 *
 * @note 这个函数通常在 1ms 定时中断节拍下被周期调用，
 *       用于：
 *       - 更新时间计数
 *       - 判断当前步骤是否到达
 *       - 判断是否需要保持
 *       - 判断是否切换到下一步
 *       - 判断是否超时失败
 */
void TaskMgr_Update1ms(float ball_x, float ball_y, uint8_t ball_valid);

/**
 * @brief 查询任务是否正在运行
 * @retval 1=运行中，0=未运行
 */
uint8_t TaskMgr_IsRunning(void);

/**
 * @brief 查询任务是否已完成
 * @retval 1=已完成，0=未完成
 */
uint8_t TaskMgr_IsFinished(void);

/**
 * @brief 查询任务是否失败
 * @retval 1=失败，0=未失败
 */
uint8_t TaskMgr_IsFailed(void);

/**
 * @brief 获取当前任务目标
 * @param x_mm 返回目标点 X 坐标
 * @param y_mm 返回目标点 Y 坐标
 * @param mode 返回当前控制模式
 *
 * @note 控制器模块通过这个接口获取：
 *       “当前这一步到底要去哪、用什么模式去”
 *
 *       外环控制器通常就靠这个接口来拿目标点。
 */
void TaskMgr_GetTarget(float *x_mm, float *y_mm, uint8_t *mode);

/**
 * @brief 获取任务上下文指针
 * @retval 指向当前任务上下文结构体的指针
 *
 * @note 适合调试、状态显示或其他需要直接访问任务内部状态的场景。
 */
TaskContext_t* TaskMgr_GetContext(void);

#endif
