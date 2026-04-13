#include "task_mgr.h"  // 任务管理模块头文件
#include "region.h"    // 区域检测模块头文件
#include "protocol_pi.h"  //树莓派协议头文件

/**
 * @brief 任务上下文结构体实例
 * @note 全局静态变量，存储当前任务状态和参数
 */
static TaskContext_t g_task;

/**
 * @brief 任务管理模块初始化
 * @note 设置默认用户路线和任务状态
 * 
 * @details 默认路线设置为1->2->6->9，对应板子上的四个区域
 */
void TaskMgr_Init(void)
{
    memset(&g_task, 0, sizeof(g_task));
    g_task.user_route[0] = 1U;
    g_task.user_route[1] = 2U;
    g_task.user_route[2] = 6U;
    g_task.user_route[3] = 9U;
}

/**
 * @brief 获取任务上下文指针
 * @return 指向任务上下文的指针
 * 
 * @note 用于外部模块直接访问任务状态
 */
TaskContext_t* TaskMgr_GetContext(void)
{
    return &g_task;
}

/**
 * @brief 设置用户自定义路线
 * @param a 路线第一个区域ID
 * @param b 路线第二个区域ID
 * @param c 路线第三个区域ID
 * @param d 路线第四个区域ID
 * 
 * @note 最多支持4个区域的路线规划
 */
void TaskMgr_SetUserRoute(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    g_task.user_route[0] = a;
    g_task.user_route[1] = b;
    g_task.user_route[2] = c;
    g_task.user_route[3] = d;
}

/**
 * @brief 加载指定任务
 * @param task_id 任务ID
 * 
 * @note 重新初始化任务状态，构建任务步骤
 * @note 最多支持12个步骤的任务
 */
void TaskMgr_LoadTask(uint8_t task_id)
{
    memset(g_task.steps, 0, sizeof(g_task.steps));
    g_task.task_id = task_id;
    g_task.current_step = 0U;
    g_task.total_steps = RouteMgr_BuildTask(task_id, g_task.user_route, g_task.steps, 12U);
    g_task.step_time_ms = 0U;
    g_task.hold_count_ms = 0U;
    g_task.finished = 0U;
    g_task.failed = 0U;
}

/**
 * @brief 启动当前任务
 * @note 任务必须有至少一个步骤才能启动
 * 
 * @details 重置任务状态，开始执行任务步骤
 */
void TaskMgr_Start(void)
{
    // 如果是激光追踪，不需要预设步骤，直接让它跑
    if (g_task.task_id == TASK_ID_TRACK_LASER)
    {
        g_task.running = 1U;
    }
    else
    {
        g_task.running = (g_task.total_steps > 0U) ? 1U : 0U;
    }
    
    g_task.finished = 0U;
    g_task.failed = 0U;
    g_task.total_time_ms = 0U;
    g_task.step_time_ms = 0U;
    g_task.hold_count_ms = 0U;
}

/**
 * @brief 停止当前任务
 * @note 立即终止任务执行
 * 
 * @details 设置running标志为0，任务状态保持在当前步骤
 */
void TaskMgr_Stop(void)
{
    g_task.running = 0U;
}

/**
 * @brief 检查任务是否正在运行
 * @return 1表示任务正在运行，0表示任务未运行
 */
uint8_t TaskMgr_IsRunning(void)
{
    return g_task.running;
}

/**
 * @brief 检查任务是否完成
 * @return 1表示任务已完成，0表示任务未完成
 */
uint8_t TaskMgr_IsFinished(void)
{
    return g_task.finished;
}

/**
 * @brief 检查任务是否失败
 * @return 1表示任务失败，0表示任务未失败
 */
uint8_t TaskMgr_IsFailed(void)
{
    return g_task.failed;
}

/**
 * @brief 获取当前任务目标位置和控制模式
 * @param x_mm 输出参数，目标X坐标(毫米)
 * @param y_mm 输出参数，目标Y坐标(毫米)
 * @param mode 输出参数，控制模式
 * 
 * @note 如果任务未运行或已完成，返回板子中心位置和保持模式
 */
/**
 * @brief 获取当前任务目标位置和控制模式
 */
void TaskMgr_GetTarget(float *x_mm, float *y_mm, uint8_t *mode)
{
    // ====== 【新增】处理激光追踪的动态目标 ======
    if (g_task.running && g_task.task_id == TASK_ID_TRACK_LASER)
    {
        PiRxData_t pi_data;
        ProtocolPi_CopyData(&pi_data); // 使用刚写好的安全拷贝函数！
        
        if (pi_data.laser_valid)
        {
            // 看到激光了，球去追激光！速度要快姿势要帅
            *x_mm = pi_data.laser_x_mm;
            *y_mm = pi_data.laser_y_mm;
            *mode = CTRL_MODE_FAST; 
        }
        else
        {
            // 激光突然消失了，为了安全，让球待在原位保持或者去中心
            *x_mm = BOARD_CENTER_X_MM;
            *y_mm = BOARD_CENTER_Y_MM;
            *mode = CTRL_MODE_HOLD;
        }
        return; // 直接返回，不走下面的常规航点逻辑
    }
    // ==========================================

    // ====== 以下为原有常规固定路线逻辑不变 ======
    if (g_task.running && g_task.current_step < g_task.total_steps)
    {
        *x_mm = g_task.steps[g_task.current_step].x_mm;
        *y_mm = g_task.steps[g_task.current_step].y_mm;
        *mode = g_task.steps[g_task.current_step].mode;
    }
    else
    {
        *x_mm = BOARD_CENTER_X_MM;
        *y_mm = BOARD_CENTER_Y_MM;
        *mode = CTRL_MODE_HOLD;
    }
}

/**
 * @brief 每毫秒更新任务状态
 * @param ball_x 球的X坐标(毫米)
 * @param ball_y 球的Y坐标(毫米)
 * @param ball_valid 球位置是否有效
 * 
 * @details 
 * 1. 检查当前步骤是否完成：
 *    - 如果是区域目标，检查球是否进入或保持在区域内
 *    - 如果是点目标，检查球是否接近目标点(30mm内)
 * 2. 如果步骤完成，切换到下一步
 * 3. 如果步骤超时，标记任务失败
 */
void TaskMgr_Update1ms(float ball_x, float ball_y, uint8_t ball_valid)
{
	RouteStep_t *s;
    uint8_t reached = 0U;

    if (!g_task.running) return;

    // 【新增】如果是激光任务，不需要步进检测，它是一个永远跑不完的无限任务，除非被外部强制 Stop
    if (g_task.task_id == TASK_ID_TRACK_LASER)
    {
        g_task.total_time_ms++;
        return; 
    }

    if (!g_task.running || g_task.current_step >= g_task.total_steps)
    {
        return;
    }

    s = &g_task.steps[g_task.current_step];
    g_task.total_time_ms++;
    g_task.step_time_ms++;

	// 修复：先判定是否超时，防止丢球导致状态机死锁
    if (g_task.step_time_ms >= s->timeout_ms)
	{
        g_task.running = 0U;
        g_task.failed = 1U;
        return;
    }
	
    if (!ball_valid)
    {
        return;
    }

    if (s->is_region_target && s->region_id > 0U)
    {
        if (s->need_hold)
        {
            if (Region_IsHeld(s->region_id, ball_x, ball_y))
            {
                g_task.hold_count_ms++;
                if (g_task.hold_count_ms >= s->hold_ms)
                {
                    reached = 1U;
                }
            }
            else
            {
                g_task.hold_count_ms = 0U;
            }
        }
        else if (Region_IsEntered(s->region_id, ball_x, ball_y))
        {
            reached = 1U;
        }
    }
    else
    {
        float dx = ball_x - s->x_mm;
        float dy = ball_y - s->y_mm;
        if ((dx * dx + dy * dy) < SQRF(30.0f))
        {
            reached = 1U;
        }
    }

    if (reached)
    {
        g_task.current_step++;
        g_task.step_time_ms = 0U;
        g_task.hold_count_ms = 0U;
        if (g_task.current_step >= g_task.total_steps)
        {
            g_task.running = 0U;
            g_task.finished = 1U;
        }
        return;
    }

    if (g_task.step_time_ms >= s->timeout_ms)
    {
        g_task.running = 0U;
        g_task.failed = 1U;
    }
}
