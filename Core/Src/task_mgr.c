#include "task_mgr.h"
#include "region.h"
#include "protocol_pi.h"

/* 外部声明的系统毫秒计时器 */
extern volatile uint32_t g_sys_ms;

/**
 * @brief 任务上下文结构体实例
 */
static TaskContext_t g_task;

/**
 * @brief 任务管理模块初始化
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
 */
TaskContext_t* TaskMgr_GetContext(void)
{
    return &g_task;
 }

/**
 * @brief 设置用户自定义路线
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
 */
void TaskMgr_Start(void)
{
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
 */
void TaskMgr_Stop(void)
{
    g_task.running = 0U;
}

/**
 * @brief 检查任务是否正在运行
 */
uint8_t TaskMgr_IsRunning(void)
{
    return g_task.running;
}

/**
 * @brief 检查任务是否完成
 */
uint8_t TaskMgr_IsFinished(void)
{
    return g_task.finished;
}

/**
 * @brief 检查任务是否失败
 */
uint8_t TaskMgr_IsFailed(void)
{
    return g_task.failed;
}

/**
 * @brief 获取当前任务目标位置和控制模式
 */
void TaskMgr_GetTarget(float *x_mm, float *y_mm, uint8_t *mode)
{
    if (g_task.running && g_task.task_id == TASK_ID_TRACK_LASER)
    {
        PiRxData_t pi_data;
        uint8_t laser_fresh;

        ProtocolPi_CopyData(&pi_data);

        /* 激光追踪必须同时满足：
           1. laser_valid = 1
           2. 激光数据未超时 */
        laser_fresh = ((g_sys_ms - pi_data.laser_update_ms) <= PI_UART_TIMEOUT_MS) ? 1U : 0U;

        if (pi_data.laser_valid && laser_fresh)
        {
            *x_mm = pi_data.laser_x_mm;
            *y_mm = pi_data.laser_y_mm;
            *mode = CTRL_MODE_FAST;
        }
        else
        {
            *x_mm = BOARD_CENTER_X_MM;
            *y_mm = BOARD_CENTER_Y_MM;
            *mode = CTRL_MODE_HOLD;
        }
        return;
    }

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
 */
void TaskMgr_Update1ms(float ball_x, float ball_y, uint8_t ball_valid)
{
    RouteStep_t *s;
    uint8_t reached = 0U;

    if (!g_task.running)
    {
        return;
    }

    /* 激光任务是持续任务，不走普通步进状态机 */
    if (g_task.task_id == TASK_ID_TRACK_LASER)
    {
        g_task.total_time_ms++;
        return;
    }

    if (g_task.current_step >= g_task.total_steps)
    {
        return;
    }

    s = &g_task.steps[g_task.current_step];
    g_task.total_time_ms++;
    g_task.step_time_ms++;

    /* 先判定是否超时，防止丢球导致状态机死锁 */
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
