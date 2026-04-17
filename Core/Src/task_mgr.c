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
 * @brief 重置任务运行时状态
 */
static void TaskMgr_ResetRuntime(void)
{
    g_task.running = 0U;
    g_task.finished = 0U;
    g_task.failed = 0U;
    g_task.current_step = 0U;
    g_task.total_steps = 0U;
    g_task.total_time_ms = 0U;
    g_task.step_time_ms = 0U;
    g_task.hold_count_ms = 0U;
    memset(g_task.steps, 0, sizeof(g_task.steps));
}

/**
 * @brief 任务管理模块初始化
 */
void TaskMgr_Init(void)
{
    memset(&g_task, 0, sizeof(g_task));

    /* 默认给一个演示路线 */
    g_task.route_region[0] = 1U;
    g_task.route_region[1] = 5U;
    g_task.route_region[2] = 9U;
    g_task.route_len = 3U;
}

/**
 * @brief 获取任务上下文指针
 */
TaskContext_t* TaskMgr_GetContext(void)
{
    return &g_task;
}

/**
 * @brief 加载简单任务
 * @note 这里只保留：
 *       1) 去中心区
 *       2) 激光追踪
 */
void TaskMgr_LoadTask(uint8_t task_id)
{
    TaskMgr_ResetRuntime();
    g_task.task_id = task_id;

    if (task_id == TASK_ID_GOTO_CENTER)
    {
        g_task.current_region_cmd = 5U;
    }
    else if (task_id == TASK_ID_TRACK_LASER)
    {
        /* 激光追踪模式不需要预构建路径 */
    }
}

/**
 * @brief 设置直接目标点
 */
void TaskMgr_SetDirectPoint(float x_mm, float y_mm)
{
    TaskMgr_ResetRuntime();
    g_task.task_id = TASK_ID_GOTO_POINT;
    g_task.point_x_mm = Limitf(x_mm, 0.0f, BOARD_SIZE_MM);
    g_task.point_y_mm = Limitf(y_mm, 0.0f, BOARD_SIZE_MM);
}

/**
 * @brief 设置直接目标区域
 */
void TaskMgr_SetDirectRegion(uint8_t region_id)
{
    TaskMgr_ResetRuntime();
    g_task.task_id = TASK_ID_GOTO_REGION;
    g_task.current_region_cmd = region_id;
}

/**
 * @brief 设置区域序列任务
 * @param route 区域编号序列
 * @param len   序列长度
 * @param pass_mode 0=每点停留，1=中间只经过终点停留
 */
void TaskMgr_SetRouteSequence(const uint8_t *route, uint8_t len, uint8_t pass_mode)
{
    TaskMgr_ResetRuntime();

    if (route == NULL || len == 0U)
    {
        g_task.task_id = TASK_ID_NONE;
        return;
    }

    if (len > USER_ROUTE_MAX_LEN)
    {
        len = USER_ROUTE_MAX_LEN;
    }

    memcpy(g_task.route_region, route, len);
    g_task.route_len = len;
    g_task.route_pass_mode = pass_mode ? 1U : 0U;

    if (g_task.route_pass_mode)
    {
        g_task.task_id = TASK_ID_ROUTE_PASS;
        g_task.total_steps = RouteMgr_BuildPassSequence(g_task.route_region,
                                                        g_task.route_len,
                                                        PASS_END_HOLD_MS,
                                                        g_task.steps,
                                                        TASK_MAX_STEPS);
    }
    else
    {
        g_task.task_id = TASK_ID_ROUTE_HOLD;
        g_task.total_steps = RouteMgr_BuildRegionSequence(g_task.route_region,
                                                          g_task.route_len,
                                                          ROUTE_HOLD_MS,
                                                          g_task.steps,
                                                          TASK_MAX_STEPS);
    }
}

/**
 * @brief 设置两点往返任务
 */
void TaskMgr_SetRoundTrip(uint8_t a, uint8_t b, uint8_t cycles)
{
    TaskMgr_ResetRuntime();

    g_task.task_id = TASK_ID_ROUND_TRIP;
    g_task.roundtrip_a = a;
    g_task.roundtrip_b = b;
    g_task.roundtrip_cycles = cycles;

    g_task.total_steps = RouteMgr_BuildRoundTrip(a, b, cycles,
                                                 ROUNDTRIP_HOLD_MS,
                                                 g_task.steps,
                                                 TASK_MAX_STEPS);
}

/**
 * @brief 启动当前任务
 */
void TaskMgr_Start(void)
{
    if (g_task.task_id == TASK_ID_TRACK_LASER ||
        g_task.task_id == TASK_ID_GOTO_POINT ||
        g_task.task_id == TASK_ID_GOTO_REGION ||
        g_task.task_id == TASK_ID_GOTO_CENTER)
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
    g_task.current_step = 0U;
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
    if (x_mm == NULL || y_mm == NULL || mode == NULL)
    {
        return;
    }

    if (g_task.running && g_task.task_id == TASK_ID_TRACK_LASER)
    {
        PiRxData_t pi_data;
        uint8_t laser_fresh;

        ProtocolPi_CopyData(&pi_data);
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

    if (g_task.running && g_task.task_id == TASK_ID_GOTO_CENTER)
    {
        Point2f_t c = Region_GetCenter(5U);
        *x_mm = c.x;
        *y_mm = c.y;
        *mode = CTRL_MODE_HOLD;
        return;
    }

    if (g_task.running && g_task.task_id == TASK_ID_GOTO_REGION)
    {
        Point2f_t c = Region_GetCenter(g_task.current_region_cmd);
        *x_mm = c.x;
        *y_mm = c.y;
        *mode = CTRL_MODE_HOLD;
        return;
    }

    if (g_task.running && g_task.task_id == TASK_ID_GOTO_POINT)
    {
        *x_mm = g_task.point_x_mm;
        *y_mm = g_task.point_y_mm;
        *mode = CTRL_MODE_HOLD;
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

    /* 这些模式是持续目标模式，不走步骤状态机 */
    if (g_task.task_id == TASK_ID_TRACK_LASER ||
        g_task.task_id == TASK_ID_GOTO_POINT ||
        g_task.task_id == TASK_ID_GOTO_REGION ||
        g_task.task_id == TASK_ID_GOTO_CENTER)
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

    /* 先判定是否超时 */
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

        if ((dx * dx + dy * dy) <= SQRF(20.0f))
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
    }
}
