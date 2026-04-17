#include "task_mgr.h"     // 任务管理模块头文件
#include "region.h"       // 区域管理模块，用于获取区域中心和判定是否到达区域
#include "protocol_pi.h"  // 树莓派协议模块，用于激光追踪时读取激光目标数据

/* -------------------- 外部系统毫秒计时器 -------------------- */
/* g_sys_ms 在 main.c 中定义，这里通过 extern 引用 */
extern volatile uint32_t g_sys_ms;

/* -------------------- 模块内静态任务上下文 -------------------- */
/* g_task 保存当前任务的全部运行状态和配置参数 */
static TaskContext_t g_task;

/**
 * @brief 重置任务运行时状态
 *
 * @note 这个函数只清理“运行过程中的状态”，
 *       不会把所有任务配置都清空。
 *
 *       被清空的内容包括：
 *       - 运行/完成/失败标志
 *       - 当前步骤号
 *       - 总步骤数
 *       - 时间统计
 *       - 已构建的步骤数组
 *
 *       常见用途：
 *       - 切换新任务前先把旧任务运行状态清空
 *       - 设置新的直接点/区域/路径任务前复位
 */
static void TaskMgr_ResetRuntime(void)
{
    g_task.running = 0U;         // 任务运行标志清零
    g_task.finished = 0U;        // 任务完成标志清零
    g_task.failed = 0U;          // 任务失败标志清零

    g_task.current_step = 0U;    // 当前步骤号清零
    g_task.total_steps = 0U;     // 总步骤数清零

    g_task.total_time_ms = 0U;   // 任务总运行时间清零
    g_task.step_time_ms = 0U;    // 当前步骤运行时间清零
    g_task.hold_count_ms = 0U;   // 当前保持计时清零

    memset(g_task.steps, 0, sizeof(g_task.steps)); // 清空步骤数组
}

/**
 * @brief 任务管理模块初始化
 *
 * @note 初始化时会把整个任务上下文清零，
 *       并给一个默认演示路线。
 *
 *       当前默认路线为：
 *           1 -> 5 -> 9
 *       只是为了上电后方便调试，不一定代表正式任务逻辑。
 */
void TaskMgr_Init(void)
{
    memset(&g_task, 0, sizeof(g_task)); // 清空整个任务上下文

    /* 默认给一个演示路线 */
    g_task.route_region[0] = 1U;
    g_task.route_region[1] = 5U;
    g_task.route_region[2] = 9U;
    g_task.route_len = 3U;
}

/**
 * @brief 获取任务上下文结构体指针
 * @retval 指向当前任务上下文的指针
 *
 * @note 常用于：
 *       - 调试观察任务状态
 *       - 串口屏主动上报任务状态
 *       - 控制器读取任务内部信息
 */
TaskContext_t* TaskMgr_GetContext(void)
{
    return &g_task;
}

/**
 * @brief 加载简单任务
 * @param task_id 任务 ID
 *
 * @note 当前这个函数只直接支持两类简单任务：
 *       1. 去中心区（TASK_ID_GOTO_CENTER）
 *       2. 激光追踪（TASK_ID_TRACK_LASER）
 *
 *       其中：
 *       - 去中心区：本质上是把当前目标区域设置为 5 号区
 *       - 激光追踪：不预先构建固定路径，而是在运行时动态跟踪激光点
 */
void TaskMgr_LoadTask(uint8_t task_id)
{
    /* 先清空旧任务的运行时状态 */
    TaskMgr_ResetRuntime();

    /* 记录当前任务类型 */
    g_task.task_id = task_id;

    /* 去中心区：默认目标就是 5 号区域 */
    if (task_id == TASK_ID_GOTO_CENTER)
    {
        g_task.current_region_cmd = 5U;
    }
    /* 激光追踪：这里不需要预先构建步骤路径 */
    else if (task_id == TASK_ID_TRACK_LASER)
    {
        /* 激光追踪模式不需要预构建路径 */
    }
}

/**
 * @brief 设置直接目标点任务
 * @param x_mm 目标点 X 坐标，单位：毫米
 * @param y_mm 目标点 Y 坐标，单位：毫米
 *
 * @note 这是“直接去某个坐标点”的任务。
 *       这里会先清空旧任务运行状态，
 *       再把新目标点保存到 g_task 中。
 *
 *       同时会对目标点做限幅，
 *       保证坐标不会超出板面范围。
 */
void TaskMgr_SetDirectPoint(float x_mm, float y_mm)
{
    TaskMgr_ResetRuntime();           // 清空旧任务运行状态
    g_task.task_id = TASK_ID_GOTO_POINT; // 标记当前任务类型为“去指定点”

    /* 将目标点限制在板子有效范围 [0, BOARD_SIZE_MM] 内 */
    g_task.point_x_mm = Limitf(x_mm, 0.0f, BOARD_SIZE_MM);
    g_task.point_y_mm = Limitf(y_mm, 0.0f, BOARD_SIZE_MM);
}

/**
 * @brief 设置直接目标区域任务
 * @param region_id 目标区域编号
 *
 * @note 这是“直接去某个区域”的任务。
 *       后续真正执行时，会通过 Region_GetCenter() 把区域编号转换成区域中心坐标。
 */
void TaskMgr_SetDirectRegion(uint8_t region_id)
{
    TaskMgr_ResetRuntime();              // 清空旧任务运行状态
    g_task.task_id = TASK_ID_GOTO_REGION; // 标记当前任务类型为“去指定区域”
    g_task.current_region_cmd = region_id; // 保存目标区域编号
}

/**
 * @brief 设置区域序列任务
 * @param route     区域编号序列
 * @param len       路径长度
 * @param pass_mode 0=每点停留，1=中间只经过终点停留
 *
 * @note 这个函数负责把用户给出的区域序列，转换成真正可执行的步骤数组。
 *
 *       两种模式：
 *       1. pass_mode = 0
 *          使用 RouteMgr_BuildRegionSequence()
 *          表示每到一个点都停下来保持一段时间
 *
 *       2. pass_mode = 1
 *          使用 RouteMgr_BuildPassSequence()
 *          表示中间点只经过，最后一个点才停下来
 */
void TaskMgr_SetRouteSequence(const uint8_t *route, uint8_t len, uint8_t pass_mode)
{
    TaskMgr_ResetRuntime(); // 清空旧任务运行状态

    /* 参数保护：空指针或长度为 0，则直接置为空任务 */
    if (route == NULL || len == 0U)
    {
        g_task.task_id = TASK_ID_NONE;
        return;
    }

    /* 路径长度不能超过系统允许的最大长度 */
    if (len > USER_ROUTE_MAX_LEN)
    {
        len = USER_ROUTE_MAX_LEN;
    }

    /* 保存用户设置的原始路径信息 */
    memcpy(g_task.route_region, route, len);
    g_task.route_len = len;
    g_task.route_pass_mode = pass_mode ? 1U : 0U;

    /* 如果是“经过不停留”模式 */
    if (g_task.route_pass_mode)
    {
        g_task.task_id = TASK_ID_ROUTE_PASS;

        /* 构建步骤序列：中间经过，终点保持 */
        g_task.total_steps = RouteMgr_BuildPassSequence(g_task.route_region,
                                                        g_task.route_len,
                                                        PASS_END_HOLD_MS,
                                                        g_task.steps,
                                                        TASK_MAX_STEPS);
    }
    /* 如果是“逐点停留”模式 */
    else
    {
        g_task.task_id = TASK_ID_ROUTE_HOLD;

        /* 构建步骤序列：每个点都保持 */
        g_task.total_steps = RouteMgr_BuildRegionSequence(g_task.route_region,
                                                          g_task.route_len,
                                                          ROUTE_HOLD_MS,
                                                          g_task.steps,
                                                          TASK_MAX_STEPS);
    }
}

/**
 * @brief 设置两点往返任务
 * @param a      区域 A
 * @param b      区域 B
 * @param cycles 往返循环次数
 *
 * @note 该函数会：
 *       1. 清空旧任务运行状态
 *       2. 记录 A/B/循环次数
 *       3. 调用 RouteMgr_BuildRoundTrip() 生成实际步骤数组
 */
void TaskMgr_SetRoundTrip(uint8_t a, uint8_t b, uint8_t cycles)
{
    TaskMgr_ResetRuntime();               // 清空旧任务运行状态

    g_task.task_id = TASK_ID_ROUND_TRIP; // 标记任务类型
    g_task.roundtrip_a = a;              // 保存往返点 A
    g_task.roundtrip_b = b;              // 保存往返点 B
    g_task.roundtrip_cycles = cycles;    // 保存往返次数

    /* 构建两点往返的具体步骤数组 */
    g_task.total_steps = RouteMgr_BuildRoundTrip(a, b, cycles,
                                                 ROUNDTRIP_HOLD_MS,
                                                 g_task.steps,
                                                 TASK_MAX_STEPS);
}

/**
 * @brief 启动当前任务
 *
 * @note 启动时会根据任务类型决定是否允许进入 running 状态。
 *
 *       这类任务不依赖 steps 数组，也能直接运行：
 *       - 激光追踪
 *       - 去指定点
 *       - 去指定区域
 *       - 去中心
 *
 *       其他任务则必须要求 total_steps > 0，
 *       说明已经成功构建了至少一步任务，才允许启动。
 *
 *       启动时还会统一清零：
 *       - finished
 *       - failed
 *       - 各类时间计数
 *       - 当前步骤号
 */
void TaskMgr_Start(void)
{
    /* 这些任务属于“持续目标模式”或“直接目标模式”，不依赖步骤数组 */
    if (g_task.task_id == TASK_ID_TRACK_LASER ||
        g_task.task_id == TASK_ID_GOTO_POINT ||
        g_task.task_id == TASK_ID_GOTO_REGION ||
        g_task.task_id == TASK_ID_GOTO_CENTER)
    {
        g_task.running = 1U;
    }
    else
    {
        /* 依赖步骤数组的任务，必须先有至少一个有效步骤 */
        g_task.running = (g_task.total_steps > 0U) ? 1U : 0U;
    }

    /* 每次启动时都把运行状态重新置为“刚开始” */
    g_task.finished = 0U;
    g_task.failed = 0U;
    g_task.total_time_ms = 0U;
    g_task.step_time_ms = 0U;
    g_task.hold_count_ms = 0U;
    g_task.current_step = 0U;
}

/**
 * @brief 停止当前任务
 *
 * @note 当前实现中，停止任务就是简单地把 running 清零。
 *       不会自动标记 finished 或 failed。
 */
void TaskMgr_Stop(void)
{
    g_task.running = 0U;
}

/**
 * @brief 查询当前任务是否在运行
 * @retval 1=运行中，0=未运行
 */
uint8_t TaskMgr_IsRunning(void)
{
    return g_task.running;
}

/**
 * @brief 查询当前任务是否已完成
 * @retval 1=已完成，0=未完成
 */
uint8_t TaskMgr_IsFinished(void)
{
    return g_task.finished;
}

/**
 * @brief 查询当前任务是否已失败
 * @retval 1=失败，0=未失败
 */
uint8_t TaskMgr_IsFailed(void)
{
    return g_task.failed;
}

/**
 * @brief 获取当前任务目标位置和控制模式
 * @param x_mm 返回目标 X 坐标
 * @param y_mm 返回目标 Y 坐标
 * @param mode 返回当前控制模式
 *
 * @note 这个函数是控制器最常用的接口之一。
 *       控制器每次要运行外环前，都会来这里问：
 *       “我当前到底应该把球往哪里送？”
 *
 *       根据不同任务类型，会有不同的目标来源：
 *       1. 激光追踪：实时读取树莓派给的激光坐标
 *       2. 去中心：固定给 5 号区中心
 *       3. 去区域：根据区域号找中心
 *       4. 去指定点：直接返回保存的点坐标
 *       5. 步骤型任务：返回当前步骤的目标点和模式
 *       6. 其他情况：默认回板中心并 HOLD
 */
void TaskMgr_GetTarget(float *x_mm, float *y_mm, uint8_t *mode)
{
    /* 参数保护 */
    if (x_mm == NULL || y_mm == NULL || mode == NULL)
    {
        return;
    }

    /* -------------------- 激光追踪任务 -------------------- */
    if (g_task.running && g_task.task_id == TASK_ID_TRACK_LASER)
    {
        PiRxData_t pi_data;   // 树莓派数据快照
        uint8_t laser_fresh;  // 激光数据是否新鲜（未超时）

        /* 先安全拷贝一份树莓派数据 */
        ProtocolPi_CopyData(&pi_data);

        /* 判断激光数据是否超时 */
        laser_fresh = ((g_sys_ms - pi_data.laser_update_ms) <= PI_UART_TIMEOUT_MS) ? 1U : 0U;

        /* 如果激光点有效且未超时，就跟踪激光点 */
        if (pi_data.laser_valid && laser_fresh)
        {
            *x_mm = pi_data.laser_x_mm;
            *y_mm = pi_data.laser_y_mm;
            *mode = CTRL_MODE_FAST; // 激光追踪默认用快速模式
        }
        else
        {
            /* 如果激光数据无效或超时，则退回板中心 */
            *x_mm = BOARD_CENTER_X_MM;
            *y_mm = BOARD_CENTER_Y_MM;
            *mode = CTRL_MODE_HOLD;
        }
        return;
    }

    /* -------------------- 去中心任务 -------------------- */
    if (g_task.running && g_task.task_id == TASK_ID_GOTO_CENTER)
    {
        Point2f_t c = Region_GetCenter(5U); // 5 号区默认是中心区
        *x_mm = c.x;
        *y_mm = c.y;
        *mode = CTRL_MODE_HOLD;
        return;
    }

    /* -------------------- 去指定区域任务 -------------------- */
    if (g_task.running && g_task.task_id == TASK_ID_GOTO_REGION)
    {
        Point2f_t c = Region_GetCenter(g_task.current_region_cmd);
        *x_mm = c.x;
        *y_mm = c.y;
        *mode = CTRL_MODE_HOLD;
        return;
    }

    /* -------------------- 去指定点任务 -------------------- */
    if (g_task.running && g_task.task_id == TASK_ID_GOTO_POINT)
    {
        *x_mm = g_task.point_x_mm;
        *y_mm = g_task.point_y_mm;
        *mode = CTRL_MODE_HOLD;
        return;
    }

    /* -------------------- 步骤型任务 -------------------- */
    if (g_task.running && g_task.current_step < g_task.total_steps)
    {
        *x_mm = g_task.steps[g_task.current_step].x_mm;
        *y_mm = g_task.steps[g_task.current_step].y_mm;
        *mode = g_task.steps[g_task.current_step].mode;
    }
    else
    {
        /* 默认安全回中心 */
        *x_mm = BOARD_CENTER_X_MM;
        *y_mm = BOARD_CENTER_Y_MM;
        *mode = CTRL_MODE_HOLD;
    }
}

/**
 * @brief 每 1ms 更新一次任务状态
 * @param ball_x     当前小球 X 坐标，单位：毫米
 * @param ball_y     当前小球 Y 坐标，单位：毫米
 * @param ball_valid 当前球坐标是否有效：1=有效，0=无效
 *
 * @note 这是任务状态机的核心推进函数。
 *
 *       它每 1ms 会做这些事情：
 *       1. 如果任务没在跑，直接返回
 *       2. 对持续目标模式，只累计总时间，不走步骤状态机
 *       3. 对步骤型任务：
 *          - 累计时间
 *          - 判断是否超时
 *          - 判断球是否到达当前目标
 *          - 判断是否已满足保持时间
 *          - 决定是否切换到下一步
 *          - 如果全部完成，则标记任务 finished
 */
void TaskMgr_Update1ms(float ball_x, float ball_y, uint8_t ball_valid)
{
    RouteStep_t *s;      // 当前步骤指针
    uint8_t reached = 0U; // 当前步骤是否已达成

    /* 如果任务没在运行，直接返回 */
    if (!g_task.running)
    {
        return;
    }

    /* -------------------- 持续目标模式 -------------------- */
    /* 这些任务不依赖 steps 数组，只要持续运行即可 */
    if (g_task.task_id == TASK_ID_TRACK_LASER ||
        g_task.task_id == TASK_ID_GOTO_POINT ||
        g_task.task_id == TASK_ID_GOTO_REGION ||
        g_task.task_id == TASK_ID_GOTO_CENTER)
    {
        g_task.total_time_ms++; // 只累计总运行时间
        return;
    }

    /* 如果当前步骤号已经超过总步骤数，说明没有可执行步骤 */
    if (g_task.current_step >= g_task.total_steps)
    {
        return;
    }

    /* 取出当前正在执行的步骤 */
    s = &g_task.steps[g_task.current_step];

    /* 更新时间统计 */
    g_task.total_time_ms++;
    g_task.step_time_ms++;

    /* -------------------- 超时判断 -------------------- */
    /* 如果当前这一步已经跑太久还没完成，则判定任务失败 */
    if (g_task.step_time_ms >= s->timeout_ms)
    {
        g_task.running = 0U;
        g_task.failed = 1U;
        return;
    }

    /* 如果当前球坐标无效，则暂时不能做“是否到达目标”的判断 */
    if (!ball_valid)
    {
        return;
    }

    /* -------------------- 区域目标判定 -------------------- */
    if (s->is_region_target && s->region_id > 0U)
    {
        /* 如果这一目标要求保持 */
        if (s->need_hold)
        {
            /* 只有当球稳定保持在区域内时，保持计时才会增加 */
            if (Region_IsHeld(s->region_id, ball_x, ball_y))
            {
                g_task.hold_count_ms++;

                /* 一旦保持时间达到要求，就认为该步骤完成 */
                if (g_task.hold_count_ms >= s->hold_ms)
                {
                    reached = 1U;
                }
            }
            else
            {
                /* 一旦中途离开稳定保持区域，保持计时清零重新计 */
                g_task.hold_count_ms = 0U;
            }
        }
        /* 如果这一目标不要求保持，只要进入区域就算到达 */
        else if (Region_IsEntered(s->region_id, ball_x, ball_y))
        {
            reached = 1U;
        }
    }
    /* -------------------- 普通坐标点判定 -------------------- */
    else
    {
        float dx = ball_x - s->x_mm;
        float dy = ball_y - s->y_mm;

        /* 如果距离目标点的平方小于等于 20mm 半径阈值，则算到达 */
        if ((dx * dx + dy * dy) <= SQRF(20.0f))
        {
            reached = 1U;
        }
    }

    /* -------------------- 如果当前步骤已完成 -------------------- */
    if (reached)
    {
        g_task.current_step++;    // 进入下一步
        g_task.step_time_ms = 0U; // 当前步骤计时清零
        g_task.hold_count_ms = 0U;// 当前保持计时清零

        /* 如果已经没有后续步骤了，则整个任务完成 */
        if (g_task.current_step >= g_task.total_steps)
        {
            g_task.running = 0U;
            g_task.finished = 1U;
        }
    }
}
