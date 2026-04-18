#include "controller_mgr.h"     // 控制器总管理模块头文件
#include "protocol_pi.h"        // 树莓派协议模块
#include "protocol_screen.h"    // 串口屏协议模块
#include "jy61p.h"              // JY61P 姿态传感器模块
#include "task_mgr.h"           // 任务管理模块
#include "ball_outer_loop.h"    // 小球位置外环控制模块
#include "plate_inner_loop.h"   // 平台姿态内环控制模块
#include "servo.h"              // 舵机控制模块
#include "safety_mgr.h"         // 安全保护模块

/* -------------------- 全局系统上下文 -------------------- */
/* g_sys 是整个控制系统的总状态仓库 */
SystemContext_t g_sys;

/* -------------------- 串口屏当前选中的任务号 -------------------- */
/* 【新增】
   用于支持“先发 TASK=n 选择任务，再发参数，最后 START 执行”的流程 */
static uint8_t g_screen_selected_task_id = TASK_ID_NONE;

/* -------------------- 串口屏任务别名判断 -------------------- */
/* 【新增】
   按当前题目和你的串口屏约定：
   - TASK=1
   - TASK=2
   - TASK=4
   都统一视为“去指定区域并停留”这一类任务。

   也就是说，这三个任务号在串口屏协议层是别名，
   最终在 MCU 内部统一映射成 TaskMgr_SetDirectRegion(...)。 */
static uint8_t ScreenTask_IsRegionHoldAlias(uint8_t task_id)
{
    if (task_id == TASK_ID_GOTO_CENTER) return 1U;  // 1
    if (task_id == TASK_ID_GOTO_REGION) return 1U;  // 2
    if (task_id == TASK_ID_ROUTE_HOLD) return 1U;   // 4（协议层兼容当作区域保持任务）
    return 0U;
}

/* ======== 调试镜像变量：任务层 ======== */
/* 这些变量方便你在调试窗口里直接观察任务状态 */
volatile uint8_t  dbg_task_id = 0U;         // 当前任务 ID
volatile uint8_t  dbg_task_running = 0U;    // 当前任务是否在运行
volatile uint8_t  dbg_route0 = 0U;          // 当前路径第 1 个点
volatile uint8_t  dbg_route1 = 0U;          // 当前路径第 2 个点
volatile uint8_t  dbg_route2 = 0U;          // 当前路径第 3 个点
volatile uint8_t  dbg_route3 = 0U;  // 当前路径第 4 个点
volatile uint32_t dbg_run10ms_exec_cnt = 0U;
volatile float    dbg_outer_theta_x = 0.0f;
volatile float    dbg_outer_theta_y = 0.0f;
volatile uint32_t dbg_ball_age_ms = 0U;

/* ======== 调试镜像变量：控制层 ======== */
/* 这些变量方便你观察当前球状态和控制目标 */
volatile float    dbg_ball_x_mm = 0.0f;     // 当前球 X 坐标
volatile float    dbg_ball_y_mm = 0.0f;     // 当前球 Y 坐标
volatile uint8_t  dbg_ball_valid = 0U;      // 当前球数据是否有效
volatile uint32_t dbg_ball_tick_ms = 0U;    // 当前球数据时间戳

volatile float    dbg_target_x_mm = 0.0f;   // 当前目标 X 坐标
volatile float    dbg_target_y_mm = 0.0f;   // 当前目标 Y 坐标
volatile uint8_t  dbg_ctrl_mode = 0U;       // 当前控制模式

/* ======== 调试镜像变量：树莓派原始快照 ======== */
/* 用来区分“树莓派发来的原始数据”和“系统当前采用的数据” */
volatile float    dbg_pi_ball_x_mm = 0.0f;
volatile float    dbg_pi_ball_y_mm = 0.0f;
volatile float    dbg_pi_ball_vx_mmps = 0.0f;
volatile float    dbg_pi_ball_vy_mmps = 0.0f;
volatile uint8_t  dbg_pi_ball_valid = 0U;
volatile uint32_t dbg_pi_ball_tick_ms = 0U;

volatile float    dbg_pi_laser_x_mm = 0.0f;
volatile float    dbg_pi_laser_y_mm = 0.0f;
volatile uint8_t  dbg_pi_laser_valid = 0U;
volatile uint32_t dbg_pi_laser_tick_ms = 0U;

/* -------------------- 外部系统毫秒计数 -------------------- */
/* g_sys_ms 在 main.c 中定义，这里通过 extern 引用 */
extern volatile uint32_t g_sys_ms;

/**
 * @brief 控制器管理模块初始化
 *
 * @note 初始化内容：
 *       1. 清空整个系统上下文 g_sys
 *       2. 默认目标设到平台中心
 *       3. 默认控制模式设为 HOLD
 */
void ControllerMgr_Init(void)
{
    memset(&g_sys, 0, sizeof(g_sys));      // 清空系统上下文
    g_sys.ref.target_x_mm = BOARD_CENTER_X_MM; // 默认目标 X = 板中心
    g_sys.ref.target_y_mm = BOARD_CENTER_Y_MM; // 默认目标 Y = 板中心
    g_sys.ref.mode = CTRL_MODE_HOLD;           // 默认进入保持模式

    /* 【新增】清空串口屏当前选中的任务号 */
    g_screen_selected_task_id = TASK_ID_NONE;
}

/**
 * @brief 更新系统输入数据
 *
 * @note 这是整个控制器的“输入汇总函数”。
 *       它会统一收集：
 *       - 树莓派视觉数据
 *       - IMU 数据
 *       - 树莓派下发的任务 / 路径 / 目标命令
 *       - 串口屏下发的任务 / 路径 / 区域命令
 *
 *       最终结果会同步到：
 *       - g_sys（系统上下文）
 *       - TaskMgr（任务管理器）
 */
void ControllerMgr_UpdateInputs(void)
{
    PiRxData_t pi;                     // 树莓派数据快照
    JY61P_Data_t imu;                  // IMU 数据快照
    ScreenRxData_t *screen = ProtocolScreen_GetData(); // 串口屏解析结果指针

    /* 树莓派一次性命令的临时变量 */
    uint8_t task_id = TASK_ID_NONE;
    uint8_t start_cmd = 0U;
    uint8_t stop_cmd = 0U;

    /* 树莓派路线命令的临时变量 */
    uint8_t route_a = 0U, route_b = 0U, route_c = 0U, route_d = 0U;

    /* 树莓派直接目标点的临时变量 */
    float target_x = 0.0f;
    float target_y = 0.0f;

    /* 用于整理树莓派路线点的数组 */
    uint8_t pi_route[4];
    uint8_t pi_route_len = 0U;

    /* 任务上下文指针，用于调试镜像 */
    TaskContext_t *ctx;

    /* -------------------- 先复制连续状态 -------------------- */
    ProtocolPi_CopyData(&pi);   // 安全复制一份树莓派数据
    JY61P_CopyData(&imu);       // 安全复制一份 IMU 数据

    /* ======== 原始树莓派快照：写到调试镜像 ======== */
    dbg_pi_ball_x_mm = pi.ball_x_mm;
    dbg_pi_ball_y_mm = pi.ball_y_mm;
    dbg_pi_ball_vx_mmps = pi.ball_vx_mmps;
    dbg_pi_ball_vy_mmps = pi.ball_vy_mmps;
    dbg_pi_ball_valid = pi.ball_valid;
    dbg_pi_ball_tick_ms = pi.ball_update_ms;

    dbg_pi_laser_x_mm = pi.laser_x_mm;
    dbg_pi_laser_y_mm = pi.laser_y_mm;
    dbg_pi_laser_valid = pi.laser_valid;
    dbg_pi_laser_tick_ms = pi.laser_update_ms;

    /* ======== 连续状态写入系统上下文 g_sys ======== */
    g_sys.ball.x_mm = pi.ball_x_mm;          // 球 X 坐标
    g_sys.ball.y_mm = pi.ball_y_mm;          // 球 Y 坐标
    g_sys.ball.vx_mmps = pi.ball_vx_mmps;    // 球 X 速度
    g_sys.ball.vy_mmps = pi.ball_vy_mmps;    // 球 Y 速度
    g_sys.ball.valid = pi.ball_valid;        // 球是否有效
    g_sys.ball.tick_ms = pi.ball_update_ms;  // 球状态更新时间

    g_sys.imu = imu;                         // IMU 整体写入系统上下文

    /* ======== 树莓派命令：路线 ======== */
    /* 读取后会自动清空，避免重复执行 */
    if (ProtocolPi_ConsumeRoute(&route_a, &route_b, &route_c, &route_d))
    {
        pi_route_len = 0U;

        /* 只保留合法区域号 1~9 */
        if (route_a >= 1U && route_a <= 9U) pi_route[pi_route_len++] = route_a;
        if (route_b >= 1U && route_b <= 9U) pi_route[pi_route_len++] = route_b;
        if (route_c >= 1U && route_c <= 9U) pi_route[pi_route_len++] = route_c;
        if (route_d >= 1U && route_d <= 9U) pi_route[pi_route_len++] = route_d;

        /* 如果至少有一个有效区域点，就设置路线任务并启动 */
        if (pi_route_len > 0U)
        {
            TaskMgr_SetRouteSequence(pi_route, pi_route_len, 0U);
            TaskMgr_Start();
        }
    }

    /* ======== 树莓派命令：直接目标点 ======== */
    if (ProtocolPi_ConsumeTarget(&target_x, &target_y))
    {
        TaskMgr_SetDirectPoint(target_x, target_y);
        TaskMgr_Start();
    }

    /* ======== 树莓派命令：任务控制 ======== */
    if (ProtocolPi_ConsumeTaskCtrl(&task_id, &start_cmd, &stop_cmd))
    {
        /* 当前只直接支持两类简单任务 */
        if (task_id == TASK_ID_GOTO_CENTER || task_id == TASK_ID_TRACK_LASER)
        {
            TaskMgr_LoadTask(task_id);
        }

        if (start_cmd)
        {
            TaskMgr_Start();
        }

        if (stop_cmd)
        {
            TaskMgr_Stop();
        }
    }

    /* ======== 串口屏任务号选择 ======== */
    /* 【修改】
       现在把串口屏侧 TASK=1 / TASK=2 / TASK=4
       统一看作“去指定区域并停留”的同类任务。 */
    if (screen->task_id != TASK_ID_NONE)
    {
        if (ScreenTask_IsRegionHoldAlias(screen->task_id))
        {
            /* 【修改】这三类任务在 MCU 内部统一映射成“去指定区域” */
            g_screen_selected_task_id = TASK_ID_GOTO_REGION;
        }
        else
        {
            g_screen_selected_task_id = screen->task_id;
        }

        /* 激光追踪不依赖额外参数，可以直接先装载 */
        if (g_screen_selected_task_id == TASK_ID_TRACK_LASER)
        {
            TaskMgr_LoadTask(g_screen_selected_task_id);
        }

        screen->task_id = TASK_ID_NONE; // 命令消费后清零
    }

    /* ======== 串口屏命令：如果没有先选 TASK，就保持原来的“直接执行”行为 ======== */
    if (g_screen_selected_task_id == TASK_ID_NONE)
    {
        /* ======== 串口屏命令：单点（区域 1~9） ======== */
        /* POINT=n 现在表示区域号，而不是坐标 */
        if (screen->point_valid)
        {
            TaskMgr_SetDirectRegion(screen->point_region_id);
            TaskMgr_Start();
            screen->point_valid = 0U; // 用完清零，防止重复执行
        }

        /* ======== 串口屏命令：指定区域 ======== */
        if (screen->region_valid)
        {
            TaskMgr_SetDirectRegion(screen->region_id);
            TaskMgr_Start();
            screen->region_valid = 0U;
        }

        /* ======== 串口屏命令：区域序列 ======== */
        if (screen->route_valid)
        {
            TaskMgr_SetRouteSequence(screen->route, screen->route_len, screen->route_pass_mode);
            TaskMgr_Start();
            screen->route_valid = 0U;
        }

        /* ======== 串口屏命令：两点往返（默认 2 次循环） ======== */
        if (screen->roundtrip_valid)
        {
            TaskMgr_SetRoundTrip(screen->roundtrip_a, screen->roundtrip_b, 2U);
            TaskMgr_Start();
            screen->roundtrip_valid = 0U;
        }
    }
    else
    {
        /* ======== 串口屏命令：如果已经先选了 TASK，则先装载参数，不立即启动 ======== */
        switch (g_screen_selected_task_id)
        {
            case TASK_ID_GOTO_REGION:
                /* 【修改】
                   串口屏侧 TASK=1 / 2 / 4 最终都会走到这里。
                   目标由 REGION=n 或 POINT=n 给出。 */
                if (screen->region_valid)
                {
                    TaskMgr_SetDirectRegion(screen->region_id);
                    screen->region_valid = 0U;
                }
                else if (screen->point_valid)
                {
                    TaskMgr_SetDirectRegion(screen->point_region_id);
                    screen->point_valid = 0U;
                }
                break;

            case TASK_ID_GOTO_POINT:
                /* 当前保留原兼容处理：
                   由于串口屏已经没有 POINT=x,y，POINT=n 也表示区域号，
                   所以这里临时仍按“去指定区域”处理。 */
                if (screen->point_valid)
                {
                    TaskMgr_SetDirectRegion(screen->point_region_id);
                    screen->point_valid = 0U;
                }
                else if (screen->region_valid)
                {
                    TaskMgr_SetDirectRegion(screen->region_id);
                    screen->region_valid = 0U;
                }
                break;

            case TASK_ID_ROUTE_HOLD:
                /* 多区域顺序停留，配合 ROUTE=a,b,c... 使用 */
                if (screen->route_valid)
                {
                    TaskMgr_SetRouteSequence(screen->route, screen->route_len, 0U);
                    screen->route_valid = 0U;
                }
                break;

            case TASK_ID_ROUND_TRIP:
                /* 两点往返，配合 ROUND=a,b 使用 */
                if (screen->roundtrip_valid)
                {
                    TaskMgr_SetRoundTrip(screen->roundtrip_a, screen->roundtrip_b, 2U);
                    screen->roundtrip_valid = 0U;
                }
                break;

            case TASK_ID_ROUTE_PASS:
                /* 路径经过不停留，配合 PASS=a,b,c... 或 ROUTE=a,b,c... 使用 */
                if (screen->route_valid)
                {
                    TaskMgr_SetRouteSequence(screen->route, screen->route_len, 1U);
                    screen->route_valid = 0U;
                }
                break;

            case TASK_ID_TRACK_LASER:
                /* 已在上面 LoadTask，这里不用额外参数 */
                break;

            default:
                break;
        }
    }

    /* 串口屏启动命令 */
    if (screen->start_cmd)
    {
        TaskMgr_Start();

        /* 【新增】激光追踪需要保持当前任务选择，其余启动后清掉 */
        if (g_screen_selected_task_id != TASK_ID_TRACK_LASER)
        {
            g_screen_selected_task_id = TASK_ID_NONE;
        }

        screen->start_cmd = 0U;
    }

    /* 串口屏停止命令 */
    if (screen->stop_cmd)
    {
        TaskMgr_Stop();

        /* 【新增】停止时也把当前任务选择清掉 */
        g_screen_selected_task_id = TASK_ID_NONE;

        screen->stop_cmd = 0U;
    }

    /* 串口屏 IMU 置零命令 */
    if (screen->imu_zero_cmd)
    {
        JY61P_SetZero();
        screen->imu_zero_cmd = 0U;
    }

    /* ======== 调试镜像 ======== */
    ctx = TaskMgr_GetContext();

    dbg_task_id = ctx->task_id;
    dbg_task_running = ctx->running;
    dbg_route0 = ctx->route_region[0];
    dbg_route1 = ctx->route_region[1];
    dbg_route2 = ctx->route_region[2];
    dbg_route3 = ctx->route_region[3];

    dbg_ball_x_mm = g_sys.ball.x_mm;
    dbg_ball_y_mm = g_sys.ball.y_mm;
    dbg_ball_valid = g_sys.ball.valid;
    dbg_ball_tick_ms = g_sys.ball.tick_ms;

    dbg_target_x_mm = g_sys.ref.target_x_mm;
    dbg_target_y_mm = g_sys.ref.target_y_mm;
    dbg_ctrl_mode = g_sys.ref.mode;
}

/**
 * @brief 10ms 控制任务
 *
 * @note 这是外环控制函数，主要负责：
 *       1. 从任务管理器取当前目标点
 *       2. 根据球当前状态运行外环控制器
 *       3. 计算平台目标倾角
 */
void ControllerMgr_Run10ms(void)
{
	dbg_run10ms_exec_cnt++;
	dbg_ball_age_ms = g_sys_ms - g_sys.ball.tick_ms;
	
    BallOuterOutput_t outer; // 外环输出：平台目标倾角

    /* 先从任务管理器读取当前目标点和模式 */
    TaskMgr_GetTarget(&g_sys.ref.target_x_mm, &g_sys.ref.target_y_mm, &g_sys.ref.mode);

    /* 如果当前没有任务在运行，则默认目标回到平台中心 */
    if (!TaskMgr_IsRunning())
    {
        g_sys.ref.target_x_mm = BOARD_CENTER_X_MM;
        g_sys.ref.target_y_mm = BOARD_CENTER_Y_MM;
        g_sys.ref.mode = CTRL_MODE_HOLD;
    }

    /* 更新调试镜像 */
    dbg_target_x_mm = g_sys.ref.target_x_mm;
    dbg_target_y_mm = g_sys.ref.target_y_mm;
    dbg_ctrl_mode = g_sys.ref.mode;

    /* 如果球无效，则不运行外环 */
    if (!g_sys.ball.valid)
    {
        BallOuterLoop_Reset(); // 重置外环历史状态

#if BALL_LOST_SAFE_CENTER
        /* 如果配置了丢球安全回中，则把目标倾角清零 */
        g_sys.ref.theta_x_ref_deg = 0.0f;
        g_sys.ref.theta_y_ref_deg = 0.0f;
#endif
        return;
    }

    /* 运行外环，计算平台目标倾角 */
    BallOuterLoop_Run(g_sys.ref.target_x_mm, g_sys.ref.target_y_mm,
                      g_sys.ball.x_mm, g_sys.ball.y_mm,
                      g_sys.ball.vx_mmps, g_sys.ball.vy_mmps,
                      g_sys.ref.mode, &outer);

	dbg_outer_theta_x = outer.theta_x_ref_deg;
	dbg_outer_theta_y = outer.theta_y_ref_deg;
	
    /* 写回系统上下文 */
    g_sys.ref.theta_x_ref_deg = outer.theta_x_ref_deg;
    g_sys.ref.theta_y_ref_deg = outer.theta_y_ref_deg;

    /* 如果球靠近边缘，则将倾角减半，降低风险 */
    if (SafetyMgr_CheckBallNearEdge(g_sys.ball.x_mm, g_sys.ball.y_mm))
    {
        g_sys.ref.theta_x_ref_deg *= 0.5f;
        g_sys.ref.theta_y_ref_deg *= 0.5f;
    }
}

/**
 * @brief 5ms 控制任务
 *
 * @note 这是内环控制函数，主要负责：
 *       1. 做 IMU / 球数据超时保护
 *       2. 根据平台目标倾角和 IMU 姿态运行内环
 *       3. 计算舵机命令角
 *       4. 输出到舵机
 */
void ControllerMgr_Run5ms(void)
{
    PlateInnerOutput_t inner; // 内环输出：舵机命令角

    /* -------------------- IMU 超时检查 -------------------- */
    if (SafetyMgr_CheckImuTimeout(g_sys_ms, g_sys.imu.update_ms))
    {
        Servo_Center(); // IMU 失效时直接让平台回中
        return;
    }

    /* -------------------- 球数据超时检查 -------------------- */
    if (SafetyMgr_CheckPiTimeout(g_sys_ms, g_sys.ball.tick_ms) && g_sys.ball.valid)
    {
        g_sys.ball.valid = 0U;   // 把球状态标记为无效
        BallOuterLoop_Reset();   // 重置外环状态

#if BALL_LOST_SAFE_CENTER
        /* 如果配置了丢球安全回中，则平台目标倾角清零 */
        g_sys.ref.theta_x_ref_deg = 0.0f;
        g_sys.ref.theta_y_ref_deg = 0.0f;
#endif
    }

    /* -------------------- 运行内环 -------------------- */
    /* 【修改】
       你当前实测关系是：
       - 左边上 -> pitch 增加
       - 下边上 -> roll 增加

       所以平台坐标与 IMU 坐标系相差 90°：
       - X轴（左右）应使用 pitch / gyro_y
       - Y轴（上下）应使用 roll / gyro_x
     */
    PlateInnerLoop_Run(g_sys.ref.theta_x_ref_deg,
                       g_sys.ref.theta_y_ref_deg,
                       g_sys.imu.pitch_deg - g_sys.imu.pitch_zero,
                       g_sys.imu.roll_deg - g_sys.imu.roll_zero,
                       g_sys.imu.gyro_y_dps,
                       g_sys.imu.gyro_x_dps,
                       &inner);

    /* 写回系统上下文 */
    g_sys.ref.servo_x_cmd_deg = inner.servo_x_cmd_deg;
    g_sys.ref.servo_y_cmd_deg = inner.servo_y_cmd_deg;

    /* 输出到舵机 */
    Servo_SetXYDeg(inner.servo_x_cmd_deg, inner.servo_y_cmd_deg);
}
