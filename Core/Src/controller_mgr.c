#include "controller_mgr.h"
#include "protocol_pi.h"
#include "protocol_screen.h"
#include "jy61p.h"
#include "task_mgr.h"
#include "ball_outer_loop.h"
#include "plate_inner_loop.h"
#include "servo.h"
#include "safety_mgr.h"

/* 全局系统上下文变量，存储系统状态和数据 */
SystemContext_t g_sys;

/* ======== 调试镜像变量：任务层 ======== */
volatile uint8_t  dbg_task_id = 0U;
volatile uint8_t  dbg_task_running = 0U;
volatile uint8_t  dbg_route0 = 0U;
volatile uint8_t  dbg_route1 = 0U;
volatile uint8_t  dbg_route2 = 0U;
volatile uint8_t  dbg_route3 = 0U;

/* ======== 调试镜像变量：控制层 ======== */
volatile float    dbg_ball_x_mm = 0.0f;
volatile float    dbg_ball_y_mm = 0.0f;
volatile uint8_t  dbg_ball_valid = 0U;
volatile uint32_t dbg_ball_tick_ms = 0U;

volatile float    dbg_target_x_mm = 0.0f;
volatile float    dbg_target_y_mm = 0.0f;
volatile uint8_t  dbg_ctrl_mode = 0U;

/* ======== 调试镜像变量：树莓派原始快照 ======== */
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

/* 外部声明的系统毫秒计时器 */
extern volatile uint32_t g_sys_ms;

void ControllerMgr_Init(void)
{
    memset(&g_sys, 0, sizeof(g_sys));
    g_sys.ref.target_x_mm = BOARD_CENTER_X_MM;
    g_sys.ref.target_y_mm = BOARD_CENTER_Y_MM;
    g_sys.ref.mode = CTRL_MODE_HOLD;
}

void ControllerMgr_UpdateInputs(void)
{
    PiRxData_t pi;
    JY61P_Data_t imu;
    ScreenRxData_t *screen = ProtocolScreen_GetData();

    uint8_t task_id = TASK_ID_NONE;
    uint8_t start_cmd = 0U;
    uint8_t stop_cmd = 0U;
    uint8_t route_a = 0U, route_b = 0U, route_c = 0U, route_d = 0U;
    float target_x = 0.0f;
    float target_y = 0.0f;
    uint8_t pi_route[4];
    uint8_t pi_route_len = 0U;
    TaskContext_t *ctx;

    ProtocolPi_CopyData(&pi);
    JY61P_CopyData(&imu);

    /* ======== 原始树莓派快照 ======== */
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

    /* ======== 连续状态写入系统 ======== */
    g_sys.ball.x_mm = pi.ball_x_mm;
    g_sys.ball.y_mm = pi.ball_y_mm;
    g_sys.ball.vx_mmps = pi.ball_vx_mmps;
    g_sys.ball.vy_mmps = pi.ball_vy_mmps;
    g_sys.ball.valid = pi.ball_valid;
    g_sys.ball.tick_ms = pi.ball_update_ms;

    g_sys.imu = imu;

    /* ======== 树莓派命令：路线 ======== */
    if (ProtocolPi_ConsumeRoute(&route_a, &route_b, &route_c, &route_d))
    {
        pi_route_len = 0U;
        if (route_a >= 1U && route_a <= 9U) pi_route[pi_route_len++] = route_a;
        if (route_b >= 1U && route_b <= 9U) pi_route[pi_route_len++] = route_b;
        if (route_c >= 1U && route_c <= 9U) pi_route[pi_route_len++] = route_c;
        if (route_d >= 1U && route_d <= 9U) pi_route[pi_route_len++] = route_d;

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

    /* ======== 串口屏命令：直接坐标 ======== */
    if (screen->point_valid)
    {
        TaskMgr_SetDirectPoint(screen->point_x_mm, screen->point_y_mm);
        TaskMgr_Start();
        screen->point_valid = 0U;
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

    /* ======== 串口屏命令：两点往返（默认2次循环） ======== */
    if (screen->roundtrip_valid)
    {
        TaskMgr_SetRoundTrip(screen->roundtrip_a, screen->roundtrip_b, 2U);
        TaskMgr_Start();
        screen->roundtrip_valid = 0U;
    }

    /* ======== 串口屏命令：简单任务（中心 / 激光） ======== */
    if (screen->task_id != TASK_ID_NONE)
    {
        if (screen->task_id == TASK_ID_GOTO_CENTER || screen->task_id == TASK_ID_TRACK_LASER)
        {
            TaskMgr_LoadTask(screen->task_id);
        }
        screen->task_id = TASK_ID_NONE;
    }

    if (screen->start_cmd)
    {
        TaskMgr_Start();
        screen->start_cmd = 0U;
    }

    if (screen->stop_cmd)
    {
        TaskMgr_Stop();
        screen->stop_cmd = 0U;
    }

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

void ControllerMgr_Run10ms(void)
{
    BallOuterOutput_t outer;

    TaskMgr_GetTarget(&g_sys.ref.target_x_mm, &g_sys.ref.target_y_mm, &g_sys.ref.mode);

    if (!TaskMgr_IsRunning())
    {
        g_sys.ref.target_x_mm = BOARD_CENTER_X_MM;
        g_sys.ref.target_y_mm = BOARD_CENTER_Y_MM;
        g_sys.ref.mode = CTRL_MODE_HOLD;
    }

    dbg_target_x_mm = g_sys.ref.target_x_mm;
    dbg_target_y_mm = g_sys.ref.target_y_mm;
    dbg_ctrl_mode = g_sys.ref.mode;

    if (!g_sys.ball.valid)
    {
        BallOuterLoop_Reset();
#if BALL_LOST_SAFE_CENTER
        g_sys.ref.theta_x_ref_deg = 0.0f;
        g_sys.ref.theta_y_ref_deg = 0.0f;
#endif
        return;
    }

    BallOuterLoop_Run(g_sys.ref.target_x_mm, g_sys.ref.target_y_mm,
                      g_sys.ball.x_mm, g_sys.ball.y_mm,
                      g_sys.ball.vx_mmps, g_sys.ball.vy_mmps,
                      g_sys.ref.mode, &outer);

    g_sys.ref.theta_x_ref_deg = outer.theta_x_ref_deg;
    g_sys.ref.theta_y_ref_deg = outer.theta_y_ref_deg;

    if (SafetyMgr_CheckBallNearEdge(g_sys.ball.x_mm, g_sys.ball.y_mm))
    {
        g_sys.ref.theta_x_ref_deg *= 0.5f;
        g_sys.ref.theta_y_ref_deg *= 0.5f;
    }
}

void ControllerMgr_Run5ms(void)
{
    PlateInnerOutput_t inner;

    if (SafetyMgr_CheckImuTimeout(g_sys_ms, g_sys.imu.update_ms))
    {
        Servo_Center();
        return;
    }

    if (SafetyMgr_CheckPiTimeout(g_sys_ms, g_sys.ball.tick_ms) && g_sys.ball.valid)
    {
        g_sys.ball.valid = 0U;
        BallOuterLoop_Reset();

#if BALL_LOST_SAFE_CENTER
        g_sys.ref.theta_x_ref_deg = 0.0f;
        g_sys.ref.theta_y_ref_deg = 0.0f;
#endif
    }

    PlateInnerLoop_Run(g_sys.ref.theta_x_ref_deg,
                       g_sys.ref.theta_y_ref_deg,
                       g_sys.imu.roll_deg - g_sys.imu.roll_zero,
                       g_sys.imu.pitch_deg - g_sys.imu.pitch_zero,
                       g_sys.imu.gyro_x_dps,
                       g_sys.imu.gyro_y_dps,
                       &inner);

    g_sys.ref.servo_x_cmd_deg = inner.servo_x_cmd_deg;
    g_sys.ref.servo_y_cmd_deg = inner.servo_y_cmd_deg;

    Servo_SetXYDeg(inner.servo_x_cmd_deg, inner.servo_y_cmd_deg);
}
