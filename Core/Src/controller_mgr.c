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

/* 外部声明的系统毫秒计时器 */
extern volatile uint32_t g_sys_ms;

/**
 * @brief 控制器管理模块初始化函数
 */
void ControllerMgr_Init(void)
{
    memset(&g_sys, 0, sizeof(g_sys));
    g_sys.ref.target_x_mm = BOARD_CENTER_X_MM;
    g_sys.ref.target_y_mm = BOARD_CENTER_Y_MM;
    g_sys.ref.mode = CTRL_MODE_HOLD;
}

/**
 * @brief 更新系统输入数据
 * @details 连续状态用 CopyData，一次性命令用原子 Consume
 */
void ControllerMgr_UpdateInputs(void)
{
    PiRxData_t pi;
    JY61P_Data_t imu;
    ScreenRxData_t *screen = ProtocolScreen_GetData();

    uint8_t task_id = TASK_ID_NONE;
    uint8_t start_cmd = 0U;
    uint8_t stop_cmd = 0U;
    uint8_t route_a = 0U, route_b = 0U, route_c = 0U, route_d = 0U;
    float target_x_dummy = 0.0f;
    float target_y_dummy = 0.0f;

    /* 连续状态：安全快照 */
    ProtocolPi_CopyData(&pi);
    JY61P_CopyData(&imu);

    /* 更新小球位置和速度数据 */
    g_sys.ball.x_mm = pi.ball_x_mm;
    g_sys.ball.y_mm = pi.ball_y_mm;
    g_sys.ball.vx_mmps = pi.ball_vx_mmps;
    g_sys.ball.vy_mmps = pi.ball_vy_mmps;
    g_sys.ball.valid = pi.ball_valid;
    g_sys.ball.tick_ms = pi.ball_update_ms;   /* 只看球状态帧时间戳 */

    /* 更新IMU数据 */
    g_sys.imu = imu;

    /* 一次性命令：原子取出并清空，避免比赛时出现丢命令/重复命令 */
    if (ProtocolPi_ConsumeRoute(&route_a, &route_b, &route_c, &route_d))
    {
        TaskMgr_SetUserRoute(route_a, route_b, route_c, route_d);
    }

    if (ProtocolPi_ConsumeTaskCtrl(&task_id, &start_cmd, &stop_cmd))
    {
        if (task_id != TASK_ID_NONE)
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

    /* direct target 目前还没接入任务层，先消费掉，避免一直悬挂 */
    (void)ProtocolPi_ConsumeTarget(&target_x_dummy, &target_y_dummy);

    /* 处理屏幕发送的路线数据 */
    if (screen->route_valid)
    {
        TaskMgr_SetUserRoute(screen->route_a, screen->route_b, screen->route_c, screen->route_d);
        screen->route_valid = 0U;
    }

    /* 处理屏幕发送的任务ID */
    if (screen->task_id != TASK_ID_NONE)
    {
        TaskMgr_LoadTask(screen->task_id);
        screen->task_id = TASK_ID_NONE;
    }

    /* 处理屏幕启动/停止命令 */
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

    /* 处理IMU校零命令 */
    if (screen->imu_zero_cmd)
    {
        JY61P_SetZero();
        screen->imu_zero_cmd = 0U;
    }
}

/**
 * @brief 10ms控制周期执行函数
 */
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

/**
 * @brief 5ms控制周期执行函数
 */
void ControllerMgr_Run5ms(void)
{
    PlateInnerOutput_t inner;

    if (SafetyMgr_CheckImuTimeout(g_sys_ms, g_sys.imu.update_ms))
    {
        Servo_Center();
        return;
    }

    /* 只依据 ball_update_ms 判断球数据是否过期 */
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
