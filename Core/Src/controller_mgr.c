#include "controller_mgr.h"     // 控制器管理模块头文件
#include "protocol_pi.h"         // 与树莓派通信协议头文件
#include "protocol_screen.h"    // 与屏幕通信协议头文件
#include "jy61p.h"              // IMU传感器JY61P驱动头文件
#include "task_mgr.h"           // 任务管理模块头文件
#include "ball_outer_loop.h"    // 球外环控制头文件
#include "plate_inner_loop.h"   // 板内环控制头文件
#include "servo.h"              // 伺服电机控制头文件
#include "safety_mgr.h"         // 安全管理模块头文件

// 全局系统上下文变量，存储系统状态和数据
SystemContext_t g_sys;
// 外部声明的系统毫秒计时器
extern volatile uint32_t g_sys_ms;

/**
 * @brief 控制器管理模块初始化函数
 * @details 初始化系统上下文变量，设置默认目标位置和控制模式
 */
void ControllerMgr_Init(void)
{
    // 清零系统上下文结构体
    memset(&g_sys, 0, sizeof(g_sys));
    // 设置默认目标位置为板子中心
    g_sys.ref.target_x_mm = BOARD_CENTER_X_MM;
    g_sys.ref.target_y_mm = BOARD_CENTER_Y_MM;
    // 设置默认控制模式为保持模式
    g_sys.ref.mode = CTRL_MODE_HOLD;
}

/**
 * @brief 更新系统输入数据
 * @details 从各种通信接口获取最新数据并更新系统状态
 */
void ControllerMgr_UpdateInputs(void)
{
    // 获取树莓派通信协议数据
    PiRxData_t *pi = ProtocolPi_GetData();
    // 获取屏幕通信协议数据
    ScreenRxData_t *screen = ProtocolScreen_GetData();
    // 获取IMU传感器数据
    JY61P_Data_t *imu = JY61P_GetData();

    // 更新小球位置和速度数据
    g_sys.ball.x_mm = pi->ball_x_mm;
    g_sys.ball.y_mm = pi->ball_y_mm;
    g_sys.ball.vx_mmps = pi->ball_vx_mmps;
    g_sys.ball.vy_mmps = pi->ball_vy_mmps;
    g_sys.ball.valid = pi->ball_valid;
    g_sys.ball.tick_ms = pi->update_ms;

    // 更新IMU数据
    g_sys.imu = *imu;

    // 处理树莓派发送的路线数据
    if (pi->route_a >= 1U && pi->route_a <= 9U)
    {
        TaskMgr_SetUserRoute(pi->route_a, pi->route_b, pi->route_c, pi->route_d);
        pi->route_a = 0U;
    }
    // 处理屏幕发送的路线数据
    if (screen->route_valid)
    {
        TaskMgr_SetUserRoute(screen->route_a, screen->route_b, screen->route_c, screen->route_d);
        screen->route_valid = 0U;
    }

    // 处理树莓派发送的任务ID
    if (pi->task_id != TASK_ID_NONE)
    {
        TaskMgr_LoadTask(pi->task_id);
        pi->task_id = TASK_ID_NONE;
    }
    // 处理屏幕发送的任务ID
    if (screen->task_id != TASK_ID_NONE)
    {
        TaskMgr_LoadTask(screen->task_id);
        screen->task_id = TASK_ID_NONE;
    }

    // 处理启动命令
    if (pi->start_cmd || screen->start_cmd)
    {
        TaskMgr_Start();
        pi->start_cmd = 0U;
        screen->start_cmd = 0U;
    }
    // 处理停止命令
    if (pi->stop_cmd || screen->stop_cmd)
    {
        TaskMgr_Stop();
        pi->stop_cmd = 0U;
        screen->stop_cmd = 0U;
    }
    // 处理IMU校零命令
    if (screen->imu_zero_cmd)
    {
        JY61P_SetZero();
        screen->imu_zero_cmd = 0U;
    }
}

/**
 * @brief 10ms控制周期执行函数
 * @details 执行外环控制逻辑，计算板子倾斜角度参考值
 */
void ControllerMgr_Run10ms(void)
{
    BallOuterOutput_t outer;  // 外环控制输出结构体

    // 获取当前任务的目标位置和控制模式
    TaskMgr_GetTarget(&g_sys.ref.target_x_mm, &g_sys.ref.target_y_mm, &g_sys.ref.mode);

    // 如果任务未运行，设置默认目标位置和模式
    if (!TaskMgr_IsRunning())
    {
        g_sys.ref.target_x_mm = BOARD_CENTER_X_MM;
        g_sys.ref.target_y_mm = BOARD_CENTER_Y_MM;
        g_sys.ref.mode = CTRL_MODE_HOLD;
    }

    // 如果小球数据无效，重置外环控制并返回
    if (!g_sys.ball.valid)
    {
        BallOuterLoop_Reset();
#if BALL_LOST_SAFE_CENTER
        g_sys.ref.theta_x_ref_deg = 0.0f;
        g_sys.ref.theta_y_ref_deg = 0.0f;
#endif
        return;
    }

    // 运行外环控制算法
    BallOuterLoop_Run(g_sys.ref.target_x_mm, g_sys.ref.target_y_mm,
                      g_sys.ball.x_mm, g_sys.ball.y_mm,
                      g_sys.ball.vx_mmps, g_sys.ball.vy_mmps,
                      g_sys.ref.mode, &outer);

    // 保存外环控制输出
    g_sys.ref.theta_x_ref_deg = outer.theta_x_ref_deg;
    g_sys.ref.theta_y_ref_deg = outer.theta_y_ref_deg;

    // 如果小球靠近板子边缘，减小倾斜角度以防止小球掉落
    if (SafetyMgr_CheckBallNearEdge(g_sys.ball.x_mm, g_sys.ball.y_mm))
    {
        g_sys.ref.theta_x_ref_deg *= 0.5f;
        g_sys.ref.theta_y_ref_deg *= 0.5f;
    }
}

/**
 * @brief 5ms控制周期执行函数
 * @details 执行内环控制逻辑，计算伺服电机控制信号
 */
void ControllerMgr_Run5ms(void)
{
    PlateInnerOutput_t inner;  // 内环控制输出结构体

    // 检查IMU数据是否超时
    if (SafetyMgr_CheckImuTimeout(g_sys_ms, g_sys.imu.update_ms))
    {
        Servo_Center();  // 如果IMU超时，将伺服电机置中
        return;
    }

    // 检查树莓派数据是否超时
    if (SafetyMgr_CheckPiTimeout(g_sys_ms, g_sys.ball.tick_ms) && g_sys.ball.valid)
    {
        g_sys.ball.valid = 0U;  // 如果树莓派数据超时，标记小球数据无效
    }

    // 运行内环控制算法
    PlateInnerLoop_Run(g_sys.ref.theta_x_ref_deg,
                       g_sys.ref.theta_y_ref_deg,
                       g_sys.imu.roll_deg - g_sys.imu.roll_zero,  // 去除零偏的横滚角
                       g_sys.imu.pitch_deg - g_sys.imu.pitch_zero,  // 去除零偏的俯仰角
                       g_sys.imu.gyro_x_dps,  // X轴角速度
                       g_sys.imu.gyro_y_dps,  // Y轴角速度
                       &inner);

    // 保存内环控制输出
    g_sys.ref.servo_x_cmd_deg = inner.servo_x_cmd_deg;
    g_sys.ref.servo_y_cmd_deg = inner.servo_y_cmd_deg;
    // 设置伺服电机角度
    Servo_SetXYDeg(inner.servo_x_cmd_deg, inner.servo_y_cmd_deg);
}
