#include "controller_mgr.h"     // 控制器总管理模块头文件，包含 g_sys 定义和接口声明
#include "protocol_pi.h"        // 树莓派通信协议模块
#include "protocol_screen.h"    // 串口屏通信协议模块
#include "jy61p.h"              // JY61P IMU 姿态传感器模块
#include "task_mgr.h"           // 任务管理模块
#include "ball_outer_loop.h"    // 小球位置外环控制模块
#include "plate_inner_loop.h"   // 平台姿态内环控制模块
#include "servo.h"              // 舵机控制模块
#include "safety_mgr.h"         // 安全保护模块

/* -------------------- 全局系统上下文 -------------------- */
/* g_sys 是整个控制系统最核心的“总状态仓库”
   里面保存了：
   - 小球当前状态
   - IMU 当前状态
   - 当前目标点
   - 当前目标倾角
   - 当前舵机命令角
*/
SystemContext_t g_sys;

/* ======== 调试镜像变量：任务层 ======== */
/* 这些变量主要用于在线调试或看变量窗口。
   好处是你不用层层点进结构体，就能直接看关心的值。 */
volatile uint8_t  dbg_task_id = 0U;         // 当前任务 ID
volatile uint8_t  dbg_task_running = 0U;    // 当前任务是否在运行
volatile uint8_t  dbg_route0 = 0U;          // 当前路径第 1 个区域点
volatile uint8_t  dbg_route1 = 0U;          // 当前路径第 2 个区域点
volatile uint8_t  dbg_route2 = 0U;          // 当前路径第 3 个区域点
volatile uint8_t  dbg_route3 = 0U;          // 当前路径第 4 个区域点

/* ======== 调试镜像变量：控制层 ======== */
volatile float    dbg_ball_x_mm = 0.0f;     // 当前球 X 坐标
volatile float    dbg_ball_y_mm = 0.0f;     // 当前球 Y 坐标
volatile uint8_t  dbg_ball_valid = 0U;      // 当前球数据是否有效
volatile uint32_t dbg_ball_tick_ms = 0U;    // 当前球数据时间戳

volatile float    dbg_target_x_mm = 0.0f;   // 当前目标 X 坐标
volatile float    dbg_target_y_mm = 0.0f;   // 当前目标 Y 坐标
volatile uint8_t  dbg_ctrl_mode = 0U;       // 当前控制模式

/* ======== 调试镜像变量：树莓派原始快照 ======== */
/* 这组变量专门保存“树莓派刚发过来的原始数据快照”
   方便你区分：
   - 树莓派实际发了什么
   - 系统当前真正采用了什么 */
volatile float    dbg_pi_ball_x_mm = 0.0f;      // 树莓派发送的小球 X 坐标
volatile float    dbg_pi_ball_y_mm = 0.0f;      // 树莓派发送的小球 Y 坐标
volatile float    dbg_pi_ball_vx_mmps = 0.0f;   // 树莓派发送的小球 X 速度
volatile float    dbg_pi_ball_vy_mmps = 0.0f;   // 树莓派发送的小球 Y 速度
volatile uint8_t  dbg_pi_ball_valid = 0U;       // 树莓派发送的小球有效标志
volatile uint32_t dbg_pi_ball_tick_ms = 0U;     // 树莓派球数据更新时间戳

volatile float    dbg_pi_laser_x_mm = 0.0f;     // 树莓派发送的激光 X 坐标
volatile float    dbg_pi_laser_y_mm = 0.0f;     // 树莓派发送的激光 Y 坐标
volatile uint8_t  dbg_pi_laser_valid = 0U;      // 树莓派发送的激光有效标志
volatile uint32_t dbg_pi_laser_tick_ms = 0U;    // 树莓派激光数据更新时间戳

/* -------------------- 外部系统毫秒计数 -------------------- */
/* 这个变量在 main.c 中真正定义，在这里通过 extern 引用 */
extern volatile uint32_t g_sys_ms;

/**
 * @brief 控制器管理模块初始化
 * @note 作用：
 *       1. 清空整个系统上下文 g_sys
 *       2. 把默认目标设为板子中心
 *       3. 默认控制模式设为 HOLD（保持）
 */
void ControllerMgr_Init(void)
{
    /* 将整个系统上下文全部清零 */
    memset(&g_sys, 0, sizeof(g_sys));

    /* 上电默认目标先放在平台中心 */
    g_sys.ref.target_x_mm = BOARD_CENTER_X_MM;
    g_sys.ref.target_y_mm = BOARD_CENTER_Y_MM;

    /* 默认控制模式设为保持模式 */
    g_sys.ref.mode = CTRL_MODE_HOLD;
}

/**
 * @brief 更新系统输入数据
 * @note 这是总控制器里非常核心的一个函数。
 *       它负责把所有“外部输入”统一收进来，包括：
 *       - 树莓派视觉数据
 *       - IMU 姿态数据
 *       - 树莓派下发的任务/路径/目标点命令
 *       - 串口屏下发的任务/路径/目标点命令
 *
 *       最终会把这些输入同步到：
 *       - g_sys（系统上下文）
 *       - TaskMgr（任务管理器）
 */
void ControllerMgr_UpdateInputs(void)
{
    /* 本地快照变量：用于从各模块安全复制一份当前数据 */
    PiRxData_t pi;                // 树莓派数据快照
    JY61P_Data_t imu;             // IMU 数据快照
    ScreenRxData_t *screen = ProtocolScreen_GetData(); // 串口屏当前解析结果指针

    /* 临时命令变量：用于接收“原子消费接口”取出来的一次性命令 */
    uint8_t task_id = TASK_ID_NONE; // 当前取出的任务 ID
    uint8_t start_cmd = 0U;         // 当前取出的启动命令
    uint8_t stop_cmd = 0U;          // 当前取出的停止命令

    /* 路线命令临时变量：最多收 4 个区域点 */
    uint8_t route_a = 0U, route_b = 0U, route_c = 0U, route_d = 0U;

    /* 目标点命令临时变量 */
    float target_x = 0.0f;
    float target_y = 0.0f;

    /* 用于把树莓派给出的有效路径整理成数组 */
    uint8_t pi_route[4];
    uint8_t pi_route_len = 0U;

    /* 任务管理器上下文指针，用于调试镜像 */
    TaskContext_t *ctx;

    /* -------------------- 先复制最新外部状态 -------------------- */
    /* 从协议模块安全拷贝一份树莓派数据快照 */
    ProtocolPi_CopyData(&pi);

    /* 从 IMU 模块安全拷贝一份姿态数据快照 */
    JY61P_CopyData(&imu);

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
    /* 这里写入的是“当前连续状态”，不是一次性命令 */
    g_sys.ball.x_mm = pi.ball_x_mm;           // 小球 X 坐标
    g_sys.ball.y_mm = pi.ball_y_mm;           // 小球 Y 坐标
    g_sys.ball.vx_mmps = pi.ball_vx_mmps;     // 小球 X 速度
    g_sys.ball.vy_mmps = pi.ball_vy_mmps;     // 小球 Y 速度
    g_sys.ball.valid = pi.ball_valid;         // 小球数据是否有效
    g_sys.ball.tick_ms = pi.ball_update_ms;   // 球状态更新时间戳

    /* IMU 整体复制进系统上下文 */
    g_sys.imu = imu;

    /* ======== 树莓派命令：路线 ======== */
    /* 这里使用 ProtocolPi_ConsumeRoute()：
       一旦读取成功，协议模块内部会立刻清空这条命令，
       避免下次循环重复执行同一条路线命令 */
    if (ProtocolPi_ConsumeRoute(&route_a, &route_b, &route_c, &route_d))
    {
        /* 先清零路径长度 */
        pi_route_len = 0U;

        /* 只保留合法区域编号（1~9） */
        if (route_a >= 1U && route_a <= 9U) pi_route[pi_route_len++] = route_a;
        if (route_b >= 1U && route_b <= 9U) pi_route[pi_route_len++] = route_b;
        if (route_c >= 1U && route_c <= 9U) pi_route[pi_route_len++] = route_c;
        if (route_d >= 1U && route_d <= 9U) pi_route[pi_route_len++] = route_d;

        /* 如果至少有一个有效区域点，就设置路线任务并立即启动 */
        if (pi_route_len > 0U)
        {
            TaskMgr_SetRouteSequence(pi_route, pi_route_len, 0U); // 0U 表示逐点停留模式
            TaskMgr_Start();                                      // 启动任务
        }
    }

    /* ======== 树莓派命令：直接目标点 ======== */
    /* 如果树莓派下发了一个直接坐标点，则设置为直接点任务并立即启动 */
    if (ProtocolPi_ConsumeTarget(&target_x, &target_y))
    {
        TaskMgr_SetDirectPoint(target_x, target_y);
        TaskMgr_Start();
    }

    /* ======== 树莓派命令：任务控制 ======== */
    /* 这里处理的是任务 ID / 启动 / 停止命令 */
    if (ProtocolPi_ConsumeTaskCtrl(&task_id, &start_cmd, &stop_cmd))
    {
        /* 当前只允许树莓派直接加载两种简单任务：
           1. 去中心
           2. 激光追踪 */
        if (task_id == TASK_ID_GOTO_CENTER || task_id == TASK_ID_TRACK_LASER)
        {
            TaskMgr_LoadTask(task_id);
        }

        /* 如果收到启动命令，就启动当前任务 */
        if (start_cmd)
        {
            TaskMgr_Start();
        }

        /* 如果收到停止命令，就停止当前任务 */
        if (stop_cmd)
        {
            TaskMgr_Stop();
        }
    }

    /* ======== 串口屏命令：直接坐标 ======== */
    /* point_valid 表示串口屏解析到了一个有效坐标点命令 */
    if (screen->point_valid)
    {
        TaskMgr_SetDirectPoint(screen->point_x_mm, screen->point_y_mm);
        TaskMgr_Start();
        screen->point_valid = 0U; // 使用后清零，防止重复执行
    }

    /* ======== 串口屏命令：指定区域 ======== */
    if (screen->region_valid)
    {
        TaskMgr_SetDirectRegion(screen->region_id);
        TaskMgr_Start();
        screen->region_valid = 0U; // 清除命令有效标志
    }

    /* ======== 串口屏命令：区域序列 ======== */
    if (screen->route_valid)
    {
        TaskMgr_SetRouteSequence(screen->route, screen->route_len, screen->route_pass_mode);
        TaskMgr_Start();
        screen->route_valid = 0U; // 清除命令有效标志
    }

    /* ======== 串口屏命令：两点往返（默认 2 次循环） ======== */
    if (screen->roundtrip_valid)
    {
        TaskMgr_SetRoundTrip(screen->roundtrip_a, screen->roundtrip_b, 2U);
        TaskMgr_Start();
        screen->roundtrip_valid = 0U; // 清除命令有效标志
    }

    /* ======== 串口屏命令：简单任务（中心 / 激光） ======== */
    /* 屏幕直接指定 task_id 时，只允许装载两类简单任务 */
    if (screen->task_id != TASK_ID_NONE)
    {
        if (screen->task_id == TASK_ID_GOTO_CENTER || screen->task_id == TASK_ID_TRACK_LASER)
        {
            TaskMgr_LoadTask(screen->task_id);
        }
        screen->task_id = TASK_ID_NONE; // 使用后清空
    }

    /* 串口屏启动命令 */
    if (screen->start_cmd)
    {
        TaskMgr_Start();
        screen->start_cmd = 0U; // 清空启动命令
    }

    /* 串口屏停止命令 */
    if (screen->stop_cmd)
    {
        TaskMgr_Stop();
        screen->stop_cmd = 0U; // 清空停止命令
    }

    /* 串口屏 IMU 置零命令 */
    if (screen->imu_zero_cmd)
    {
        JY61P_SetZero();          // 把当前平台姿态作为零点
        screen->imu_zero_cmd = 0U;
    }

    /* ======== 调试镜像：任务层和控制层 ======== */
    /* 取出任务上下文，用于把关键状态镜像到调试变量 */
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
 * @note 这是外环执行函数，主要负责：
 *       1. 从任务管理器取当前目标点
 *       2. 根据当前球状态运行外环控制器
 *       3. 算出平台目标倾角
 *
 *       一般来说：
 *       - 外环负责“球应该往哪里滚”
 *       - 内环负责“平台应该怎么倾斜去实现这个目标”
 */
void ControllerMgr_Run10ms(void)
{
    BallOuterOutput_t outer; // 外环输出：平台目标倾角

    /* -------------------- 先从任务管理器取当前目标 -------------------- */
    TaskMgr_GetTarget(&g_sys.ref.target_x_mm, &g_sys.ref.target_y_mm, &g_sys.ref.mode);

    /* 如果当前没有任务在运行，则默认目标回到平台中心，并进入保持模式 */
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

    /* 如果当前球数据无效，则不能运行外环 */
    if (!g_sys.ball.valid)
    {
        /* 重置外环历史状态，避免旧目标导数影响后续重新进入控制 */
        BallOuterLoop_Reset();

#if BALL_LOST_SAFE_CENTER
        /* 如果配置了“球丢失时安全回中”，则把平台目标倾角直接清零 */
        g_sys.ref.theta_x_ref_deg = 0.0f;
        g_sys.ref.theta_y_ref_deg = 0.0f;
#endif
        return; // 直接退出，不进行外环计算
    }

    /* -------------------- 运行外环 -------------------- */
    /* 输入：
       - 当前目标点
       - 当前小球位置和速度
       - 当前控制模式
       输出：
       - 平台 X/Y 目标倾角 */
    BallOuterLoop_Run(g_sys.ref.target_x_mm, g_sys.ref.target_y_mm,
                      g_sys.ball.x_mm, g_sys.ball.y_mm,
                      g_sys.ball.vx_mmps, g_sys.ball.vy_mmps,
                      g_sys.ref.mode, &outer);

    /* 把外环输出写回系统上下文 */
    g_sys.ref.theta_x_ref_deg = outer.theta_x_ref_deg;
    g_sys.ref.theta_y_ref_deg = outer.theta_y_ref_deg;

    /* -------------------- 边缘安全处理 -------------------- */
    /* 如果小球靠近平台边缘，则将目标倾角减半，
       防止继续大幅倾斜导致小球更快冲向边缘 */
    if (SafetyMgr_CheckBallNearEdge(g_sys.ball.x_mm, g_sys.ball.y_mm))
    {
        g_sys.ref.theta_x_ref_deg *= 0.5f;
        g_sys.ref.theta_y_ref_deg *= 0.5f;
    }
}

/**
 * @brief 5ms 控制任务
 * @note 这是内环执行函数，主要负责：
 *       1. 做安全检查（IMU 是否超时、球数据是否超时）
 *       2. 根据平台目标倾角和 IMU 实际姿态运行内环
 *       3. 计算舵机命令角
 *       4. 输出到舵机
 */
void ControllerMgr_Run5ms(void)
{
    PlateInnerOutput_t inner; // 内环输出：舵机命令角

    /* -------------------- IMU 超时检查 -------------------- */
    /* 如果 IMU 太久没更新，就不能再可靠地做姿态闭环控制 */
    if (SafetyMgr_CheckImuTimeout(g_sys_ms, g_sys.imu.update_ms))
    {
        /* 直接让平台回中，防止失控 */
        Servo_Center();
        return;
    }

    /* -------------------- 树莓派球数据超时检查 -------------------- */
    /* 如果球数据超时，但系统还以为球有效，则要把球状态改成无效 */
    if (SafetyMgr_CheckPiTimeout(g_sys_ms, g_sys.ball.tick_ms) && g_sys.ball.valid)
    {
        g_sys.ball.valid = 0U; // 把球数据标记为无效

        /* 重置外环历史状态 */
        BallOuterLoop_Reset();

#if BALL_LOST_SAFE_CENTER
        /* 小球丢失时，把目标倾角清零，让平台回到安全姿态 */
        g_sys.ref.theta_x_ref_deg = 0.0f;
        g_sys.ref.theta_y_ref_deg = 0.0f;
#endif
    }

    /* -------------------- 运行内环 -------------------- */
    /* 输入：
       - 外环给出的平台目标倾角
       - IMU 当前实际姿态（减去零偏）
       - IMU 当前角速度
       输出：
       - 舵机 X/Y 命令角 */
    PlateInnerLoop_Run(g_sys.ref.theta_x_ref_deg,
                       g_sys.ref.theta_y_ref_deg,
                       g_sys.imu.roll_deg - g_sys.imu.roll_zero,
                       g_sys.imu.pitch_deg - g_sys.imu.pitch_zero,
                       g_sys.imu.gyro_x_dps,
                       g_sys.imu.gyro_y_dps,
                       &inner);

    /* 把内环输出写回系统上下文 */
    g_sys.ref.servo_x_cmd_deg = inner.servo_x_cmd_deg;
    g_sys.ref.servo_y_cmd_deg = inner.servo_y_cmd_deg;

    /* -------------------- 输出到舵机 -------------------- */
    /* 最终把两个轴的舵机角度命令发送到舵机驱动模块 */
    Servo_SetXYDeg(inner.servo_x_cmd_deg, inner.servo_y_cmd_deg);
}
