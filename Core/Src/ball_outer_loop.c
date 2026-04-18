/** ****************************************************************************
 * @file    ball_outer_loop.c
 * @brief   小球位置外环控制模块源文件（双轴 PID 版，直连映射修正版）
 ******************************************************************************/

#include "ball_outer_loop.h"
#include "app_config.h"
#include "pid.h"

/* -------------------- 外环 PID 控制器对象 -------------------- */
/* 这里用 PID_t 来保存两个方向的位置环状态。
   注意：D 项不用 pid.c 里的误差微分，而是直接使用视觉速度做阻尼，
   这样更稳、更好调。 */
static PID_t g_pid_x_pos;   // 球 X 方向位置环（最终输出平台 X 轴目标倾角）
static PID_t g_pid_y_pos;   // 球 Y 方向位置环（最终输出平台 Y 轴目标倾角）

/* -------------------- 外环 PID 参数 -------------------- */
/* 调参建议：
   1. 先调 kp，让球“能明显朝目标动”
   2. 再调 kd_vel，抑制冲过头
   3. 最后只加很小 ki，消除残余静差 */
#define OUTER_X_KP        (25.20f)
#define OUTER_X_KI        (0.00010f)
#define OUTER_X_KD_VEL    (0.0030f)

#define OUTER_Y_KP        (25.20f)
#define OUTER_Y_KI        (0.00010f)
#define OUTER_Y_KD_VEL    (0.0030f)

/* -------------------- 模块内部状态 -------------------- */
static uint8_t g_outer_inited = 0U;   // 外环 PID 是否已初始化
static uint8_t g_last_mode = 0xFFU;   // 上一次控制模式，用于切模式时清积分

/**
 * @brief 根据控制模式和位置误差计算允许的最大倾角
 * @param mode       当前控制模式
 * @param pos_err_mm 当前轴位置误差，单位：毫米
 * @retval 允许的最大倾角，单位：度
 *
 * @note 逻辑说明：
 *       1. BRAKE（刹车）模式时，使用较小的制动倾角
 *       2. HOLD（保持）模式时，如果已经很接近目标，就只允许更小倾角，防止抖动
 *       3. 其他情况，允许用最大倾角快速运动
 */
static float GetTiltLimitDeg(uint8_t mode, float pos_err_mm)
{
    /* 刹车模式：限制在制动倾角内 */
    if (mode == CTRL_MODE_BRAKE)
    {
        return BOARD_BRAKE_TILT_REF_DEG;
    }

    /* 保持模式：如果已经靠近目标，则进一步减小倾角 */
    if (mode == CTRL_MODE_HOLD)
    {
        if (ABSF(pos_err_mm) <= REGION_ENTER_RADIUS_MM)
        {
            return BOARD_HOLD_TILT_REF_DEG;
        }
    }

    /* 默认使用最大允许倾角 */
    return BOARD_MAX_TILT_REF_DEG;
}

/**
 * @brief 外环控制器初始化
 * @note 初始化两个方向的位置 PID
 */
void BallOuterLoop_Init(void)
{
    /* 这里只用 PID_t 的 P / I 两部分，D 项由“速度阻尼”单独完成 */
    PID_Init(&g_pid_x_pos,
             OUTER_X_KP, OUTER_X_KI, 0.0f,
             -20.0f, 20.0f,
             -3000.0f, 3000.0f);

    PID_Init(&g_pid_y_pos,
             OUTER_Y_KP, OUTER_Y_KI, 0.0f,
             -20.0f, 20.0f,
             -3000.0f, 3000.0f);

    g_outer_inited = 1U;
    g_last_mode = 0xFFU;
}

/**
 * @brief 外环控制器重置
 * @note 清空两个方向的位置环积分状态
 */
void BallOuterLoop_Reset(void)
{
    PID_Reset(&g_pid_x_pos);
    PID_Reset(&g_pid_y_pos);
    g_last_mode = 0xFFU;
}

/**
 * @brief 外环主控制函数
 * @param x_ref    目标 X 坐标，单位：毫米
 * @param y_ref    目标 Y 坐标，单位：毫米
 * @param x_meas   当前小球实际 X 坐标，单位：毫米
 * @param y_meas   当前小球实际 Y 坐标，单位：毫米
 * @param vx_meas  当前小球实际 X 速度，单位：毫米/秒
 * @param vy_meas  当前小球实际 Y 速度，单位：毫米/秒
 * @param mode     当前控制模式（FAST / BRAKE / HOLD）
 * @param out      输出结构体指针，用于返回平台目标倾角
 *
 * @note 控制思想：
 *       1. 位置环用 PI 计算“该倾多少角”
 *       2. 再减去一个与当前球速度成比例的阻尼项 D
 *       3. 最后按当前模式进行限幅
 *
 *       【修正】
 *       按当前机械定义直接映射：
 *       - X轴（左右）位置误差 -> 平台 X 轴目标倾角
 *       - Y轴（上下）位置误差 -> 平台 Y 轴目标倾角
 */
void BallOuterLoop_Run(float x_ref, float y_ref,
                       float x_meas, float y_meas,
                       float vx_meas, float vy_meas,
                       uint8_t mode,
                       BallOuterOutput_t *out)
{
    float x_tilt_limit;     // 平台 X 轴最大倾角
    float y_tilt_limit;     // 平台 Y 轴最大倾角

    float theta_x_cmd_deg;  // 控制球 X 方向的倾角命令
    float theta_y_cmd_deg;  // 控制球 Y 方向的倾角命令

    /* 空指针保护 */
    if (out == NULL)
    {
        return;
    }

    /* 若尚未初始化，则补初始化一次 */
    if (!g_outer_inited)
    {
        BallOuterLoop_Init();
    }

    /* 切模式时，把积分清掉，防止旧状态影响新模式 */
    if (mode != g_last_mode)
    {
        PID_Reset(&g_pid_x_pos);
        PID_Reset(&g_pid_y_pos);
        g_last_mode = mode;
    }

    /* 根据当前模式决定最大倾角 */
    x_tilt_limit = GetTiltLimitDeg(mode, x_meas - x_ref);
    y_tilt_limit = GetTiltLimitDeg(mode, y_meas - y_ref);

    /* 只有在有效控制模式下才输出角度 */
    if (mode == CTRL_MODE_FAST || mode == CTRL_MODE_HOLD || mode == CTRL_MODE_BRAKE)
    {
        /* -------------------- 球 X 方向：位置 PI + 速度阻尼 D -------------------- */
        /* 位置环输出的是“平台 X 轴应该倾多少角”，再减去速度阻尼项 */
        theta_x_cmd_deg = PID_Run(&g_pid_x_pos, x_ref, x_meas, CONTROL_OUTER_DT_S)
                        - OUTER_X_KD_VEL * vx_meas;

        /* -------------------- 球 Y 方向：位置 PI + 速度阻尼 D -------------------- */
        /* 位置环输出的是“平台 Y 轴应该倾多少角”，再减去速度阻尼项 */
        theta_y_cmd_deg = PID_Run(&g_pid_y_pos, y_ref, y_meas, CONTROL_OUTER_DT_S)
                        - OUTER_Y_KD_VEL * vy_meas;

        /* 最终按当前模式允许的最大倾角限幅 */
        out->theta_x_ref_deg = Limitf(theta_x_cmd_deg, -x_tilt_limit, x_tilt_limit);
        out->theta_y_ref_deg = Limitf(theta_y_cmd_deg, -y_tilt_limit, y_tilt_limit);
    }
    else
    {
        out->theta_x_ref_deg = 0.0f;
        out->theta_y_ref_deg = 0.0f;
    }
}
