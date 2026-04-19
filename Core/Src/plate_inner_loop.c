#include "plate_inner_loop.h"
#include "pid.h"

/* -------------------- 内环 PID 控制器对象 -------------------- */
static PID_t g_pid_theta_x;
static PID_t g_pid_theta_y;

/* -------------------- 内环 PID 参数 -------------------- */
/* -------------------- 内环 PID 参数 -------------------- */

/**
 * @brief X 轴姿态环比例系数
 * @note  决定 X 轴对目标倾角误差的“反应力度”
 *
 * 调大：
 *  - 跟随更快
 *  - 但更容易生硬、抖动、顶行程
 *
 * 调小：
 *  - 更平滑
 *  - 但可能跟随偏慢、显得发软
 */
#define INNER_X_KP                  (3.8f)

/**
 * @brief X 轴姿态环积分系数
 * @note  用来消除小静差，但也最容易引起拖尾和反向不干脆
 *
 * 调大：
 *  - 更容易消掉“总差一点”的稳态误差
 *  - 但容易积累旧误差，导致减速慢、反向慢、拖尾
 *
 * 调小：
 *  - 更干脆、更稳
 *  - 但可能存在一点小静差
 *
 * 建议：
 *  - 调试阶段通常先设 0
 *  - 系统稳了以后再加一点点
 */
#define INNER_X_KI                  (0.000f)

/**
 * @brief X 轴角速度阻尼系数
 * @note  用陀螺仪角速度做 D 项，抑制发冲和抖动
 *
 * 调大：
 *  - 刹得更住，动作更稳
 *  - 但太大会显得发钝、发死
 *
 * 调小：
 *  - 更灵敏
 *  - 但更容易冲过头
 */
#define INNER_X_KD_RATE             (0.030f)

/**
 * @brief Y 轴姿态环比例系数
 * @note  含义和 X 轴一样，只是作用在 Y 轴
 */
#define INNER_Y_KP                  (3.8f)

/**
 * @brief Y 轴姿态环积分系数
 * @note  含义和 X 轴一样，只是作用在 Y 轴
 */
#define INNER_Y_KI                  (0.000f)

/**
 * @brief Y 轴角速度阻尼系数
 * @note  含义和 X 轴一样，只是作用在 Y 轴
 */
#define INNER_Y_KD_RATE             (0.030f)
/* -------------------- 平滑参数 -------------------- */
/* 目标倾角每个 5ms 周期允许变化的最大值（度） */
#define INNER_REF_MAX_DELTA_PER_STEP    (0.12f)

/* 舵机输出每个 5ms 周期允许变化的最大值（度） */
#define INNER_OUT_MAX_DELTA_PER_STEP    (0.35f)

/* -------------------- 模块内部状态 -------------------- */
static uint8_t g_inner_inited = 0U;

/* 平滑后的内部状态 */
static float g_theta_x_ref_filt = 0.0f;
static float g_theta_y_ref_filt = 0.0f;
static float g_last_ux = 0.0f;
static float g_last_uy = 0.0f;

/**
 * @brief 平台姿态内环控制器初始化函数
 */
void PlateInnerLoop_Init(void)
{
    PID_Init(&g_pid_theta_x,
             INNER_X_KP, INNER_X_KI, 0.0f,
             -SERVO_X_NEG_MAX_CMD_DEG, SERVO_X_POS_MAX_CMD_DEG,
             -200.0f, 200.0f);

    PID_Init(&g_pid_theta_y,
             INNER_Y_KP, INNER_Y_KI, 0.0f,
             -SERVO_Y_NEG_MAX_CMD_DEG, SERVO_Y_POS_MAX_CMD_DEG,
             -200.0f, 200.0f);

    g_theta_x_ref_filt = 0.0f;
    g_theta_y_ref_filt = 0.0f;
    g_last_ux = 0.0f;
    g_last_uy = 0.0f;

    g_inner_inited = 1U;
}

/**
 * @brief 平台姿态内环控制器重置函数
 */
void PlateInnerLoop_Reset(void)
{
    PID_Reset(&g_pid_theta_x);
    PID_Reset(&g_pid_theta_y);

    g_theta_x_ref_filt = 0.0f;
    g_theta_y_ref_filt = 0.0f;
    g_last_ux = 0.0f;
    g_last_uy = 0.0f;
}

/**
 * @brief 平台姿态内环主控制函数
 */
void PlateInnerLoop_Run(float theta_x_ref, float theta_y_ref,
                        float theta_x_meas, float theta_y_meas,
                        float gyro_x_meas, float gyro_y_meas,
                        PlateInnerOutput_t *out)
{
    float ux;
    float uy;

    if (out == NULL)
    {
        return;
    }

    if (!g_inner_inited)
    {
        PlateInnerLoop_Init();
    }

    /* -------------------- IMU 方向适配：X 轴 -------------------- */
#if IMU_ROLL_REVERSE
    theta_x_meas = -theta_x_meas;
    gyro_x_meas  = -gyro_x_meas;
#endif

    /* -------------------- IMU 方向适配：Y 轴 -------------------- */
#if IMU_PITCH_REVERSE
    theta_y_meas = -theta_y_meas;
    gyro_y_meas  = -gyro_y_meas;
#endif

    /* -------------------- 目标倾角斜坡限制 -------------------- */
    g_theta_x_ref_filt = Limitf(theta_x_ref,
                                g_theta_x_ref_filt - INNER_REF_MAX_DELTA_PER_STEP,
                                g_theta_x_ref_filt + INNER_REF_MAX_DELTA_PER_STEP);

    g_theta_y_ref_filt = Limitf(theta_y_ref,
                                g_theta_y_ref_filt - INNER_REF_MAX_DELTA_PER_STEP,
                                g_theta_y_ref_filt + INNER_REF_MAX_DELTA_PER_STEP);

    /* -------------------- X 轴：姿态 PI + 角速度阻尼 D -------------------- */
    ux = PID_Run(&g_pid_theta_x, g_theta_x_ref_filt, theta_x_meas, CONTROL_INNER_DT_S)
       - INNER_X_KD_RATE * gyro_x_meas;

    /* -------------------- Y 轴：姿态 PI + 角速度阻尼 D -------------------- */
    uy = PID_Run(&g_pid_theta_y, g_theta_y_ref_filt, theta_y_meas, CONTROL_INNER_DT_S)
       - INNER_Y_KD_RATE * gyro_y_meas;

    /* -------------------- 输出斜坡限制 -------------------- */
    ux = Limitf(ux,
                g_last_ux - INNER_OUT_MAX_DELTA_PER_STEP,
                g_last_ux + INNER_OUT_MAX_DELTA_PER_STEP);

    uy = Limitf(uy,
                g_last_uy - INNER_OUT_MAX_DELTA_PER_STEP,
                g_last_uy + INNER_OUT_MAX_DELTA_PER_STEP);

    g_last_ux = ux;
    g_last_uy = uy;

    /* -------------------- 输出限幅 -------------------- */
    out->servo_x_cmd_deg = Limitf(ux, -SERVO_X_NEG_MAX_CMD_DEG, SERVO_X_POS_MAX_CMD_DEG);
    out->servo_y_cmd_deg = Limitf(uy, -SERVO_Y_NEG_MAX_CMD_DEG, SERVO_Y_POS_MAX_CMD_DEG);
}
