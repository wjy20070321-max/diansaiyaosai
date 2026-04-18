#include "plate_inner_loop.h"
#include "pid.h"

/* -------------------- 内环 PID 控制器对象 -------------------- */
/* 两个方向分别一个姿态 PID。
   D 项不用 pid.c 里的误差微分，而是直接用陀螺仪角速度做阻尼。 */
static PID_t g_pid_theta_x;
static PID_t g_pid_theta_y;

/* -------------------- 内环 PID 参数 -------------------- */
/* 这版先按“平滑优先”来：
   1. 先把 Ki 关掉，避免过零附近积分顶着走
   2. Kp 先保持中等
   3. Kd_rate 稍微加大一点，增强阻尼 */
#define INNER_X_KP          (30.20f)
#define INNER_X_KI          (0.000f)
#define INNER_X_KD_RATE     (0.040f)

#define INNER_Y_KP          (30.20f)
#define INNER_Y_KI          (0.000f)
#define INNER_Y_KD_RATE     (0.040f)

/* -------------------- 平滑参数 -------------------- */
/* 目标倾角每个 5ms 周期允许变化的最大值（度）
   用来把外部给进来的 theta_ref 做个小斜坡，避免一步跳太猛 */
#define INNER_REF_MAX_DELTA_PER_STEP    (0.12f)

/* 舵机输出每个 5ms 周期允许变化的最大值（度）
   这个是最关键的“过零平滑”参数 */
#define INNER_OUT_MAX_DELTA_PER_STEP    (0.35f)

/* -------------------- 模块内部状态 -------------------- */
static uint8_t g_inner_inited = 0U;

/* 目标倾角平滑后的内部状态 */
static float g_theta_x_ref_filt = 0.0f;
static float g_theta_y_ref_filt = 0.0f;

/* 上一拍输出，用于输出斜坡限制 */
static float g_last_ux = 0.0f;
static float g_last_uy = 0.0f;

/**
 * @brief 平台姿态内环控制器初始化函数
 *
 * @note 初始化两个方向的姿态 PID
 */
void PlateInnerLoop_Init(void)
{
    /* 输出限幅直接跟舵机命令限幅保持一致 */
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
 *
 * @note 清空两个方向的积分状态和内部平滑状态
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
 * @param theta_x_ref  X 轴目标倾角，单位：度
 * @param theta_y_ref  Y 轴目标倾角，单位：度
 * @param theta_x_meas X 轴实际测量倾角，单位：度
 * @param theta_y_meas Y 轴实际测量倾角，单位：度
 * @param gyro_x_meas  X 轴角速度测量值，单位：度/秒
 * @param gyro_y_meas  Y 轴角速度测量值，单位：度/秒
 * @param out          输出结构体指针，用于返回两个舵机命令角
 *
 * @note 控制思想：
 *       1. 先对目标倾角做小斜坡，避免目标本身突变
 *       2. 姿态环用 PI 计算舵机命令基值
 *       3. 再减去角速度阻尼项 D
 *       4. 再对最终输出做斜坡限制，避免过零突跳
 *       5. 最终按舵机安全范围限幅
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
