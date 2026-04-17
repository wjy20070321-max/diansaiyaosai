#include "plate_inner_loop.h"
#include "pid.h"

/* -------------------- 内环 PID 控制器对象 -------------------- */
/* 两个方向分别一个姿态 PID。
   同样，D 项不用 pid.c 里的误差微分，而是直接用陀螺仪角速度做阻尼。 */
static PID_t g_pid_theta_x;
static PID_t g_pid_theta_y;

/* -------------------- 内环 PID 参数 -------------------- */
/* 调参建议：
   1. 先调 kp，让平台能明显跟随目标倾角
   2. 再调 kd_rate，减少发冲和抖动
   3. 最后只加一点点 ki，修正轻微静差 */
#define INNER_X_KP          (1.20f)
#define INNER_X_KI          (0.020f)
#define INNER_X_KD_RATE     (0.020f)

#define INNER_Y_KP          (1.20f)
#define INNER_Y_KI          (0.020f)
#define INNER_Y_KD_RATE     (0.020f)

/* -------------------- 模块内部状态 -------------------- */
static uint8_t g_inner_inited = 0U;

/**
 * @brief 平台姿态内环控制器初始化函数
 *
 * @note 初始化两个方向的姿态 PID
 */
void PlateInnerLoop_Init(void)
{
    /* 这里只用 PID_t 的 P / I 两部分，D 项由角速度阻尼单独完成 */
    PID_Init(&g_pid_theta_x,
             INNER_X_KP, INNER_X_KI, 0.0f,
             -20.0f, 20.0f,
             -200.0f, 200.0f);

    PID_Init(&g_pid_theta_y,
             INNER_Y_KP, INNER_Y_KI, 0.0f,
             -20.0f, 20.0f,
             -200.0f, 200.0f);

    g_inner_inited = 1U;
}

/**
 * @brief 平台姿态内环控制器重置函数
 *
 * @note 清空两个方向的积分状态
 */
void PlateInnerLoop_Reset(void)
{
    PID_Reset(&g_pid_theta_x);
    PID_Reset(&g_pid_theta_y);
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
 *       1. 姿态环用 PI 计算舵机命令基值
 *       2. 再减去角速度阻尼项 D
 *       3. 最终按舵机安全范围限幅
 *
 *       这种“PI + rate D”比直接对误差做微分更稳，也更容易调。
 */
void PlateInnerLoop_Run(float theta_x_ref, float theta_y_ref,
                        float theta_x_meas, float theta_y_meas,
                        float gyro_x_meas, float gyro_y_meas,
                        PlateInnerOutput_t *out)
{
    float ux;   // X 方向舵机控制输出
    float uy;   // Y 方向舵机控制输出

    /* 输出指针保护 */
    if (out == NULL)
    {
        return;
    }

    /* 若尚未初始化，则补初始化一次 */
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

    /* -------------------- X 轴：姿态 PI + 角速度阻尼 D -------------------- */
    ux = PID_Run(&g_pid_theta_x, theta_x_ref, theta_x_meas, CONTROL_INNER_DT_S)
       - INNER_X_KD_RATE * gyro_x_meas;

    /* -------------------- Y 轴：姿态 PI + 角速度阻尼 D -------------------- */
    uy = PID_Run(&g_pid_theta_y, theta_y_ref, theta_y_meas, CONTROL_INNER_DT_S)
       - INNER_Y_KD_RATE * gyro_y_meas;

    /* -------------------- 输出限幅 -------------------- */
    out->servo_x_cmd_deg = Limitf(ux, -SERVO_X_NEG_MAX_CMD_DEG, SERVO_X_POS_MAX_CMD_DEG);
    out->servo_y_cmd_deg = Limitf(uy, -SERVO_Y_NEG_MAX_CMD_DEG, SERVO_Y_POS_MAX_CMD_DEG);
}
