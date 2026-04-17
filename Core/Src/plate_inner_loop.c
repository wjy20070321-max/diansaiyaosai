#include "plate_inner_loop.h"   // 平台姿态内环控制模块头文件

/**
 * @brief 平台姿态内环控制器初始化函数
 *
 * @note 当前版本的内环控制器没有额外的状态变量，
 *       所以这里暂时不需要实际初始化内容。
 *
 *       以后如果你在内环中加入：
 *       - PID 控制器对象
 *       - 滤波器
 *       - 观测器
 *       - 状态变量
 *       那么通常就在这里做初始化。
 */
void PlateInnerLoop_Init(void)
{
    /* 当前实现为空，预留给后续扩展 */
}

/**
 * @brief 平台姿态内环控制器重置函数
 *
 * @note 当前版本内环没有需要清零的历史状态，
 *       所以这里暂时为空。
 *
 *       如果以后你加入：
 *       - 积分项
 *       - 历史误差
 *       - 历史滤波输出
 *       等状态量，
 *       就可以在这里统一清零。
 */
void PlateInnerLoop_Reset(void)
{
    /* 当前实现为空，预留给后续扩展 */
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
 * @note 这个函数实现的是一个简单的 PD 姿态控制器。
 *
 *       控制思想：
 *       1. 比较“目标平台倾角”和“实际平台倾角”，得到角度误差
 *       2. 再结合角速度做阻尼抑制
 *       3. 计算出两个舵机的控制命令
 *
 *       控制公式：
 *           u = kp * (ref - meas) + kd * (0 - gyro)
 *
 *       其中：
 *       - kp * (ref - meas)   是比例项，用于消除当前姿态误差
 *       - kd * (0 - gyro)     是微分项，用于抑制平台转动过快
 *
 *       最终输出会被限制在舵机允许的命令范围内。
 */
void PlateInnerLoop_Run(float theta_x_ref, float theta_y_ref,
                        float theta_x_meas, float theta_y_meas,
                        float gyro_x_meas, float gyro_y_meas,
                        PlateInnerOutput_t *out)
{
    /* -------------------- PD 参数 -------------------- */
    /* 这里直接把参数写成常量，表示当前版本内环用固定参数 */
    const float kp = 1.00f;   // 比例系数：决定角度误差的纠正强度
    const float kd = 0.08f;   // 微分系数：决定角速度阻尼强度

    /* 中间控制量 */
    float ux, uy;             // 分别表示 X / Y 两个方向的控制输出

    /* -------------------- IMU 方向适配：X 轴 -------------------- */
#if IMU_ROLL_REVERSE
    /* 如果硬件安装方向导致 IMU X 轴方向反了，
       就在这里把测量角度和角速度统一取反 */
    theta_x_meas = -theta_x_meas;
    gyro_x_meas = -gyro_x_meas;
#endif

    /* -------------------- IMU 方向适配：Y 轴 -------------------- */
#if IMU_PITCH_REVERSE
    /* 如果硬件安装方向导致 IMU Y 轴方向反了，
       就在这里把测量角度和角速度统一取反 */
    theta_y_meas = -theta_y_meas;
    gyro_y_meas = -gyro_y_meas;
#endif

    /* -------------------- 计算 X 轴控制输出 -------------------- */
    /* 比例项：目标角度 - 实际角度
       微分项：希望角速度尽量为 0，所以用 (0 - gyro) */
    ux = kp * (theta_x_ref - theta_x_meas) + kd * (0.0f - gyro_x_meas);

    /* -------------------- 计算 Y 轴控制输出 -------------------- */
    uy = kp * (theta_y_ref - theta_y_meas) + kd * (0.0f - gyro_y_meas);

    /* -------------------- 输出限幅 -------------------- */
    /* 舵机命令角必须限制在允许范围内，避免超出机械安全范围 */
    out->servo_x_cmd_deg = Limitf(ux, -SERVO_MAX_CMD_DEG, SERVO_MAX_CMD_DEG);
    out->servo_y_cmd_deg = Limitf(uy, -SERVO_MAX_CMD_DEG, SERVO_MAX_CMD_DEG);
}
