#include "plate_inner_loop.h"  // 平板内环控制模块头文件

/**
 * @brief 平板内环控制器初始化函数
 * @details 初始化内环控制器参数和状态
 * 
 * @note 当前实现为空，可根据需要添加初始化代码
 */
void PlateInnerLoop_Init(void)
{
    // TODO: 添加初始化代码
}

/**
 * @brief 平板内环控制器重置函数
 * @details 重置内环控制器状态
 * 
 * @note 当前实现为空，可根据需要添加重置代码
 */
void PlateInnerLoop_Reset(void)
{
    // TODO: 添加重置代码
}

/**
 * @brief 平板内环控制核心函数
 * @details 实现PD控制算法，计算伺服控制命令
 * 
 * @param theta_x_ref X轴参考角度(度)
 * @param theta_y_ref Y轴参考角度(度)
 * @param theta_x_meas X轴测量角度(度)
 * @param theta_y_meas Y轴测量角度(度)
 * @param gyro_x_meas X轴陀螺仪测量值(度/秒)
 * @param gyro_y_meas Y轴陀螺仪测量值(度/秒)
 * @param out 指向输出结构体的指针
 * 
 * @note 控制算法：
 *       1. 使用PD控制：u = kp * (ref - meas) + kd * (0 - gyro)
 *       2. kp = 1.00, kd = 0.08
 *       3. 输出限制在[-SERVO_MAX_CMD_DEG, SERVO_MAX_CMD_DEG]范围内
 *       4. 支持IMU信号反转配置
 */
void PlateInnerLoop_Run(float theta_x_ref, float theta_y_ref,
                        float theta_x_meas, float theta_y_meas,
                        float gyro_x_meas, float gyro_y_meas,
                        PlateInnerOutput_t *out)
{
    const float kp = 1.00f;  // 比例系数
    const float kd = 0.08f;  // 微分系数
    float ux, uy;             // 控制输出值

    // 处理X轴IMU信号反转
#if IMU_ROLL_REVERSE
    theta_x_meas = -theta_x_meas;  // 反转测量角度
    gyro_x_meas = -gyro_x_meas;    // 反转陀螺仪值
#endif

    // 处理Y轴IMU信号反转
#if IMU_PITCH_REVERSE
    theta_y_meas = -theta_y_meas;  // 反转测量角度
    gyro_y_meas = -gyro_y_meas;      // 反转陀螺仪值
#endif

    // 计算X轴控制输出
    ux = kp * (theta_x_ref - theta_x_meas) + kd * (0.0f - gyro_x_meas);
    // 计算Y轴控制输出
    uy = kp * (theta_y_ref - theta_y_meas) + kd * (0.0f - gyro_y_meas);

    // 限制X轴输出在伺服范围
    out->servo_x_cmd_deg = Limitf(ux, -SERVO_MAX_CMD_DEG, SERVO_MAX_CMD_DEG);
    // 限制Y轴输出在伺服范围
    out->servo_y_cmd_deg = Limitf(uy, -SERVO_MAX_CMD_DEG, SERVO_MAX_CMD_DEG);
}
