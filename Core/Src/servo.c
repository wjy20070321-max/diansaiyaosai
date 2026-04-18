#include "servo.h"   // 舵机控制模块头文件
#include "tim.h"     // 定时器头文件，使用 PWM 输出时需要 htim2

/**
 * @brief 将脉宽值（微秒）转换为定时器比较寄存器 CCR 值
 * @param us 脉宽值，单位：微秒
 * @retval 对应的定时器 CCR 计数值
 *
 * @note 当前这个工程里，定时器 PWM 的配置是按“1us 一个计数”来设计的，
 *       所以这里基本上可以把 us 直接近似看作 CCR 值。
 *
 *       同时这里会做一次脉宽限幅：
 *       - 小于 SERVO_PWM_MIN_US 时，强制拉到最小脉宽
 *       - 大于 SERVO_PWM_MAX_US 时，强制压到最大脉宽
 *
 *       最后加 0.5f 是为了做四舍五入。
 */
static uint32_t Servo_UsToCcr(float us)
{
    if (us < SERVO_PWM_MIN_US) us = SERVO_PWM_MIN_US;
    if (us > SERVO_PWM_MAX_US) us = SERVO_PWM_MAX_US;
    return (uint32_t)(us + 0.5f);
}

/**
 * @brief 将“绝对舵机角度”转换为 PWM 脉宽
 * @param abs_deg 舵机绝对角度，单位：度
 * @retval 对应 PWM 脉宽，单位：微秒
 *
 * @note 这里的绝对角度是按“舵机物理总角度”来定义的，
 *       当前配置里舵机总行程是：
 *           SERVO_PHYS_TOTAL_DEG = 270°
 *
 *       映射关系为线性映射：
 *           0°      -> SERVO_PWM_MIN_US
 *           270°    -> SERVO_PWM_MAX_US
 *
 *       在进入映射前，会先把角度限制在 [0, 270] 之间。
 */
static float Servo_AbsDegToUs(float abs_deg)
{
    abs_deg = Limitf(abs_deg, 0.0f, SERVO_PHYS_TOTAL_DEG);

    return SERVO_PWM_MIN_US +
           (abs_deg / SERVO_PHYS_TOTAL_DEG) * (SERVO_PWM_MAX_US - SERVO_PWM_MIN_US);
}

/**
 * @brief 将“相对平台中心角”转换为“舵机绝对角度（每轴独立版）”
 * @param rel_deg           相对平台中心的目标角度，单位：度，可正可负
 * @param work_center_deg   当前轴的工作中心角，单位：度
 * @param center_offset_deg 当前轴安装补偿角，单位：度
 * @param neg_max_cmd_deg   当前轴负向最大相对命令角，单位：度
 * @param pos_max_cmd_deg   当前轴正向最大相对命令角，单位：度
 * @param work_min_deg      当前轴有效工作区最小角，单位：度
 * @param work_max_deg      当前轴有效工作区最大角，单位：度
 * @retval 转换后的绝对舵机角度，单位：度
 *
 * @note 这是这次修正的核心：
 *       X / Y 轴不再共用同一套中心位和工作区，
 *       而是每个轴独立建模。
 */
static float Servo_RelDegToAbsDegAxis(float rel_deg,
                                      float work_center_deg,
                                      float center_offset_deg,
                                      float neg_max_cmd_deg,
                                      float pos_max_cmd_deg,
                                      float work_min_deg,
                                      float work_max_deg)
{
    float abs_deg;

    /* 非对称限幅：按真实机械边界分别限制正向和负向 */
    rel_deg = Limitf(rel_deg, -neg_max_cmd_deg, pos_max_cmd_deg);

    /* 绝对角 = 当前轴工作中心角 + 当前轴安装补偿 + 相对偏转角 */
    abs_deg = work_center_deg + center_offset_deg + rel_deg;

    /* 最终只允许落在当前轴有效工作区内 */
    abs_deg = Limitf(abs_deg, work_min_deg, work_max_deg);

    return abs_deg;
}

/**
 * @brief 舵机模块初始化函数
 *
 * @note 该函数主要完成两件事：
 *       1. 启动 TIM2 的 PWM 通道 1、通道 2
 *       2. 让两个舵机先回到中心位置
 *
 *       当前默认映射关系是：
 *       - TIM2_CH1 -> X 轴舵机
 *       - TIM2_CH2 -> Y 轴舵机
 */
void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 启动 X 轴舵机 PWM 输出
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // 启动 Y 轴舵机 PWM 输出
    Servo_Center();                           // 上电后先回中，保证平台安全
}

/**
 * @brief 设置 X 轴舵机角度
 * @param deg 相对平台中心的目标角度，单位：度
 */
void Servo_SetXDeg(float deg)
{
#if SERVO_X_REVERSE
    deg = -deg;
#endif

    __HAL_TIM_SET_COMPARE(
        &htim2,
        TIM_CHANNEL_1,
        Servo_UsToCcr(
            Servo_AbsDegToUs(
                Servo_RelDegToAbsDegAxis(
                    deg,
                    SERVO_X_WORK_CENTER_DEG,
                    SERVO_X_CENTER_OFFSET_DEG,
                    SERVO_X_NEG_MAX_CMD_DEG,
                    SERVO_X_POS_MAX_CMD_DEG,
                    SERVO_X_WORK_MIN_DEG,
                    SERVO_X_WORK_MAX_DEG
                )
            )
        )
    );
}

/**
 * @brief 设置 Y 轴舵机角度
 * @param deg 相对平台中心的目标角度，单位：度
 */
void Servo_SetYDeg(float deg)
{
#if SERVO_Y_REVERSE
    deg = -deg;
#endif

    __HAL_TIM_SET_COMPARE(
        &htim2,
        TIM_CHANNEL_2,
        Servo_UsToCcr(
            Servo_AbsDegToUs(
                Servo_RelDegToAbsDegAxis(
                    deg,
                    SERVO_Y_WORK_CENTER_DEG,
                    SERVO_Y_CENTER_OFFSET_DEG,
                    SERVO_Y_NEG_MAX_CMD_DEG,
                    SERVO_Y_POS_MAX_CMD_DEG,
                    SERVO_Y_WORK_MIN_DEG,
                    SERVO_Y_WORK_MAX_DEG
                )
            )
        )
    );
}

/**
 * @brief 同时设置 X / Y 两个舵机的目标角度
 * @param x_deg X 轴相对平台中心的目标角度，单位：度
 * @param y_deg Y 轴相对平台中心的目标角度，单位：度
 */
void Servo_SetXYDeg(float x_deg, float y_deg)
{
    Servo_SetXDeg(x_deg); // 设置 X 轴舵机
    Servo_SetYDeg(y_deg); // 设置 Y 轴舵机
}

/**
 * @brief 舵机回中函数
 *
 * @note 这里的“回中”不是直接写固定脉宽，
 *       而是把两个轴都设为“相对平台中心角 = 0°”。
 */
void Servo_Center(void)
{
    Servo_SetXDeg(0.0f); // X 轴回到平台中心位
    Servo_SetYDeg(0.0f); // Y 轴回到平台中心位
}
