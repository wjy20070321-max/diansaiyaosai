#include "servo.h"
#include "tim.h"

/**
 * @brief 将微秒(us)转换为定时器计数器值(CCR)
 */
static uint32_t Servo_UsToCcr(float us)
{
    if (us < SERVO_PWM_MIN_US) us = SERVO_PWM_MIN_US;
    if (us > SERVO_PWM_MAX_US) us = SERVO_PWM_MAX_US;
    return (uint32_t)(us + 0.5f);
}

/**
 * @brief 将绝对舵机角度转换为脉宽
 * @param abs_deg 绝对舵机角度，按舵机物理总角度 270° 映射
 */
static float Servo_AbsDegToUs(float abs_deg)
{
    abs_deg = Limitf(abs_deg, 0.0f, SERVO_PHYS_TOTAL_DEG);

    return SERVO_PWM_MIN_US +
           (abs_deg / SERVO_PHYS_TOTAL_DEG) * (SERVO_PWM_MAX_US - SERVO_PWM_MIN_US);
}

/**
 * @brief 将“相对平台中心角”转换成“绝对舵机角”
 * @param rel_deg 相对平台有效中心的角度（可正可负）
 * @param center_offset_deg 安装补偿角
 * @param max_cmd_deg 当前轴允许的最大相对控制角
 *
 * @note 最终绝对角会被限制在机构有效工作区 [SERVO_WORK_MIN_DEG, SERVO_WORK_MAX_DEG]
 */
static float Servo_RelDegToAbsDeg(float rel_deg, float center_offset_deg, float max_cmd_deg)
{
    float abs_deg;

    rel_deg = Limitf(rel_deg, -max_cmd_deg, max_cmd_deg);

    /* 以机构有效中心角为中心，而不是舵机物理中点 135° */
    abs_deg = SERVO_WORK_CENTER_DEG + center_offset_deg + rel_deg;

    /* 关键：只允许落在 0~180°有效工作区内 */
    abs_deg = Limitf(abs_deg, SERVO_WORK_MIN_DEG, SERVO_WORK_MAX_DEG);

    return abs_deg;
}

/**
 * @brief 舵机初始化
 */
void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    Servo_Center();
}

/**
 * @brief 设置X轴舵机角度
 * @param deg 相对平台有效中心的目标角度
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
                Servo_RelDegToAbsDeg(deg, SERVO_X_CENTER_OFFSET_DEG, SERVO_X_MAX_CMD_DEG)
            )
        )
    );
}

/**
 * @brief 设置Y轴舵机角度
 * @param deg 相对平台有效中心的目标角度
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
                Servo_RelDegToAbsDeg(deg, SERVO_Y_CENTER_OFFSET_DEG, SERVO_Y_MAX_CMD_DEG)
            )
        )
    );
}

/**
 * @brief 同时设置X和Y轴舵机角度
 */
void Servo_SetXYDeg(float x_deg, float y_deg)
{
    Servo_SetXDeg(x_deg);
    Servo_SetYDeg(y_deg);
}

/**
 * @brief 舵机归中
 * @note 回到“机构有效工作区中心位”，即 90°附近
 */
void Servo_Center(void)
{
    Servo_SetXDeg(0.0f);
    Servo_SetYDeg(0.0f);
}
