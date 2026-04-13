#include "servo.h"  // 舵机控制主头文件
#include "tim.h"    // 定时器相关头文件

/**
 * @brief 将微秒(us)转换为定时器计数器值(CCR)
 * @param us 微秒值
 * @return 定时器计数器值
 * 
 * @note 限制范围在500-2500us之间，对应舵机标准PWM范围
 */
static uint32_t Servo_UsToCcr(float us)
{
    if (us < 500.0f) us = 500.0f;
    if (us > 2500.0f) us = 2500.0f;
    return (uint32_t)(us + 0.5f);
}

/**
 * @brief 将角度(deg)转换为微秒(us)
 * @param deg 角度值
 * @return 微秒值
 * 
 * @note 角度范围限制在-SERVO_MAX_CMD_DEG到SERVO_MAX_CMD_DEG之间
 * @note 使用线性映射将角度转换为PWM脉宽
 */
static float Servo_DegToUs(float deg)
{
    deg = Limitf(deg, -SERVO_MAX_CMD_DEG, SERVO_MAX_CMD_DEG);
    return SERVO_PWM_MID_US + (deg / SERVO_MAX_CMD_DEG) * (SERVO_PWM_MAX_US - SERVO_PWM_MID_US);
}

/**
 * @brief 舵机初始化
 * @note 启动TIM2定时器的PWM输出通道1和2
 * @note 将舵机归中
 */
void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    Servo_Center();
}

/**
 * @brief 设置X轴舵机角度
 * @param deg X轴角度
 * 
 * @note 如果定义了SERVO_X_REVERSE，则角度取反
 */
void Servo_SetXDeg(float deg)
{
#if SERVO_X_REVERSE
    deg = -deg;
#endif
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Servo_UsToCcr(Servo_DegToUs(deg)));
}

/**
 * @brief 设置Y轴舵机角度
 * @param deg Y轴角度
 * 
 * @note 如果定义了SERVO_Y_REVERSE，则角度取反
 */
void Servo_SetYDeg(float deg)
{
#if SERVO_Y_REVERSE
    deg = -deg;
#endif
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Servo_UsToCcr(Servo_DegToUs(deg)));
}

/**
 * @brief 同时设置X和Y轴舵机角度
 * @param x_deg X轴角度
 * @param y_deg Y轴角度
 */
void Servo_SetXYDeg(float x_deg, float y_deg)
{
    Servo_SetXDeg(x_deg);
    Servo_SetYDeg(y_deg);
}

/**
 * @brief 将舵机归中
 * @note 设置X和Y轴舵机到中间位置
 */
void Servo_Center(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Servo_UsToCcr(SERVO_PWM_MID_US));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Servo_UsToCcr(SERVO_PWM_MID_US));
}
