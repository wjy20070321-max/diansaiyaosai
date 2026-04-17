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
    /* 防止绝对角度超出舵机物理允许范围 */
    abs_deg = Limitf(abs_deg, 0.0f, SERVO_PHYS_TOTAL_DEG);

    /* 线性映射：角度 -> 脉宽 */
    return SERVO_PWM_MIN_US +
           (abs_deg / SERVO_PHYS_TOTAL_DEG) * (SERVO_PWM_MAX_US - SERVO_PWM_MIN_US);
}

/**
 * @brief 将“相对平台中心角”转换为“舵机绝对角度”
 * @param rel_deg           相对平台中心的目标角度，单位：度，可正可负
 * @param center_offset_deg 舵机安装中心补偿角，单位：度
 * @param max_cmd_deg       当前轴允许的最大相对控制角，单位：度
 * @retval 转换后的绝对舵机角度，单位：度
 *
 * @note 这是这个舵机模块里很关键的一步：
 *
 *       你的控制器输出的不是“舵机绝对角度”，
 *       而是“相对平台中位的偏转量”。
 *
 *       例如：
 *       - 0°  表示平台回中
 *       - +5° 表示在平台中心基础上向一个方向偏 5°
 *       - -5° 表示向反方向偏 5°
 *
 *       但舵机真正需要的是“绝对物理角度”，例如：
 *       - 90°
 *       - 95°
 *       - 85°
 *
 *       所以这里要做 3 件事：
 *       1. 先把相对命令角 rel_deg 限制在允许控制范围内
 *       2. 再以机构中心角 SERVO_WORK_CENTER_DEG 为基准，换算成绝对角
 *       3. 最后把结果限制在机构有效工作区 [SERVO_WORK_MIN_DEG, SERVO_WORK_MAX_DEG]
 */
static float Servo_RelDegToAbsDeg(float rel_deg, float center_offset_deg, float max_cmd_deg)
{
    float abs_deg;

    /* 先对“相对控制角”做限幅，防止控制器给出的命令过大 */
    rel_deg = Limitf(rel_deg, -max_cmd_deg, max_cmd_deg);

    /* 关键步骤：
       绝对角 = 机构中心角 + 安装补偿角 + 相对偏转角
       注意这里以“机构有效中心角”作为中心，而不是舵机物理中点 135° */
    abs_deg = SERVO_WORK_CENTER_DEG + center_offset_deg + rel_deg;

    /* 最终只允许舵机落在机构有效工作区内 */
    abs_deg = Limitf(abs_deg, SERVO_WORK_MIN_DEG, SERVO_WORK_MAX_DEG);

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
 *
 *       如果以后你修改了定时器或引脚，
 *       这里以及 tim.c 里对应的 PWM 配置要同步改。
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
 *
 * @note 输入的 deg 不是“舵机物理绝对角度”，
 *       而是“相对平台中位”的控制角度。
 *
 *       本函数内部会依次完成：
 *       1. 根据配置决定是否反向
 *       2. 相对角 -> 绝对角
 *       3. 绝对角 -> 脉宽
 *       4. 脉宽 -> CCR
 *       5. 写入 TIM2_CH1 比较寄存器
 */
void Servo_SetXDeg(float deg)
{
#if SERVO_X_REVERSE
    /* 如果配置要求 X 轴方向反向，则把控制角取反 */
    deg = -deg;
#endif

    __HAL_TIM_SET_COMPARE(
        &htim2,                // 使用 TIM2
        TIM_CHANNEL_1,         // CH1 对应 X 轴舵机
        Servo_UsToCcr(         // 脉宽 -> CCR
            Servo_AbsDegToUs(  // 绝对角 -> 脉宽
                Servo_RelDegToAbsDeg(
                    deg,                       // 当前相对角度命令
                    SERVO_X_CENTER_OFFSET_DEG,// X 轴安装中心补偿
                    SERVO_X_MAX_CMD_DEG       // X 轴最大允许相对命令角
                )
            )
        )
    );
}

/**
 * @brief 设置 Y 轴舵机角度
 * @param deg 相对平台中心的目标角度，单位：度
 *
 * @note 与 X 轴类似，内部也会依次完成：
 *       1. 方向反转处理
 *       2. 相对角 -> 绝对角
 *       3. 绝对角 -> 脉宽
 *       4. 脉宽 -> CCR
 *       5. 写入 TIM2_CH2 比较寄存器
 */
void Servo_SetYDeg(float deg)
{
#if SERVO_Y_REVERSE
    /* 如果配置要求 Y 轴方向反向，则把控制角取反 */
    deg = -deg;
#endif

    __HAL_TIM_SET_COMPARE(
        &htim2,                // 使用 TIM2
        TIM_CHANNEL_2,         // CH2 对应 Y 轴舵机
        Servo_UsToCcr(
            Servo_AbsDegToUs(
                Servo_RelDegToAbsDeg(
                    deg,                       // 当前相对角度命令
                    SERVO_Y_CENTER_OFFSET_DEG,// Y 轴安装中心补偿
                    SERVO_Y_MAX_CMD_DEG       // Y 轴最大允许相对命令角
                )
            )
        )
    );
}

/**
 * @brief 同时设置 X / Y 两个舵机的目标角度
 * @param x_deg X 轴相对平台中心的目标角度，单位：度
 * @param y_deg Y 轴相对平台中心的目标角度，单位：度
 *
 * @note 该函数只是一个方便调用的封装，
 *       内部实际上还是分别调用：
 *       - Servo_SetXDeg()
 *       - Servo_SetYDeg()
 *
 *       常用于控制器每一拍同时输出两个轴的舵机命令。
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
 *
 *       这样做的好处是：
 *       - 会自动经过方向反转处理
 *       - 会自动经过安装中心补偿
 *       - 会自动经过绝对角/脉宽换算
 *
 *       最终会让舵机回到机构有效工作区中心附近，
 *       也就是当前配置中的 90° 附近。
 */
void Servo_Center(void)
{
    Servo_SetXDeg(0.0f); // X 轴回到平台中心位
    Servo_SetYDeg(0.0f); // Y 轴回到平台中心位
}
