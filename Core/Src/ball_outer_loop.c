/**
 ******************************************************************************
 * @file    ball_outer_loop.c
 * @brief   球位置外环控制模块源文件（LQR + 动态目标前馈 + BRAKE修复版）
 ******************************************************************************
 */

#include "ball_outer_loop.h"   // 外环模块头文件，包含输出结构体和接口声明
#include "app_config.h"        // 系统全局配置头文件，包含控制周期、角度限制等参数

/* -------------------- LQR 参数 -------------------- */
/* K[0] 对应位置误差反馈增益，K[1] 对应速度误差反馈增益
   这里 X/Y 两个方向分别给了一组相同参数 */
static const float LQR_K_X[2] = { 0.0015f, 0.0025f }; // X 方向 LQR 增益
static const float LQR_K_Y[2] = { 0.0015f, 0.0025f }; // Y 方向 LQR 增益

/* -------------------- 前馈增益常数 -------------------- */
/* 用于把“目标加速度”转成一个前馈控制量，帮助系统更快跟踪动态目标 */
#define FF_GAIN            (0.0001427f)

/* -------------------- 目标导数估计滤波参数 -------------------- */
/* 目标位置在变化时，需要对其求导得到目标速度、目标加速度。
   直接差分会很抖，所以这里用一阶低通做平滑。 */
#define TARGET_VEL_ALPHA   (0.25f)  // 目标速度低通滤波系数
#define TARGET_ACC_ALPHA   (0.15f)  // 目标加速度低通滤波系数

/* -------------------- 目标历史状态 -------------------- */
/* 这些静态变量用于保存上一拍的目标值，
   便于通过“差分”估计目标速度和目标加速度 */
static float last_target_x = 0.0f;   // 上一次目标 X 坐标
static float last_target_y = 0.0f;   // 上一次目标 Y 坐标
static float last_target_vx = 0.0f;  // 上一次目标 X 速度
static float last_target_vy = 0.0f;  // 上一次目标 Y 速度
static float last_target_ax = 0.0f;  // 上一次目标 X 加速度
static float last_target_ay = 0.0f;  // 上一次目标 Y 加速度
static uint8_t target_hist_valid = 0U; // 历史目标值是否已初始化：1=已初始化

/**
 * @brief 浮点限幅函数
 * @param val  输入值
 * @param minv 最小允许值
 * @param maxv 最大允许值
 * @retval 限幅后的结果
 *
 * @note 如果 val 超出 [minv, maxv] 区间，就把它限制到边界上
 */
static float LQR_Limitf(float val, float minv, float maxv)
{
    if (val > maxv) return maxv;
    if (val < minv) return minv;
    return val;
}

/**
 * @brief 一阶低通滤波
 * @param prev  上一次滤波输出
 * @param in    当前输入
 * @param alpha 滤波系数
 * @retval 新的滤波输出
 *
 * @note 公式：
 *       y(k) = y(k-1) + alpha * (x(k) - y(k-1))
 *
 *       alpha 越大，响应越快，平滑效果越弱
 *       alpha 越小，响应越慢，平滑效果越强
 */
static float LPF1(float prev, float in, float alpha)
{
    return prev + alpha * (in - prev);
}

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
 * @brief 单轴 LQR 控制计算
 * @param ref_p          目标位置
 * @param ref_v          目标速度
 * @param ref_a          目标加速度
 * @param act_p          实际位置
 * @param act_v          实际速度
 * @param K              LQR 增益数组，K[0]=位置增益，K[1]=速度增益
 * @param tilt_limit_deg 当前轴允许的最大倾角，单位：度
 * @retval 该轴参考倾角，单位：度
 *
 * @note 控制思想：
 *       - 反馈项：根据位置误差和速度误差做 LQR 状态反馈
 *       - 前馈项：根据目标加速度给一个前馈量，提高动态目标跟踪性能
 *       - 最终把结果从弧度转成角度，再做限幅
 */
static float LQR_Axis_Calc(float ref_p, float ref_v, float ref_a,
                           float act_p, float act_v,
                           const float *K,
                           float tilt_limit_deg)
{
    float err_pos = act_p - ref_p;  // 位置误差：实际 - 目标
    float err_vel = act_v - ref_v;  // 速度误差：实际 - 目标
    float u_fb_rad;                 // 反馈控制量（弧度）
    float u_ff_rad;                 // 前馈控制量（弧度）
    float total_deg;                // 总输出（角度）

    /* LQR 反馈项：u = -(K1*位置误差 + K2*速度误差) */
    u_fb_rad = -(K[0] * err_pos + K[1] * err_vel);

    /* 前馈项：用目标加速度乘以一个比例系数 */
    u_ff_rad = ref_a * FF_GAIN;

    /* 弧度转角度：1 rad = 57.2957795 deg */
    total_deg = (u_fb_rad + u_ff_rad) * 57.2957795f;

    /* 最终限幅，保证平台倾角不超过安全范围 */
    return LQR_Limitf(total_deg, -tilt_limit_deg, tilt_limit_deg);
}

/**
 * @brief 外环控制器初始化
 * @note 初始化本质上就是清空历史状态
 */
void BallOuterLoop_Init(void)
{
    BallOuterLoop_Reset();
}

/**
 * @brief 外环控制器重置
 * @note 清空目标历史值，避免旧任务的历史导数影响新任务
 */
void BallOuterLoop_Reset(void)
{
    last_target_x = 0.0f;
    last_target_y = 0.0f;
    last_target_vx = 0.0f;
    last_target_vy = 0.0f;
    last_target_ax = 0.0f;
    last_target_ay = 0.0f;
    target_hist_valid = 0U;
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
 * @note 这个函数完成的主要事情：
 *       1. 根据目标位置变化估计目标速度和目标加速度
 *       2. 对目标速度、加速度做低通滤波
 *       3. 按控制模式确定角度限幅
 *       4. 用 LQR + 前馈 算出平台 X/Y 两个方向的目标倾角
 */
void BallOuterLoop_Run(float x_ref, float y_ref,
                       float x_meas, float y_meas,
                       float vx_meas, float vy_meas,
                       uint8_t mode,
                       BallOuterOutput_t *out)
{
    /* 原始差分得到的目标速度/加速度 */
    float target_vx_raw;
    float target_vy_raw;
    float target_ax_raw;
    float target_ay_raw;

    /* 滤波后的目标速度/加速度 */
    float target_vx;
    float target_vy;
    float target_ax;
    float target_ay;

    /* X/Y 方向允许的最大倾角 */
    float x_tilt_limit;
    float y_tilt_limit;

    /* 空指针保护，避免野指针访问 */
    if (out == NULL)
    {
        return;
    }

    /* 第一次运行时，把历史目标值对齐到当前目标，
       这样可以避免第一次差分时出现很大的导数尖峰 */
    if (!target_hist_valid)
    {
        last_target_x = x_ref;
        last_target_y = y_ref;
        last_target_vx = 0.0f;
        last_target_vy = 0.0f;
        last_target_ax = 0.0f;
        last_target_ay = 0.0f;
        target_hist_valid = 1U;
    }

    /* -------------------- 估计目标速度 -------------------- */
    /* 用当前目标位置减去上一次目标位置，再除以控制周期，得到差分速度 */
    target_vx_raw = (x_ref - last_target_x) / CONTROL_OUTER_DT_S;
    target_vy_raw = (y_ref - last_target_y) / CONTROL_OUTER_DT_S;

    /* 对速度做一阶低通，抑制差分抖动 */
    target_vx = LPF1(last_target_vx, target_vx_raw, TARGET_VEL_ALPHA);
    target_vy = LPF1(last_target_vy, target_vy_raw, TARGET_VEL_ALPHA);

    /* -------------------- 估计目标加速度 -------------------- */
    /* 用当前目标速度减去上一次目标速度，再除以控制周期，得到差分加速度 */
    target_ax_raw = (target_vx - last_target_vx) / CONTROL_OUTER_DT_S;
    target_ay_raw = (target_vy - last_target_vy) / CONTROL_OUTER_DT_S;

    /* 对加速度做一阶低通，且使用上一次加速度输出做“有记忆滤波” */
    target_ax = LPF1(last_target_ax, target_ax_raw, TARGET_ACC_ALPHA);
    target_ay = LPF1(last_target_ay, target_ay_raw, TARGET_ACC_ALPHA);

    /* -------------------- BRAKE 模式特殊处理 -------------------- */
    /* 刹车模式的思想是：
       不再去追随“移动的目标趋势”，而是只利用当前小球速度做阻尼减速。
       所以把目标速度和目标加速度清零。 */
    if (mode == CTRL_MODE_BRAKE)
    {
        target_vx = 0.0f;
        target_vy = 0.0f;
        target_ax = 0.0f;
        target_ay = 0.0f;
    }

    /* -------------------- 根据模式决定最大倾角 -------------------- */
    /* 注意这里做的是交叉映射：
       - 球 X 方向运动，主要靠平台绕 Y 轴倾斜来控制
       - 球 Y 方向运动，主要靠平台绕 X 轴倾斜来控制 */
    y_tilt_limit = GetTiltLimitDeg(mode, x_meas - x_ref); // 控制球 X 方向时，对应 Y 轴倾角限制
    x_tilt_limit = GetTiltLimitDeg(mode, y_meas - y_ref); // 控制球 Y 方向时，对应 X 轴倾角限制

    /* -------------------- 计算平台目标倾角 -------------------- */
    /* 只有在 FAST / HOLD / BRAKE 三种有效模式下才输出控制角度 */
    if (mode == CTRL_MODE_FAST || mode == CTRL_MODE_HOLD || mode == CTRL_MODE_BRAKE)
    {
        /* 倾斜 Y 轴 -> 控制球 X 方向 */
        out->theta_y_ref_deg = LQR_Axis_Calc(x_ref, target_vx, target_ax,
                                             x_meas, vx_meas,
                                             LQR_K_X, y_tilt_limit);

        /* 倾斜 X 轴 -> 控制球 Y 方向 */
        out->theta_x_ref_deg = LQR_Axis_Calc(y_ref, target_vy, target_ay,
                                             y_meas, vy_meas,
                                             LQR_K_Y, x_tilt_limit);
    }
    else
    {
        /* 如果模式无效，则输出 0，保持平台不再倾斜 */
        out->theta_x_ref_deg = 0.0f;
        out->theta_y_ref_deg = 0.0f;
    }

    /* -------------------- 更新历史值 -------------------- */
    /* 为下一次控制周期保存本次目标位置、速度、加速度 */
    last_target_x = x_ref;
    last_target_y = y_ref;
    last_target_vx = target_vx;
    last_target_vy = target_vy;
    last_target_ax = target_ax;
    last_target_ay = target_ay;
}
