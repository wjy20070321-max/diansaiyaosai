/**
 ******************************************************************************
 * @file    ball_outer_loop.c
 * @brief   球位置外环控制模块源文件（LQR + 动态目标前馈 + BRAKE修复版）
 ******************************************************************************
 */

#include "ball_outer_loop.h"
#include "app_config.h"

/* ==========================================================
 * LQR 最优控制参数区
 * ========================================================== */
static const float LQR_K_X[2] = { 0.0015f, 0.0025f };
static const float LQR_K_Y[2] = { 0.0015f, 0.0025f };

/* 前馈增益常数 */
#define FF_GAIN   (0.0001427f)

/* 目标导数估计滤波参数 */
#define TARGET_VEL_ALPHA   (0.25f)
#define TARGET_ACC_ALPHA   (0.15f)

/* 历史状态 */
static float last_target_x = 0.0f;
static float last_target_y = 0.0f;
static float last_target_vx = 0.0f;
static float last_target_vy = 0.0f;
static uint8_t target_hist_valid = 0U;

/**
 * @brief 限幅
 */
static float LQR_Limitf(float val, float minv, float maxv)
{
    if (val > maxv) return maxv;
    if (val < minv) return minv;
    return val;
}

/**
 * @brief 一阶低通
 */
static float LPF1(float prev, float in, float alpha)
{
    return prev + alpha * (in - prev);
}

/**
 * @brief 根据模式和误差计算角度限幅
 */
static float GetTiltLimitDeg(uint8_t mode, float pos_err_mm)
{
    if (mode == CTRL_MODE_BRAKE)
    {
        return BOARD_BRAKE_TILT_REF_DEG;
    }

    if (mode == CTRL_MODE_HOLD)
    {
        if (ABSF(pos_err_mm) <= REGION_ENTER_RADIUS_MM)
        {
            return BOARD_HOLD_TILT_REF_DEG;
        }
    }

    return BOARD_MAX_TILT_REF_DEG;
}

/**
 * @brief 单轴 LQR 计算
 */
static float LQR_Axis_Calc(float ref_p, float ref_v, float ref_a,
                           float act_p, float act_v,
                           const float *K,
                           float tilt_limit_deg)
{
    float err_pos = act_p - ref_p;
    float err_vel = act_v - ref_v;

    float u_fb_rad = -(K[0] * err_pos + K[1] * err_vel);
    float u_ff_rad = ref_a * FF_GAIN;

    float total_deg = (u_fb_rad + u_ff_rad) * 57.2957795f;

    return LQR_Limitf(total_deg, -tilt_limit_deg, tilt_limit_deg);
}

/**
 * @brief 外环初始化
 */
void BallOuterLoop_Init(void)
{
    BallOuterLoop_Reset();
}

/**
 * @brief 外环重置
 */
void BallOuterLoop_Reset(void)
{
    last_target_x = 0.0f;
    last_target_y = 0.0f;
    last_target_vx = 0.0f;
    last_target_vy = 0.0f;
    target_hist_valid = 0U;
}

/**
 * @brief 外环主函数
 */
void BallOuterLoop_Run(float x_ref, float y_ref,
                       float x_meas, float y_meas,
                       float vx_meas, float vy_meas,
                       uint8_t mode,
                       BallOuterOutput_t *out)
{
    float target_vx_raw, target_vy_raw;
    float target_ax_raw, target_ay_raw;
    float target_vx, target_vy;
    float target_ax, target_ay;
    float x_tilt_limit, y_tilt_limit;

    if (out == NULL)
    {
        return;
    }

    /* 第一次进入时，先把历史值对齐，避免导数尖峰 */
    if (!target_hist_valid)
    {
        last_target_x = x_ref;
        last_target_y = y_ref;
        last_target_vx = 0.0f;
        last_target_vy = 0.0f;
        target_hist_valid = 1U;
    }

    /* 对动态目标求导 */
    target_vx_raw = (x_ref - last_target_x) / CONTROL_OUTER_DT_S;
    target_vy_raw = (y_ref - last_target_y) / CONTROL_OUTER_DT_S;

    target_vx = LPF1(last_target_vx, target_vx_raw, TARGET_VEL_ALPHA);
    target_vy = LPF1(last_target_vy, target_vy_raw, TARGET_VEL_ALPHA);

    target_ax_raw = (target_vx - last_target_vx) / CONTROL_OUTER_DT_S;
    target_ay_raw = (target_vy - last_target_vy) / CONTROL_OUTER_DT_S;

    /* 加速度也做低通，避免激光点抖动时前馈炸掉 */
    target_ax = LPF1(0.0f, target_ax_raw, TARGET_ACC_ALPHA);
    target_ay = LPF1(0.0f, target_ay_raw, TARGET_ACC_ALPHA);

    /* BRAKE 模式：目标速度/加速度置零，利用当前测得球速做阻尼刹车 */
    if (mode == CTRL_MODE_BRAKE)
    {
        target_vx = 0.0f;
        target_vy = 0.0f;
        target_ax = 0.0f;
        target_ay = 0.0f;
    }

    /* 根据模式和误差决定倾角上限 */
    y_tilt_limit = GetTiltLimitDeg(mode, x_meas - x_ref);
    x_tilt_limit = GetTiltLimitDeg(mode, y_meas - y_ref);

    /* 兼容交叉控制映射：
       倾斜Y轴 -> 控制球X方向
       倾斜X轴 -> 控制球Y方向 */
    if (mode == CTRL_MODE_FAST || mode == CTRL_MODE_HOLD || mode == CTRL_MODE_BRAKE)
    {
        out->theta_y_ref_deg = LQR_Axis_Calc(x_ref, target_vx, target_ax,
                                             x_meas, vx_meas,
                                             LQR_K_X, y_tilt_limit);

        out->theta_x_ref_deg = LQR_Axis_Calc(y_ref, target_vy, target_ay,
                                             y_meas, vy_meas,
                                             LQR_K_Y, x_tilt_limit);
    }
    else
    {
        out->theta_x_ref_deg = 0.0f;
        out->theta_y_ref_deg = 0.0f;
    }

    /* 更新历史值 */
    last_target_x = x_ref;
    last_target_y = y_ref;
    last_target_vx = target_vx;
    last_target_vy = target_vy;
}
