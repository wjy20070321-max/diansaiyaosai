/** ****************************************************************************
 * @file    ball_outer_loop.c
 * @brief   小球位置外环控制模块源文件（路径进度加减速版）
 *
 * @note
 * 这版外环不再只是“位置误差越大，倾角越大”，
 * 而是按“当前在整段路径上的进度”来做速度规划：
 *
 * 1. 起点附近：目标速度较小，缓缓起步
 * 2. 路径前半段：目标速度逐步升高
 * 3. 路径后半段：目标速度逐步降低
 * 4. 接近目标：目标速度趋近于 0
 *
 * 如果当前球速度已经比“此时应有的目标速度”更大，
 * 控制器就会自动给反向倾角，提前减速。
 ******************************************************************************/

#include "ball_outer_loop.h"
#include "app_config.h"
#include "pid.h"
#include <math.h>

/* -------------------- 外环 PID 控制器对象 -------------------- */
/* 这里只保留横向纠偏用 PID。
   沿路径主方向改成“进度-速度规划”控制，不再直接用位置 PID 硬推。 */
static PID_t g_pid_lateral;

/* -------------------- 路径段状态 -------------------- */
/* 记录当前这一段运动的起点和目标点。
   只要目标点变化，就把当前球位置记成新一段的起点。 */
static float g_seg_start_x_mm = 0.0f;
static float g_seg_start_y_mm = 0.0f;
static float g_seg_target_x_mm = 0.0f;
static float g_seg_target_y_mm = 0.0f;
static uint8_t g_seg_valid = 0U;

/* -------------------- 模块内部状态 -------------------- */
static uint8_t g_outer_inited = 0U;
static uint8_t g_last_mode = 0xFFU;

/* -------------------- 速度规划参数 -------------------- */
/* 这些参数是这版逻辑的关键。 */
#define PATH_RELOCK_TARGET_EPS_MM      (5.0f)   // 目标变化超过该值，认为进入新路径段

/* 不同模式下的路径峰值速度（毫米/秒） */
#define PATH_VPEAK_FAST_MMPS           (200.0f)
#define PATH_VPEAK_BRAKE_MMPS          (100.0f)
#define PATH_VPEAK_HOLD_MMPS            (50.0f)

/* 沿路径方向：速度误差 -> 倾角 的比例 */
#define PATH_K_TAN_FAST                (0.022f)
#define PATH_K_TAN_BRAKE               (0.032f)
#define PATH_K_TAN_HOLD                (0.040f)

/* 终点附近的位置“收尾拉回” */
#define PATH_K_END_POS                 (0.008f)

/* 横向偏离路径的纠偏参数 */
#define PATH_K_LAT_P                   (0.055f)
#define PATH_K_LAT_D                   (0.0040f)

/* -------------------- 工具函数 -------------------- */
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

static float GetPeakSpeedByMode(uint8_t mode)
{
    if (mode == CTRL_MODE_BRAKE) return PATH_VPEAK_BRAKE_MMPS;
    if (mode == CTRL_MODE_HOLD)  return PATH_VPEAK_HOLD_MMPS;
    return PATH_VPEAK_FAST_MMPS;
}

static float GetTanGainByMode(uint8_t mode)
{
    if (mode == CTRL_MODE_BRAKE) return PATH_K_TAN_BRAKE;
    if (mode == CTRL_MODE_HOLD)  return PATH_K_TAN_HOLD;
    return PATH_K_TAN_FAST;
}

/**
 * @brief 根据路径进度生成“目标线速度”
 * @param progress 当前路径进度，0~1
 * @param v_peak   该模式下的峰值线速度
 * @retval 当前进度对应的目标线速度（毫米/秒）
 *
 * @note
 * - 前半程：逐步加速
 * - 后半程：逐步减速
 * - 超过终点：目标速度给负值，帮助反向制动
 */
static float BuildDesiredLineSpeed(float progress, float v_peak)
{
    if (progress <= 0.0f)
    {
        return 0.25f * v_peak;  // 起步不要给 0，留一点“起步推力”
    }

    if (progress < 0.5f)
    {
        /* 0~0.5：从 0.25*v_peak 线性升到 1.0*v_peak */
        return v_peak * (0.25f + 1.50f * progress);
    }

    if (progress <= 1.0f)
    {
        /* 0.5~1.0：从 1.0*v_peak 线性降到 0 */
        return v_peak * (2.0f - 2.0f * progress);
    }

    /* 已经过终点：给一个反向目标速度，帮助刹车回拉 */
    return -0.30f * v_peak;
}

/**
 * @brief 外环控制器初始化
 */
void BallOuterLoop_Init(void)
{
    PID_Init(&g_pid_lateral,
             PATH_K_LAT_P, 0.0f, 0.0f,
             -10.0f, 10.0f,
             -1000.0f, 1000.0f);

    g_seg_start_x_mm = 0.0f;
    g_seg_start_y_mm = 0.0f;
    g_seg_target_x_mm = 0.0f;
    g_seg_target_y_mm = 0.0f;
    g_seg_valid = 0U;

    g_outer_inited = 1U;
    g_last_mode = 0xFFU;
}

/**
 * @brief 外环控制器重置
 */
void BallOuterLoop_Reset(void)
{
    PID_Reset(&g_pid_lateral);

    g_seg_start_x_mm = 0.0f;
    g_seg_start_y_mm = 0.0f;
    g_seg_target_x_mm = 0.0f;
    g_seg_target_y_mm = 0.0f;
    g_seg_valid = 0U;

    g_last_mode = 0xFFU;
}

/**
 * @brief 外环主控制函数
 */
void BallOuterLoop_Run(float x_ref, float y_ref,
                       float x_meas, float y_meas,
                       float vx_meas, float vy_meas,
                       uint8_t mode,
                       BallOuterOutput_t *out)
{
    float path_dx, path_dy;
    float path_len, inv_path_len;
    float tx, ty;             // 路径切向单位向量
    float nx, ny;             // 路径法向单位向量

    float cx, cy;             // 当前点相对起点坐标
    float s;                  // 当前在路径上的投影长度
    float progress;           // 当前路径进度 0~1

    float v_line;             // 当前沿路径方向的速度
    float v_lat;              // 当前横向速度
    float v_peak;             // 当前模式下的峰值目标速度
    float v_ref;              // 当前进度下的目标线速度

    float remain_mm;          // 剩余路程
    float tan_cmd_deg;        // 沿路径方向的倾角命令
    float lat_cmd_deg;        // 横向纠偏倾角命令

    float theta_x_cmd_deg;
    float theta_y_cmd_deg;

    float x_tilt_limit;
    float y_tilt_limit;

    if (out == NULL)
    {
        return;
    }

    if (!g_outer_inited)
    {
        BallOuterLoop_Init();
    }

    /* 模式变化时，横向纠偏状态清一下，避免旧状态影响 */
    if (mode != g_last_mode)
    {
        PID_Reset(&g_pid_lateral);
        g_last_mode = mode;
    }

    /* -------------------- 如果目标点变化，重新锁定一段新路径 -------------------- */
    if ((!g_seg_valid) ||
        (ABSF(x_ref - g_seg_target_x_mm) > PATH_RELOCK_TARGET_EPS_MM) ||
        (ABSF(y_ref - g_seg_target_y_mm) > PATH_RELOCK_TARGET_EPS_MM))
    {
        g_seg_start_x_mm = x_meas;
        g_seg_start_y_mm = y_meas;
        g_seg_target_x_mm = x_ref;
        g_seg_target_y_mm = y_ref;
        g_seg_valid = 1U;

        PID_Reset(&g_pid_lateral);
    }

    /* 当前路径向量 */
    path_dx = g_seg_target_x_mm - g_seg_start_x_mm;
    path_dy = g_seg_target_y_mm - g_seg_start_y_mm;
    path_len = sqrtf(path_dx * path_dx + path_dy * path_dy);

    /* 如果目标太近，直接清零 */
    if (path_len < 1.0f)
    {
        out->theta_x_ref_deg = 0.0f;
        out->theta_y_ref_deg = 0.0f;
        return;
    }

    inv_path_len = 1.0f / path_len;

    /* 路径切向单位向量 */
    tx = path_dx * inv_path_len;
    ty = path_dy * inv_path_len;

    /* 路径法向单位向量（左法向） */
    nx = -ty;
    ny = tx;

    /* 当前球位置相对起点的向量 */
    cx = x_meas - g_seg_start_x_mm;
    cy = y_meas - g_seg_start_y_mm;

    /* 当前在路径方向上的投影长度 */
    s = cx * tx + cy * ty;

    /* 进度：允许略微超过 1，便于过点后反向刹车 */
    progress = s / path_len;
    progress = Limitf(progress, -0.10f, 1.20f);

    /* 当前沿路径方向的速度 */
    v_line = vx_meas * tx + vy_meas * ty;

    /* 当前横向速度 */
    v_lat = vx_meas * nx + vy_meas * ny;

    /* 当前模式对应的峰值速度 */
    v_peak = GetPeakSpeedByMode(mode);

    /* 根据“路径进度”生成目标线速度 */
    v_ref = BuildDesiredLineSpeed(progress, v_peak);

    /* 剩余路程（允许为负，表示已经冲过目标） */
    remain_mm = path_len - s;

    /* -------------------- 沿路径方向控制 -------------------- */
    /* 核心逻辑：
       - 前半程：v_ref 逐渐增大 => 倾角朝目标方向，推进加速
       - 后半程：v_ref 逐渐减小 => 若当前速度仍大，就会自动给反向倾角减速
       - 接近终点：再叠加一点位置收尾项，帮助平稳停住 */
    tan_cmd_deg = GetTanGainByMode(mode) * (v_ref - v_line)
                + PATH_K_END_POS * remain_mm;

    /* -------------------- 横向纠偏控制 -------------------- */
    /* 横向偏离路径太多时，给一点法向纠偏倾角 */
    lat_cmd_deg = PID_RunErr(&g_pid_lateral, -(cx * nx + cy * ny), CONTROL_OUTER_DT_S)
                - PATH_K_LAT_D * v_lat;

    /* -------------------- 合成到 X / Y 两轴倾角 -------------------- */
    theta_x_cmd_deg = tan_cmd_deg * tx + lat_cmd_deg * nx;
    theta_y_cmd_deg = tan_cmd_deg * ty + lat_cmd_deg * ny;

    /* -------------------- 仍保留现有的模式限幅 -------------------- */
    x_tilt_limit = GetTiltLimitDeg(mode, x_meas - x_ref);
    y_tilt_limit = GetTiltLimitDeg(mode, y_meas - y_ref);

    out->theta_x_ref_deg = Limitf(theta_x_cmd_deg, -x_tilt_limit, x_tilt_limit);
    out->theta_y_ref_deg = Limitf(theta_y_cmd_deg, -y_tilt_limit, y_tilt_limit);
}
