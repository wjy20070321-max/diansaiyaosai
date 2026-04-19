/** ****************************************************************************
 * @file    ball_outer_loop.c
 * @brief   小球位置外环控制模块源文件（路径进度加减速版，按你这台机构重调）
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

/**
 * @brief 目标变化超过该值，认为进入新路径段
 * @note  外环会把“当前球位置”当成新起点，重新规划一条到新目标的路径
 *
 * 调大：
 *  - 不容易频繁重建路径
 *  - 目标有轻微抖动时更稳
 *  - 但切换目标会变迟钝
 *
 * 调小：
 *  - 对目标变化更敏感
 *  - 但如果目标点本身有抖动，路径会老重置，球会发飘
 */
#define PATH_RELOCK_TARGET_EPS_MM      (3.0f)

/**
 * @brief FAST 模式下的路径峰值速度（毫米/秒）
 * @note  远距离快速推进时，希望球达到的目标最高速度
 *
 * 调大：
 *  - 前半程加速更猛，整体更快
 *  - 但更容易后半程刹不住、冲过头
 *
 * 调小：
 *  - 前半程更稳
 *  - 但整体会变肉
 */
#define PATH_VPEAK_FAST_MMPS           (110.0f)

/**
 * @brief BRAKE 模式下的路径峰值速度（毫米/秒）
 * @note  中距离减速阶段允许的目标速度上限
 *
 * 调大：
 *  - 中段减速不那么明显
 *  - 更容易“减速晚、停不住”
 *
 * 调小：
 *  - 更早进入明显减速
 *  - 但可能提前慢下来
 */
#define PATH_VPEAK_BRAKE_MMPS          (40.0f)

/**
 * @brief HOLD 模式下的路径峰值速度（毫米/秒）
 * @note  接近终点时允许的目标速度上限
 *
 * 调大：
 *  - 最后一段更愿意继续往前贴
 *  - 但容易在目标附近晃
 *
 * 调小：
 *  - 终点附近更稳
 *  - 但容易“差一点到不了”
 */
#define PATH_VPEAK_HOLD_MMPS           (5.5f)

/**
 * @brief FAST 模式下，速度误差转换成倾角的比例
 * @note  速度误差越大，板子沿路径方向给的倾角越大
 *
 * 调大：
 *  - 前半程加速更猛
 *  - 也更容易动作生硬
 *
 * 调小：
 *  - 更平缓
 *  - 但可能推不动
 */
#define PATH_K_TAN_FAST                (0.025f)

/**
 * @brief BRAKE 模式下，速度误差转换成倾角的比例
 * @note  主要决定“减速阶段”板子反向刹车有多积极
 *
 * 调大：
 *  - 后半程更愿意刹车
 *  - 但太大容易突然反向、动作发硬
 *
 * 调小：
 *  - 更柔和
 *  - 但容易减速晚
 */
#define PATH_K_TAN_BRAKE               (0.037f)

/**
 * @brief HOLD 模式下，速度误差转换成倾角的比例
 * @note  主要决定目标附近小范围修正时的敏感度
 *
 * 调大：
 *  - 更愿意修正末段误差
 *  - 但更容易在终点附近来回抖
 *
 * 调小：
 *  - 更稳
 *  - 但可能差一点不进去
 */
#define PATH_K_TAN_HOLD                (0.035f)

/**
 * @brief 终点附近的位置“收尾拉回”系数
 * @note  即使目标速度已经降下来了，只要还没到终点，
 *        仍然按“剩余距离 remain_mm”给一点朝终点的倾角
 *
 * 调大：
 *  - 更容易把最后一点距离补过去
 *  - 但太大了会在终点附近来回扯
 *
 * 调小：
 *  - 更柔和
 *  - 但容易提前停住、慢慢蹭过去
 */
#define PATH_K_END_POS                 (0.009f)

/**
 * @brief 终点增强窗口（毫米）
 * @note  当离终点小于这个距离时，会把 PATH_K_END_POS 再增强
 *
 * 调大：
 *  - 更早开始“最后冲一下”
 *  - 但可能更早被拉得不自然
 *
 * 调小：
 *  - 只有 very near target 才增强
 *  - 更自然，但可能最后一小段不够有力
 */
#define PATH_END_BOOST_WINDOW_MM       (35.0f)

/**
 * @brief 终点增强倍率
 * @note  进入终点增强窗口后，PATH_K_END_POS 会乘以这个系数
 *
 * 调大：
 *  - 最后一小段更有力，更容易补进目标区
 *  - 但太大容易过点后反复扯
 *
 * 调小：
 *  - 更平滑
 *  - 但可能最后差一点
 */
#define PATH_END_BOOST_SCALE           (2.0f)

/**
 * @brief 横向位置纠偏 P 系数
 * @note  球偏离“起点到终点那条直线”多少，就按比例往回拉多少
 *
 * 调大：
 *  - 更愿意走直线
 *  - 但太大容易左右来回摆
 *
 * 调小：
 *  - 横向更柔和
 *  - 但路径可能变弯、偏着走
 */
#define PATH_K_LAT_P                   (0.045f)

/**
 * @brief 横向速度阻尼 D 系数
 * @note  横向移动速度越大，阻尼越强，用来抑制左右摆动
 *
 * 调大：
 *  - 横向更稳，更不容易振荡
 *  - 但太大会显得迟钝
 *
 * 调小：
 *  - 横向更灵活
 *  - 但容易过终点后左右晃
 */
#define PATH_K_LAT_D                   (0.0060f)


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
 * - 到终点时不直接掉到 0，而是保留一点点向前速度，避免提前停死
 * - 超过终点：目标速度给负值，帮助反向制动
 */
static float BuildDesiredLineSpeed(float progress, float v_peak)
{
    if (progress <= 0.0f)
    {
        return 0.20f * v_peak;
    }

    if (progress < 0.5f)
    {
        /* 0~0.5：从 0.2*v_peak 线性升到 1.0*v_peak */
        return v_peak * (0.20f + 1.60f * progress);
    }

    if (progress <= 1.0f)
    {
        /* 0.5~1.0：从 1.0*v_peak 线性降到 0.15*v_peak */
        float norm = (progress - 0.5f) / 0.5f;   /* 0~1 */
        return v_peak * (1.00f - 0.85f * norm);
    }

    /* 已经过终点：给一个反向目标速度，帮助刹车回拉 */
    return -0.20f * v_peak;
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
    float tx, ty;             /* 路径切向单位向量 */
    float nx, ny;             /* 路径法向单位向量 */

    float cx, cy;             /* 当前点相对起点坐标 */
    float s;                  /* 当前在路径上的投影长度 */
    float progress;           /* 当前路径进度 */

    float v_line;             /* 当前沿路径方向的速度 */
    float v_lat;              /* 当前横向速度 */
    float v_peak;             /* 当前模式下的峰值目标速度 */
    float v_ref;              /* 当前进度下的目标线速度 */

    float remain_mm;          /* 剩余路程 */
    float end_gain;           /* 终点拉回增益 */
    float tan_cmd_deg;        /* 沿路径方向的倾角命令 */
    float lat_cmd_deg;        /* 横向纠偏倾角命令 */

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

    /* 根据路径进度生成目标线速度 */
    v_ref = BuildDesiredLineSpeed(progress, v_peak);

    /* 剩余路程（允许为负，表示已经冲过目标） */
    remain_mm = path_len - s;

    /* 终点附近增强一点“拉回”能力，避免提前停住后慢慢蹭过去 */
    end_gain = PATH_K_END_POS;
    if (ABSF(remain_mm) <= PATH_END_BOOST_WINDOW_MM)
    {
        end_gain *= PATH_END_BOOST_SCALE;
    }

    /* -------------------- 沿路径方向控制 -------------------- */
    tan_cmd_deg = GetTanGainByMode(mode) * (v_ref - v_line)
                + end_gain * remain_mm;

    /* -------------------- 横向纠偏控制 -------------------- */
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
