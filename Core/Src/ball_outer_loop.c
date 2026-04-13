/**
 ******************************************************************************
 * @file    ball_outer_loop.c
 * @brief   球位置外环控制模块源文件 (LQR最优控制+前馈版)
 ******************************************************************************
 */

#include "ball_outer_loop.h"
#include "app_config.h" 
#include "filter.h"     

// ==========================================================
// ?? LQR 最优控制参数区 
// ==========================================================
// K[0] 对应位置误差的惩罚权重 (相当于超强的 P)
// K[1] 对应速度误差的惩罚权重 (相当于超强的 D)
static const float LQR_K_X[2] = { 0.0015f, 0.0025f }; 
static const float LQR_K_Y[2] = { 0.0015f, 0.0025f }; 

// 前馈增益常数 (物理公式推导: 1 / (5/7 * 9810))
#define FF_GAIN  (0.0001427f)  

// ==========================================================
// ?? 目标状态缓存区 (用于计算动态目标的移动速度和加速度)
// ==========================================================
static float last_target_x = 0.0f;
static float last_target_y = 0.0f;
static float last_target_vx = 0.0f;
static float last_target_vy = 0.0f;

// 假设外环的运行周期是 10ms (100Hz)，根据头文件注释设为 0.01s
#define DT_S (0.01f) 

// 内部使用的安全限幅函数保护
static float LQR_Limitf(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/**
 * @brief 外环初始化
 */
void BallOuterLoop_Init(void)
{
    BallOuterLoop_Reset();
}

/**
 * @brief 外环重置 (清除历史数据，防止产生瞬时大加速度)
 */
void BallOuterLoop_Reset(void)
{
    last_target_x = 0.0f;
    last_target_y = 0.0f;
    last_target_vx = 0.0f;
    last_target_vy = 0.0f;
}

/**
 * @brief  核心 LQR 轴控制器 (带前馈)
 */
static float LQR_Axis_Calc(float ref_p, float ref_v, float ref_a, 
                           float act_p, float act_v, const float* K)
{
    // 1. 算出状态误差 (实际 - 目标)
    float err_pos = act_p - ref_p;
    float err_vel = act_v - ref_v;
    
    // 2. LQR 状态反馈计算: u_fb = -K * Error
    float u_fb_rad = -(K[0] * err_pos + K[1] * err_vel);
    
    // 3. 前馈计算: 让平台提前倾斜去接应目标的加速度
    float u_ff_rad = ref_a * FF_GAIN;
    
    // 4. 总控输出 (弧度转角度: * 57.2958f)
    float total_deg = (u_fb_rad + u_ff_rad) * 57.2958f;
    
    // 5. 极值保护 (限制最大倾角为 10度)
    return LQR_Limitf(total_deg, -10.0f, 10.0f);
}

/**
 * @brief 外环主函数：被总控管理器(controller_mgr)调用
 */
void BallOuterLoop_Run(float x_ref, float y_ref,
                       float x_meas, float y_meas,
                       float vx_meas, float vy_meas,
                       uint8_t mode,
                       BallOuterOutput_t *out)
{
    // --- 第 1 步：解析目标(激光)，并求导算出它的动态 ---
    
    // 计算目标的移动速度 (当前位置 - 上次位置) / 时间
    float target_vx_raw = (x_ref - last_target_x) / DT_S;
    float target_vy_raw = (y_ref - last_target_y) / DT_S;
    
    // 计算目标的加速度 (当前速度 - 上次速度) / 时间
    float target_ax_raw = (target_vx_raw - last_target_vx) / DT_S;
    float target_ay_raw = (target_vy_raw - last_target_vy) / DT_S;
    
    // (此处若有噪声，可在此添加对 target_vx_raw 和 target_ax_raw 的低通滤波)
    float target_vx = target_vx_raw; 
    float target_vy = target_vy_raw; 
    float target_ax = target_ax_raw; 
    float target_ay = target_ay_raw; 
    
    // 记录历史数据给下一次求导用
    last_target_x = x_ref;
    last_target_y = y_ref;
    last_target_vx = target_vx;
    last_target_vy = target_vy;
    
    // --- 第 2 步：计算 LQR 控制指令 ---
    
    // 如果系统处于正常运行模式
    if (mode == 0 || mode == 2) // 兼容你的 CTRL_MODE_FAST 和 CTRL_MODE_HOLD
    {
        // 经典板球交叉控制映射：倾斜Y轴改变的是小球沿X轴的位置，倾斜X轴改变的是Y轴位置。
        // 注意：如果你上机发现某一个轴反了，把下面对应的 LQR_Axis_Calc 前面加个负号 '-' 即可。
        out->theta_y_ref_deg = LQR_Axis_Calc(x_ref, target_vx, target_ax, x_meas, vx_meas, LQR_K_X);
        out->theta_x_ref_deg = LQR_Axis_Calc(y_ref, target_vy, target_ay, y_meas, vy_meas, LQR_K_Y); 
    }
    else 
    {
        // 刹车模式或异常模式，平台回平
        out->theta_x_ref_deg = 0.0f;
        out->theta_y_ref_deg = 0.0f;
    }
}
