#include "filter.h"  // 滤波器模块头文件

/**
 * @brief 一阶低通滤波器初始化函数
 * @details 初始化滤波器参数，设置滤波系数和初始状态
 * 
 * @param f 指向滤波器结构体的指针
 * @param alpha 滤波系数(0-1)，值越小滤波效果越强，响应越慢
 */
void LPF1_Init(LPF1_t *f, float alpha)
{
    f->alpha = alpha;       // 设置滤波系数
    f->y = 0.0f;            // 初始化输出值为0
    f->inited = 0U;         // 设置未初始化标志
}

/**
 * @brief 一阶低通滤波器运行函数
 * @details 对输入信号进行低通滤波处理，输出平滑后的信号
 * 
 * @param f 指向滤波器结构体的指针
 * @param x 当前输入信号值
 * @return 滤波后的输出信号值
 * 
 * @note 滤波公式：y(n) = alpha * y(n-1) + (1-alpha) * x(n)
 *       其中alpha为滤波系数，控制滤波强度和响应速度
 */
float LPF1_Run(LPF1_t *f, float x)
{
    // 如果滤波器未初始化，直接将输入值作为初始输出
    if (!f->inited)
    {
        f->y = x;           // 设置初始输出为当前输入
        f->inited = 1U;      // 标记为已初始化
        return x;           // 返回当前输入值
    }
    
    // 应用一阶低通滤波公式计算新输出
    f->y = f->alpha * f->y + (1.0f - f->alpha) * x;
    return f->y;            // 返回滤波后的输出值
}

/**
 * @brief 浮点数限幅函数
 * @details 将浮点数限制在指定的最小值和最大值范围内
 * 
 * @param x 待限制的浮点数
 * @param minv 最小限制值
 * @param maxv 最大限制值
 * @return 限制后的浮点数值
 * 
 * @note 如果x小于minv，返回minv；如果x大于maxv，返回maxv；否则返回x
 */
float Clampf(float x, float minv, float maxv)
{
    return Limitf(x, minv, maxv);  // 使用LIMIT宏实现限幅功能
}

/**
 * @brief 浮点数死区处理函数
 * @details 为输入值添加死区处理，消除小信号噪声
 * 
 * @param x 待处理的浮点数
 * @param band 死区宽度(正数)
 * @return 处理后的浮点数值
 * 
 * @note 如果|x| <= band，返回0；如果x > band，返回x - band；如果x < -band，返回x + band
 */
float Deadbandf(float x, float band)
{
    if (x > band) return x - band;    // 正向超出死区，减去死区宽度
    if (x < -band) return x + band;   // 负向超出死区，加上死区宽度
    return 0.0f;                      // 在死区内，返回0
}
