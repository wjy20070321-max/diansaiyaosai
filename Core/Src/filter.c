#include "filter.h"   // 滤波器模块头文件

/**
 * @brief 一阶低通滤波器初始化函数
 * @param f     指向一阶低通滤波器对象的指针
 * @param alpha 滤波系数，通常取值范围为 0~1
 *
 * @note 该函数用于初始化一个一阶低通滤波器。
 *       初始化后：
 *       - alpha 保存滤波系数
 *       - y 清零
 *       - inited 置 0，表示滤波器还没有真正接收过有效输入
 *
 *       这里不直接把滤波输出初始化成某个固定输入值，
 *       而是等第一次调用 LPF1_Run() 时，再把首次输入作为初始输出，
 *       这样能避免启动瞬间出现不必要的突变。
 */
void LPF1_Init(LPF1_t *f, float alpha)
{
    f->alpha = alpha;   // 设置滤波系数
    f->y = 0.0f;        // 输出初值先清零
    f->inited = 0U;     // 标记为“尚未完成首次数值初始化”
}

/**
 * @brief 一阶低通滤波器运行函数
 * @param f 指向一阶低通滤波器对象的指针
 * @param x 当前输入值
 * @retval 当前滤波输出值
 *
 * @note 这是一个标准的一阶低通滤波器。
 *
 *       首次运行时：
 *       - 不直接使用公式滤波
 *       - 而是把当前输入 x 直接作为初始输出 y
 *       这样做的好处是可以避免系统刚启动时，
 *       因为历史输出值是 0 而导致滤波输出突然偏差很大。
 *
 *       非首次运行时，使用公式：
 *           y = alpha * y + (1 - alpha) * x
 *
 *       其中：
 *       - alpha 越大，输出越依赖历史值，变化更平缓
 *       - alpha 越小，输出越跟随当前输入，响应更快
 */
float LPF1_Run(LPF1_t *f, float x)
{
    /* 如果滤波器尚未初始化过真实输入，
       则直接把当前输入作为初始输出 */
    if (!f->inited)
    {
        f->y = x;        // 首次输出直接等于当前输入
        f->inited = 1U;  // 标记滤波器已经完成初始化
        return x;        // 返回当前输入值
    }

    /* 应用一阶低通滤波公式，更新输出 */
    f->y = f->alpha * f->y + (1.0f - f->alpha) * x;

    /* 返回最新滤波结果 */
    return f->y;
}

/**
 * @brief 浮点数限幅函数
 * @param x    输入值
 * @param minv 最小允许值
 * @param maxv 最大允许值
 * @retval 限幅后的结果
 *
 * @note 该函数本质上是对 app_config.h 中 Limitf() 的一次封装。
 *       用途是把输入值限制在 [minv, maxv] 区间内。
 *
 *       例如：
 *       - 控制器输出限幅
 *       - 倾角限幅
 *       - 舵机命令限幅
 */
float Clampf(float x, float minv, float maxv)
{
    return Limitf(x, minv, maxv);  // 调用全局通用限幅函数
}

/**
 * @brief 浮点数死区处理函数
 * @param x    输入值
 * @param band 死区宽度，通常为正数
 * @retval 死区处理后的结果
 *
 * @note 死区处理的意义是：
 *       当输入值非常小时，直接当作 0，
 *       以减少微小噪声、抖动或无意义的小动作。
 *
 *       处理规则为：
 *       - 如果 x > band，则输出 x - band
 *       - 如果 x < -band，则输出 x + band
 *       - 如果 -band <= x <= band，则输出 0
 *
 *       这种处理方式相比“单纯置零”更平滑，
 *       因为超过死区后，输出是连续衔接的。
 */
float Deadbandf(float x, float band)
{
    if (x > band) return x - band;    // 正方向超出死区，减去死区宽度
    if (x < -band) return x + band;   // 负方向超出死区，加上死区宽度
    return 0.0f;                      // 落在死区范围内，直接输出 0
}
