#include "region.h"   // 区域管理模块头文件

/* -------------------- 模块内静态全局变量 -------------------- */
/* 用于保存 1~9 号区域的中心坐标。
   这里开了 10 个元素，下标 1~9 有效，下标 0 不使用。 */
static Point2f_t g_region[10];

/**
 * @brief 区域管理模块初始化函数
 *
 * @details 当前默认按 600mm × 600mm 的板面布置 9 个目标区域中心：
 *
 *          1(50,550)   2(300,550)   3(550,550)
 *          4(50,300)   5(300,300)   6(550,300)
 *          7(50,50)    8(300,50)    9(550,50)
 *
 * @note 这个布局相当于在板面上建立了一个 3×3 的九宫格区域系统。
 *
 *       如果以后你们的实际板面尺寸、区域分布或者区域坐标定义变了，
 *       优先只需要修改这里的中心点坐标配置。
 */
void Region_Init(void)
{
    /* 第一行（靠上方） */
    g_region[1] = (Point2f_t){  50.0f, 555.0f };
    g_region[2] = (Point2f_t){ 315.0f, 555.0f };
    g_region[3] = (Point2f_t){ 545.0f, 555.0f };

    /* 第二行（中间） */
    g_region[4] = (Point2f_t){  55.0f, 305.0f };
    g_region[5] = (Point2f_t){ 305.0f, 305.0f };
    g_region[6] = (Point2f_t){ 545.0f, 305.0f };

    /* 第三行（靠下方） */
    g_region[7] = (Point2f_t){  55.0f,  50.0f };
    g_region[8] = (Point2f_t){ 315.0f,  55.0f };
    g_region[9] = (Point2f_t){ 565.0f,  55.0f };
}

/**
 * @brief 获取指定区域的中心坐标
 * @param idx 区域编号
 * @retval 对应区域的中心点坐标
 *
 * @note 合法区域编号范围是 1~9。
 *       如果传入的区域编号非法，则默认返回平台中心坐标，
 *       这样可以避免访问越界，同时给系统一个“安全默认值”。
 */
Point2f_t Region_GetCenter(uint8_t idx)
{
    /* 非法区域编号保护 */
    if (idx < 1U || idx > 9U)
    {
        return (Point2f_t){BOARD_CENTER_X_MM, BOARD_CENTER_Y_MM};
    }

    /* 返回指定区域中心坐标 */
    return g_region[idx];
}

/**
 * @brief 计算某点到指定区域中心的距离
 * @param idx 区域编号
 * @param x   当前点 X 坐标，单位：毫米
 * @param y   当前点 Y 坐标，单位：毫米
 * @retval 当前点到该区域中心的欧氏距离，单位：毫米
 *
 * @note 该函数是区域判定的基础。
 *       后续“是否进入区域”“是否稳定保持在区域内”等判断，
 *       本质上都是看这个距离是否小于某个半径阈值。
 */
float Region_DistToCenter(uint8_t idx, float x, float y)
{
    Point2f_t c = Region_GetCenter(idx); // 先取出该区域中心坐标
    float dx = x - c.x;                  // X 方向距离差
    float dy = y - c.y;                  // Y 方向距离差

    /* 返回二维平面上的欧氏距离 */
    return sqrtf(dx * dx + dy * dy);
}

/**
 * @brief 判断某点是否已经进入指定区域
 * @param idx 区域编号
 * @param x   当前点 X 坐标，单位：毫米
 * @param y   当前点 Y 坐标，单位：毫米
 * @retval 1=已经进入该区域，0=未进入
 *
 * @note “进入区域”的判定标准是：
 *       当前点到区域中心的距离 <= REGION_ENTER_RADIUS_MM
 *
 *       这个半径一般会比“保持半径”略大，
 *       用来让系统更容易判定“已经到达目标区域附近”。
 */
uint8_t Region_IsEntered(uint8_t idx, float x, float y)
{
    return Region_DistToCenter(idx, x, y) <= REGION_ENTER_RADIUS_MM;
}

/**
 * @brief 判断某点是否稳定保持在指定区域内
 * @param idx 区域编号
 * @param x   当前点 X 坐标，单位：毫米
 * @param y   当前点 Y 坐标，单位：毫米
 * @retval 1=已经稳定保持在该区域内，0=未保持
 *
 * @note “保持”判定一般比“进入”更严格，
 *       也就是要求当前点更靠近区域中心。
 *
 *       这里使用 REGION_HOLD_RADIUS_MM 作为保持半径，
 *       但为了防止配置异常导致半径太小甚至非正，
 *       又额外做了一个最小保护：
 *           r >= 1.0mm
 */
uint8_t Region_IsHeld(uint8_t idx, float x, float y)
{
    float r = REGION_HOLD_RADIUS_MM; // 默认保持半径

    /* 防止保持半径配置过小，导致逻辑异常 */
    if (r < 1.0f)
    {
        r = 1.0f;
    }

    return Region_DistToCenter(idx, x, y) <= r;
}

/**
 * @brief 查找当前点属于哪个区域
 * @param x 当前点 X 坐标，单位：毫米
 * @param y 当前点 Y 坐标，单位：毫米
 * @retval 返回当前所在区域编号；如果不在任何区域内，则返回 0
 *
 * @note 判断方法：
 *       从 1 号区域到 9 号区域依次检查，
 *       只要某个区域满足“进入区域”条件，就立即返回该区域编号。
 *
 *       如果所有区域都不满足，则返回 0，表示当前不在任何目标区域内。
 */
uint8_t Region_FindCurrent(float x, float y)
{
    uint8_t i;

    /* 依次检查 1~9 号区域 */
    for (i = 1U; i <= 9U; i++)
    {
        if (Region_IsEntered(i, x, y))
        {
            return i; // 找到所在区域后立即返回
        }
    }

    /* 不在任何区域中 */
    return 0U;
}
