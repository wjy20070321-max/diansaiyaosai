#include "region.h"  // 区域管理模块头文件

// 静态全局变量定义
static Point2f_t g_region[10];  // 存储区域中心点坐标，索引1-9有效

/**
 * @brief 区域管理系统初始化函数
 * @details 初始化9个区域的中心点坐标，形成3x3网格布局
 * 
 * @note 区域布局（单位：毫米）：
 *       1(125,525)  2(325,525)  3(525,525)
 *       4(125,325)  5(325,325)  6(525,325)
 *       7(125,125)  8(325,125)  9(525,125)
 */
void Region_Init(void)
{
    g_region[1] = (Point2f_t){125.0f, 525.0f};  // 区域1
    g_region[2] = (Point2f_t){325.0f, 525.0f};  // 区域2
    g_region[3] = (Point2f_t){525.0f, 525.0f};  // 区域3
    g_region[4] = (Point2f_t){125.0f, 325.0f};  // 区域4
    g_region[5] = (Point2f_t){325.0f, 325.0f};  // 区域5
    g_region[6] = (Point2f_t){525.0f, 325.0f};  // 区域6
    g_region[7] = (Point2f_t){125.0f, 125.0f};  // 区域7
    g_region[8] = (Point2f_t){325.0f, 125.0f};  // 区域8
    g_region[9] = (Point2f_t){525.0f, 125.0f};  // 区域9
}

/**
 * @brief 获取指定区域的中心点坐标
 * @param idx 区域索引（1-9）
 * @return 区域中心点坐标
 * 
 * @note 如果索引无效，返回板子中心坐标
 */
Point2f_t Region_GetCenter(uint8_t idx)
{
    if (idx < 1U || idx > 9U)
    {
        return (Point2f_t){BOARD_CENTER_X_MM, BOARD_CENTER_Y_MM};
    }
    return g_region[idx];
}

/**
 * @brief 计算点到区域中心的距离
 * @param idx 区域索引（1-9）
 * @param x 点的X坐标
 * @param y 点的Y坐标
 * @return 距离值（毫米）
 */
float Region_DistToCenter(uint8_t idx, float x, float y)
{
    Point2f_t c = Region_GetCenter(idx);
    float dx = x - c.x;
    float dy = y - c.y;
    return sqrtf(dx * dx + dy * dy);
}

/**
 * @brief 判断点是否进入区域
 * @param idx 区域索引（1-9）
 * @param x 点的X坐标
 * @param y 点的Y坐标
 * @return 1表示进入，0表示未进入
 * 
 * @note 使用REGION_ENTER_RADIUS_MM作为判断半径
 */
uint8_t Region_IsEntered(uint8_t idx, float x, float y)
{
    return Region_DistToCenter(idx, x, y) <= REGION_ENTER_RADIUS_MM;
}

/**
 * @brief 判断点是否保持在区域内
 * @param idx 区域索引（1-9）
 * @param x 点的X坐标
 * @param y 点的Y坐标
 * @return 1表示保持，0表示未保持
 * 
 * @note 使用REGION_HOLD_RADIUS_MM作为判断半径
 * @note 如果保持半径小于1mm，自动设为1mm
 */
uint8_t Region_IsHeld(uint8_t idx, float x, float y)
{
    float r = REGION_HOLD_RADIUS_MM;
    if (r < 1.0f) r = 1.0f;
    return Region_DistToCenter(idx, x, y) <= r;
}

/**
 * @brief 查找当前点所在的区域
 * @param x 点的X坐标
 * @param y 点的Y坐标
 * @return 区域索引（1-9），0表示不在任何区域内
 */
uint8_t Region_FindCurrent(float x, float y)
{
    uint8_t i;
    for (i = 1U; i <= 9U; i++)
    {
        if (Region_IsEntered(i, x, y)) return i;
    }
    return 0U;
}
