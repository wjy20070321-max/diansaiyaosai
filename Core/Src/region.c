#include "region.h"

/* 静态全局变量定义 */
static Point2f_t g_region[10];

/**
 * @brief 区域管理系统初始化函数
 * @details 新题默认按 600×600 板面布置 9 个圆区中心：
 *          1(50,550)  2(300,550) 3(550,550)
 *          4(50,300)  5(300,300) 6(550,300)
 *          7(50,50)   8(300,50)  9(550,50)
 *
 * @note 如果实物板面不是严格 600×600，请只改这里
 */
void Region_Init(void)
{
    g_region[1] = (Point2f_t){  50.0f, 550.0f };
    g_region[2] = (Point2f_t){ 300.0f, 550.0f };
    g_region[3] = (Point2f_t){ 550.0f, 550.0f };

    g_region[4] = (Point2f_t){  50.0f, 300.0f };
    g_region[5] = (Point2f_t){ 300.0f, 300.0f };
    g_region[6] = (Point2f_t){ 550.0f, 300.0f };

    g_region[7] = (Point2f_t){  50.0f,  50.0f };
    g_region[8] = (Point2f_t){ 300.0f,  50.0f };
    g_region[9] = (Point2f_t){ 550.0f,  50.0f };
}

Point2f_t Region_GetCenter(uint8_t idx)
{
    if (idx < 1U || idx > 9U)
    {
        return (Point2f_t){BOARD_CENTER_X_MM, BOARD_CENTER_Y_MM};
    }
    return g_region[idx];
}

float Region_DistToCenter(uint8_t idx, float x, float y)
{
    Point2f_t c = Region_GetCenter(idx);
    float dx = x - c.x;
    float dy = y - c.y;
    return sqrtf(dx * dx + dy * dy);
}

uint8_t Region_IsEntered(uint8_t idx, float x, float y)
{
    return Region_DistToCenter(idx, x, y) <= REGION_ENTER_RADIUS_MM;
}

uint8_t Region_IsHeld(uint8_t idx, float x, float y)
{
    float r = REGION_HOLD_RADIUS_MM;
    if (r < 1.0f) r = 1.0f;
    return Region_DistToCenter(idx, x, y) <= r;
}

uint8_t Region_FindCurrent(float x, float y)
{
    uint8_t i;
    for (i = 1U; i <= 9U; i++)
    {
        if (Region_IsEntered(i, x, y)) return i;
    }
    return 0U;
}
