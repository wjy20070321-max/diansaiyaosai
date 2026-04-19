#include "route_mgr.h"   // 路径构建模块头文件
#include <math.h>

/**
 * @brief 向步骤数组中追加一个任务步骤
 * @param steps            目标步骤数组
 * @param max_steps        步骤数组最大容量
 * @param count            当前已写入的步骤数指针
 * @param x                目标点 X 坐标，单位：毫米
 * @param y                目标点 Y 坐标，单位：毫米
 * @param mode             该步骤使用的控制模式
 * @param need_hold        到达该步骤后是否需要保持停留：1=需要，0=不需要
 * @param hold_ms          若需要停留，则停留时间，单位：毫秒
 * @param timeout_ms       该步骤最大允许执行时间，单位：毫秒
 * @param is_region_target 该步骤是否是区域中心目标：1=是，0=否
 * @param region_id        若是区域目标，对应的区域编号
 * @retval 1=追加成功，0=追加失败（通常是数组已满）
 */
static uint8_t PushStep(RouteStep_t *steps, uint8_t max_steps, uint8_t *count,
                        float x, float y, uint8_t mode,
                        uint8_t need_hold, uint16_t hold_ms, uint16_t timeout_ms,
                        uint8_t is_region_target, uint8_t region_id)
{
    if (*count >= max_steps)
    {
        return 0U;
    }

    steps[*count].x_mm = x;
    steps[*count].y_mm = y;
    steps[*count].mode = mode;
    steps[*count].need_hold = need_hold;
    steps[*count].hold_ms = hold_ms;
    steps[*count].timeout_ms = timeout_ms;
    steps[*count].is_region_target = is_region_target;
    steps[*count].region_id = region_id;

    (*count)++;
    return 1U;
}

/**
 * @brief 超过阈值固定拆成三段，没超过就不分
 * @note
 * - 中间两个虚拟点：CTRL_MODE_BRAKE + 保持 1s
 * - 最终真实目标点：保持原来的模式和保持逻辑
 */
static uint8_t PushSegmentedMove(RouteStep_t *steps, uint8_t max_steps, uint8_t *count,
                                 Point2f_t from, Point2f_t to,
                                 uint8_t final_mode,
                                 uint8_t final_need_hold, uint16_t final_hold_ms,
                                 uint8_t final_is_region, uint8_t final_region_id)
{
    float dx;
    float dy;
    float dist;
    uint8_t segs;
    uint8_t k;

    dx = to.x - from.x;
    dy = to.y - from.y;
    dist = sqrtf(dx * dx + dy * dy);

    /* 不超过阈值：不拆；超过阈值：固定拆成三段 */
    if (dist > ROUTE_SPLIT_THRESHOLD_MM)
    {
        segs = 3U;
    }
    else
    {
        segs = 1U;
    }

    /* 插入中间虚拟点：BRAKE + 保持1秒 */
    for (k = 1U; k < segs; k++)
    {
        float r = (float)k / (float)segs;
        float mx = from.x + dx * r;
        float my = from.y + dy * r;

        if (!PushStep(steps, max_steps, count,
                      mx, my,
                      CTRL_MODE_BRAKE,
                      1U,
                      VIRTUAL_POINT_HOLD_MS,
                      DEFAULT_STEP_TIMEOUT_MS,
                      0U,
                      0U))
        {
            return 0U;
        }
    }

    /* 最后插入真实目标点 */
    return PushStep(steps, max_steps, count,
                    to.x, to.y,
                    final_mode,
                    final_need_hold,
                    final_hold_ms,
                    DEFAULT_STEP_TIMEOUT_MS,
                    final_is_region,
                    final_region_id);
}

/**
 * @brief 构建“多区域顺序停留”任务
 */
uint8_t RouteMgr_BuildRegionSequence(const uint8_t *route, uint8_t len,
                                     uint16_t hold_ms,
                                     RouteStep_t *steps, uint8_t max_steps)
{
    uint8_t i;
    uint8_t cnt = 0U;
    Point2f_t c;
    Point2f_t prev_c;
    uint8_t has_prev = 0U;

    if (route == NULL || len == 0U)
    {
        return 0U;
    }

    for (i = 0U; i < len; i++)
    {
        if (route[i] < 1U || route[i] > 9U)
        {
            continue;
        }

        c = Region_GetCenter(route[i]);

        /* 第一个正式目标点：直接去 */
        if (!has_prev)
        {
            if (!PushStep(steps, max_steps, &cnt,
                          c.x, c.y,
                          CTRL_MODE_HOLD,
                          1U,
                          hold_ms,
                          DEFAULT_STEP_TIMEOUT_MS,
                          1U,
                          route[i]))
            {
                break;
            }

            prev_c = c;
            has_prev = 1U;
        }
        else
        {
            /* 后续正式目标点：超过阈值固定拆三段，没超过不分 */
            if (!PushSegmentedMove(steps, max_steps, &cnt,
                                   prev_c, c,
                                   CTRL_MODE_HOLD,
                                   1U,
                                   hold_ms,
                                   1U,
                                   route[i]))
            {
                break;
            }

            prev_c = c;
        }
    }

    return cnt;
}

/**
 * @brief 构建“经过不停留，终点停下”任务
 */
uint8_t RouteMgr_BuildPassSequence(const uint8_t *route, uint8_t len,
                                   uint16_t final_hold_ms,
                                   RouteStep_t *steps, uint8_t max_steps)
{
    uint8_t i;
    uint8_t cnt = 0U;
    Point2f_t c;
    Point2f_t prev_c;
    uint8_t has_prev = 0U;

    if (route == NULL || len == 0U)
    {
        return 0U;
    }

    for (i = 0U; i < len; i++)
    {
        if (route[i] < 1U || route[i] > 9U)
        {
            continue;
        }

        c = Region_GetCenter(route[i]);

        /* 第一个正式目标点 */
        if (!has_prev)
        {
            if (i < (len - 1U))
            {
                if (!PushStep(steps, max_steps, &cnt,
                              c.x, c.y,
                              CTRL_MODE_BRAKE,
                              0U,
                              0U,
                              DEFAULT_STEP_TIMEOUT_MS,
                              1U,
                              route[i]))
                {
                    break;
                }
            }
            else
            {
                if (!PushStep(steps, max_steps, &cnt,
                              c.x, c.y,
                              CTRL_MODE_HOLD,
                              1U,
                              final_hold_ms,
                              DEFAULT_STEP_TIMEOUT_MS,
                              1U,
                              route[i]))
                {
                    break;
                }
            }

            prev_c = c;
            has_prev = 1U;
        }
        else
        {
            if (i < (len - 1U))
            {
                if (!PushSegmentedMove(steps, max_steps, &cnt,
                                       prev_c, c,
                                       CTRL_MODE_BRAKE,
                                       0U,
                                       0U,
                                       1U,
                                       route[i]))
                {
                    break;
                }
            }
            else
            {
                if (!PushSegmentedMove(steps, max_steps, &cnt,
                                       prev_c, c,
                                       CTRL_MODE_HOLD,
                                       1U,
                                       final_hold_ms,
                                       1U,
                                       route[i]))
                {
                    break;
                }
            }

            prev_c = c;
        }
    }

    return cnt;
}

/**
 * @brief 构建“两点往返”任务
 */
uint8_t RouteMgr_BuildRoundTrip(uint8_t a, uint8_t b, uint8_t cycles,
                                uint16_t hold_ms,
                                RouteStep_t *steps, uint8_t max_steps)
{
    uint8_t i;
    uint8_t cnt = 0U;
    Point2f_t ca;
    Point2f_t cb;
    Point2f_t prev_c;
    uint8_t has_prev = 0U;

    if (a < 1U || a > 9U || b < 1U || b > 9U || cycles == 0U)
    {
        return 0U;
    }

    ca = Region_GetCenter(a);
    cb = Region_GetCenter(b);

    for (i = 0U; i < cycles; i++)
    {
        /* 去 A */
        if (!has_prev)
        {
            if (!PushStep(steps, max_steps, &cnt,
                          ca.x, ca.y,
                          CTRL_MODE_HOLD,
                          1U,
                          hold_ms,
                          DEFAULT_STEP_TIMEOUT_MS,
                          1U,
                          a))
            {
                break;
            }

            prev_c = ca;
            has_prev = 1U;
        }
        else
        {
            if (!PushSegmentedMove(steps, max_steps, &cnt,
                                   prev_c, ca,
                                   CTRL_MODE_HOLD,
                                   1U,
                                   hold_ms,
                                   1U,
                                   a))
            {
                break;
            }

            prev_c = ca;
        }

        /* 去 B */
        if (!PushSegmentedMove(steps, max_steps, &cnt,
                               prev_c, cb,
                               CTRL_MODE_HOLD,
                               1U,
                               hold_ms,
                               1U,
                               b))
        {
            break;
        }

        prev_c = cb;
    }

    return cnt;
}
