#include "route_mgr.h"

static uint8_t PushStep(RouteStep_t *steps, uint8_t max_steps, uint8_t *count,
                        float x, float y, uint8_t mode,
                        uint8_t need_hold, uint16_t hold_ms, uint16_t timeout_ms,
                        uint8_t is_region_target, uint8_t region_id)
{
    if (*count >= max_steps) return 0U;

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
 * @brief 构建“多区域顺序停留”任务
 * @param route 区域编号数组
 * @param len   区域数量
 * @param hold_ms 每个区域保持时间
 */
uint8_t RouteMgr_BuildRegionSequence(const uint8_t *route, uint8_t len,
                                     uint16_t hold_ms,
                                     RouteStep_t *steps, uint8_t max_steps)
{
    uint8_t i;
    uint8_t cnt = 0U;
    Point2f_t c;

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

        if (!PushStep(steps, max_steps, &cnt,
                      c.x, c.y,
                      CTRL_MODE_HOLD,
                      1U, hold_ms, DEFAULT_STEP_TIMEOUT_MS,
                      1U, route[i]))
        {
            break;
        }
    }

    return cnt;
}

/**
 * @brief 构建“经过不停留，终点停下”任务
 * @note 前面的点只要求进入，不保持；最后一点保持 final_hold_ms
 */
uint8_t RouteMgr_BuildPassSequence(const uint8_t *route, uint8_t len,
                                   uint16_t final_hold_ms,
                                   RouteStep_t *steps, uint8_t max_steps)
{
    uint8_t i;
    uint8_t cnt = 0U;
    Point2f_t c;

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

        if (i < (len - 1U))
        {
            if (!PushStep(steps, max_steps, &cnt,
                          c.x, c.y,
                          CTRL_MODE_BRAKE,
                          0U, 0U, DEFAULT_STEP_TIMEOUT_MS,
                          1U, route[i]))
            {
                break;
            }
        }
        else
        {
            if (!PushStep(steps, max_steps, &cnt,
                          c.x, c.y,
                          CTRL_MODE_HOLD,
                          1U, final_hold_ms, DEFAULT_STEP_TIMEOUT_MS,
                          1U, route[i]))
            {
                break;
            }
        }
    }

    return cnt;
}

/**
 * @brief 构建“两点往返”任务
 * @note cycles=2 时形成 A->B->A->B，每次到达保持 hold_ms
 */
uint8_t RouteMgr_BuildRoundTrip(uint8_t a, uint8_t b, uint8_t cycles,
                                uint16_t hold_ms,
                                RouteStep_t *steps, uint8_t max_steps)
{
    uint8_t i;
    uint8_t cnt = 0U;
    Point2f_t c;

    if (a < 1U || a > 9U || b < 1U || b > 9U || cycles == 0U)
    {
        return 0U;
    }

    for (i = 0U; i < cycles; i++)
    {
        c = Region_GetCenter(a);
        if (!PushStep(steps, max_steps, &cnt,
                      c.x, c.y,
                      CTRL_MODE_HOLD,
                      1U, hold_ms, DEFAULT_STEP_TIMEOUT_MS,
                      1U, a))
        {
            break;
        }

        c = Region_GetCenter(b);
        if (!PushStep(steps, max_steps, &cnt,
                      c.x, c.y,
                      CTRL_MODE_HOLD,
                      1U, hold_ms, DEFAULT_STEP_TIMEOUT_MS,
                      1U, b))
        {
            break;
        }
    }

    return cnt;
}
