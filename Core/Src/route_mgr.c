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
 * @brief 长边中继规则
 * @note 这些就是你现在想手工指定的“先到中继点，再到终点”的规则。
 */
typedef struct
{
    uint8_t from_region;
    uint8_t to_region;
    float relay_x_mm;
    float relay_y_mm;
} RelayRule_t;

/**
 * @brief 查询某一对区域之间是否定义了中继点
 * @param from_region 起始区域编号
 * @param to_region   终止区域编号
 * @param relay       输出中继点坐标
 * @retval 1=有中继点，0=没有
 */
static uint8_t GetRelayPoint(uint8_t from_region, uint8_t to_region, Point2f_t *relay)
{
    uint32_t i;

    /* 你点名的这些长边，统一手工加中继点 */
    static const RelayRule_t rules[] =
    {
        /* 1 -> 6, 6 -> 1 */
        {1U, 6U, 305.0f, 430.0f},
        {6U, 1U, 305.0f, 430.0f},

        /* 1 -> 8, 8 -> 1 */
        {1U, 8U, 180.0f, 305.0f},
        {8U, 1U, 180.0f, 305.0f},

        /* 3 -> 8, 8 -> 3 */
        {3U, 8U, 430.0f, 305.0f},
        {8U, 3U, 430.0f, 305.0f},

        /* 3 -> 4, 4 -> 3 */
        {3U, 4U, 305.0f, 430.0f},
        {4U, 3U, 305.0f, 430.0f},

        /* 9 -> 4, 4 -> 9 */
        {9U, 4U, 305.0f, 180.0f},
        {4U, 9U, 305.0f, 180.0f},

        /* 9 -> 2, 2 -> 9 */
        {9U, 2U, 430.0f, 305.0f},
        {2U, 9U, 430.0f, 305.0f},

        /* 7 -> 2, 2 -> 7 */
        {7U, 2U, 180.0f, 305.0f},
        {2U, 7U, 180.0f, 305.0f},

        /* 7 -> 6, 6 -> 7 */
        {7U, 6U, 305.0f, 180.0f},
        {6U, 7U, 305.0f, 180.0f},

        /* 1 -> 9, 9 -> 1 */
        {1U, 9U, 305.0f, 305.0f},
        {9U, 1U, 305.0f, 305.0f},

        /* 3 -> 7, 7 -> 3 */
        {3U, 7U, 305.0f, 305.0f},
        {7U, 3U, 305.0f, 305.0f}
    };

    if (relay == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < (sizeof(rules) / sizeof(rules[0])); i++)
    {
        if ((rules[i].from_region == from_region) &&
            (rules[i].to_region   == to_region))
        {
            relay->x = rules[i].relay_x_mm;
            relay->y = rules[i].relay_y_mm;
            return 1U;
        }
    }

    return 0U;
}

/**
 * @brief 在两个正式区域点之间，按需插入一个中继点
 * @param steps            步骤数组
 * @param max_steps        最大步骤数
 * @param count            当前步骤计数指针
 * @param from_region      起始区域编号
 * @param to_region        终止区域编号
 * @param final_x          最终点 X
 * @param final_y          最终点 Y
 * @param final_mode       最终点模式
 * @param final_need_hold  最终点是否保持
 * @param final_hold_ms    最终点保持时间
 * @param final_is_region  最终点是否区域目标
 * @param final_region_id  最终点区域号
 * @param relay_need_hold  中继点是否保持
 * @retval 1=成功，0=失败
 */
static uint8_t PushMoveWithOptionalRelay(RouteStep_t *steps, uint8_t max_steps, uint8_t *count,
                                         uint8_t from_region, uint8_t to_region,
                                         float final_x, float final_y,
                                         uint8_t final_mode,
                                         uint8_t final_need_hold, uint16_t final_hold_ms,
                                         uint8_t final_is_region, uint8_t final_region_id,
                                         uint8_t relay_need_hold)
{
    Point2f_t relay;

    /* 如果这对区域定义了中继点，则先插入中继点 */
    if (GetRelayPoint(from_region, to_region, &relay))
    {
        if (!PushStep(steps, max_steps, count,
                      relay.x, relay.y,
                      CTRL_MODE_BRAKE,
                      relay_need_hold,
                      relay_need_hold ? VIRTUAL_POINT_HOLD_MS : 0U,
                      DEFAULT_STEP_TIMEOUT_MS,
                      0U,
                      0U))
        {
            return 0U;
        }
    }

    /* 再插入真正目标点 */
    return PushStep(steps, max_steps, count,
                    final_x, final_y,
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
    uint8_t prev_region = 0U;

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
        if (prev_region == 0U)
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
        }
        else
        {
            /* 后续正式目标点：若命中长边规则，则先去中继点再去终点 */
            if (!PushMoveWithOptionalRelay(steps, max_steps, &cnt,
                                           prev_region, route[i],
                                           c.x, c.y,
                                           CTRL_MODE_HOLD,
                                           1U,
                                           hold_ms,
                                           1U,
                                           route[i],
                                           1U))
            {
                break;
            }
        }

        prev_region = route[i];
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
    uint8_t prev_region = 0U;

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
        if (prev_region == 0U)
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
        }
        else
        {
            if (i < (len - 1U))
            {
                /* PASS 模式中，中继点不保持，保持连续经过 */
                if (!PushMoveWithOptionalRelay(steps, max_steps, &cnt,
                                               prev_region, route[i],
                                               c.x, c.y,
                                               CTRL_MODE_BRAKE,
                                               0U,
                                               0U,
                                               1U,
                                               route[i],
                                               0U))
                {
                    break;
                }
            }
            else
            {
                if (!PushMoveWithOptionalRelay(steps, max_steps, &cnt,
                                               prev_region, route[i],
                                               c.x, c.y,
                                               CTRL_MODE_HOLD,
                                               1U,
                                               final_hold_ms,
                                               1U,
                                               route[i],
                                               0U))
                {
                    break;
                }
            }
        }

        prev_region = route[i];
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
    uint8_t prev_region = 0U;

    if (a < 1U || a > 9U || b < 1U || b > 9U || cycles == 0U)
    {
        return 0U;
    }

    ca = Region_GetCenter(a);
    cb = Region_GetCenter(b);

    for (i = 0U; i < cycles; i++)
    {
        /* 去 A */
        if (prev_region == 0U)
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
        }
        else
        {
            if (!PushMoveWithOptionalRelay(steps, max_steps, &cnt,
                                           prev_region, a,
                                           ca.x, ca.y,
                                           CTRL_MODE_HOLD,
                                           1U,
                                           hold_ms,
                                           1U,
                                           a,
                                           1U))
            {
                break;
            }
        }

        prev_region = a;

        /* 去 B */
        if (!PushMoveWithOptionalRelay(steps, max_steps, &cnt,
                                       prev_region, b,
                                       cb.x, cb.y,
                                       CTRL_MODE_HOLD,
                                       1U,
                                       hold_ms,
                                       1U,
                                       b,
                                       1U))
        {
            break;
        }

        prev_region = b;
    }

    return cnt;
}
