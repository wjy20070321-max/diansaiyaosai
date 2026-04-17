#include "route_mgr.h"   // 路径构建模块头文件

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
 *
 * @note 这是一个内部工具函数，用来统一构造 RouteStep_t。
 *       这样后面构建各种任务序列时，就不用反复写相同赋值代码。
 */
static uint8_t PushStep(RouteStep_t *steps, uint8_t max_steps, uint8_t *count,
                        float x, float y, uint8_t mode,
                        uint8_t need_hold, uint16_t hold_ms, uint16_t timeout_ms,
                        uint8_t is_region_target, uint8_t region_id)
{
    /* 如果当前步骤数已经达到数组上限，则追加失败 */
    if (*count >= max_steps)
    {
        return 0U;
    }

    /* 把当前这一步的目标信息写入 steps[count] */
    steps[*count].x_mm = x;                         // 目标点 X 坐标
    steps[*count].y_mm = y;                         // 目标点 Y 坐标
    steps[*count].mode = mode;                      // 控制模式
    steps[*count].need_hold = need_hold;            // 是否需要停留
    steps[*count].hold_ms = hold_ms;                // 停留时间
    steps[*count].timeout_ms = timeout_ms;          // 步骤超时时间
    steps[*count].is_region_target = is_region_target; // 是否是区域中心目标
    steps[*count].region_id = region_id;            // 所属区域编号

    /* 步骤数加 1，表示追加成功 */
    (*count)++;
    return 1U;
}

/**
 * @brief 构建“多区域顺序停留”任务
 * @param route     区域编号数组
 * @param len       区域数量
 * @param hold_ms   每个区域到达后的停留时间，单位：毫秒
 * @param steps     输出步骤数组
 * @param max_steps 步骤数组最大容量
 * @retval 实际构建出的步骤数量
 *
 * @note 这个函数适合构建“依次去多个区域，并且在每个区域都停一下”的任务。
 *
 *       例如：
 *       route = {1,2,6,9}
 *       表示：
 *       - 先去 1 号区并保持
 *       - 再去 2 号区并保持
 *       - 再去 6 号区并保持
 *       - 最后去 9 号区并保持
 *
 *       每一步都会使用 CTRL_MODE_HOLD，
 *       因为设计意图就是要在这些目标区稳定停下来。
 */
uint8_t RouteMgr_BuildRegionSequence(const uint8_t *route, uint8_t len,
                                     uint16_t hold_ms,
                                     RouteStep_t *steps, uint8_t max_steps)
{
    uint8_t i;        // 遍历路径数组的循环变量
    uint8_t cnt = 0U; // 当前已构建出的步骤数量
    Point2f_t c;      // 当前区域中心坐标

    /* 参数保护：路径数组为空或长度为 0，则无法构建任务 */
    if (route == NULL || len == 0U)
    {
        return 0U;
    }

    /* 依次处理 route 中的每一个区域点 */
    for (i = 0U; i < len; i++)
    {
        /* 如果区域编号非法（不是 1~9），则跳过 */
        if (route[i] < 1U || route[i] > 9U)
        {
            continue;
        }

        /* 取出当前区域中心坐标 */
        c = Region_GetCenter(route[i]);

        /* 追加一个“去该区域并保持”的步骤 */
        if (!PushStep(steps, max_steps, &cnt,
                      c.x, c.y,                   // 区域中心坐标
                      CTRL_MODE_HOLD,             // 使用保持模式
                      1U,                         // 需要保持停留
                      hold_ms,                    // 保持时间
                      DEFAULT_STEP_TIMEOUT_MS,    // 步骤超时时间
                      1U,                         // 是区域目标
                      route[i]))                  // 当前区域编号
        {
            /* 如果数组满了或追加失败，就停止继续构建 */
            break;
        }
    }

    /* 返回实际成功构建的步骤数 */
    return cnt;
}

/**
 * @brief 构建“经过不停留，终点停下”任务
 * @param route          区域编号数组
 * @param len            路径长度
 * @param final_hold_ms  最后一个区域的保持时间，单位：毫秒
 * @param steps          输出步骤数组
 * @param max_steps      步骤数组最大容量
 * @retval 实际构建出的步骤数量
 *
 * @note 这个函数适合构建“前面几个点只经过，不停留，最后一个点再稳定停下”的任务。
 *
 *       设计逻辑：
 *       - 前面的点：CTRL_MODE_BRAKE + need_hold=0
 *         表示只要求进入该区域即可，不要求停住
 *       - 最后一个点：CTRL_MODE_HOLD + need_hold=1
 *         表示到终点后必须稳定停住一段时间
 *
 *       例如：
 *       route = {1,2,6,9}
 *       表示：
 *       经过 1、2、6，不停留；
 *       最后到 9 并保持 final_hold_ms 毫秒。
 */
uint8_t RouteMgr_BuildPassSequence(const uint8_t *route, uint8_t len,
                                   uint16_t final_hold_ms,
                                   RouteStep_t *steps, uint8_t max_steps)
{
    uint8_t i;        // 路径遍历变量
    uint8_t cnt = 0U; // 当前已构建步骤数
    Point2f_t c;      // 当前区域中心坐标

    /* 参数保护 */
    if (route == NULL || len == 0U)
    {
        return 0U;
    }

    /* 依次处理每一个区域点 */
    for (i = 0U; i < len; i++)
    {
        /* 非法区域编号直接跳过 */
        if (route[i] < 1U || route[i] > 9U)
        {
            continue;
        }

        /* 获取该区域中心坐标 */
        c = Region_GetCenter(route[i]);

        /* 如果不是最后一个点，则构建“经过但不停留”的步骤 */
        if (i < (len - 1U))
        {
            if (!PushStep(steps, max_steps, &cnt,
                          c.x, c.y,                 // 当前区域中心
                          CTRL_MODE_BRAKE,          // 使用 BRAKE 模式，强调减速通过
                          0U,                       // 不需要保持
                          0U,                       // 停留时间为 0
                          DEFAULT_STEP_TIMEOUT_MS,  // 超时时间
                          1U,                       // 是区域目标
                          route[i]))                // 当前区域编号
            {
                break;
            }
        }
        /* 如果是最后一个点，则构建“到终点后保持”的步骤 */
        else
        {
            if (!PushStep(steps, max_steps, &cnt,
                          c.x, c.y,                 // 当前区域中心
                          CTRL_MODE_HOLD,           // 最终点使用 HOLD 模式
                          1U,                       // 需要保持
                          final_hold_ms,            // 终点保持时间
                          DEFAULT_STEP_TIMEOUT_MS,  // 超时时间
                          1U,                       // 是区域目标
                          route[i]))                // 当前区域编号
            {
                break;
            }
        }
    }

    return cnt;
}

/**
 * @brief 构建“两点往返”任务
 * @param a          区域 A
 * @param b          区域 B
 * @param cycles     往返循环次数
 * @param hold_ms    每次到达后保持时间，单位：毫秒
 * @param steps      输出步骤数组
 * @param max_steps  步骤数组最大容量
 * @retval 实际构建出的步骤数量
 *
 * @note 当前实现逻辑是：
 *       cycles=2 时，会生成：
 *           A -> B -> A -> B
 *
 *       也就是说，每个 cycle 都会追加两步：
 *       1. 去 A 并保持
 *       2. 去 B 并保持
 *
 *       这里每一步都使用 CTRL_MODE_HOLD，
 *       说明设计意图是到达后稳定停住，而不是仅仅经过。
 */
uint8_t RouteMgr_BuildRoundTrip(uint8_t a, uint8_t b, uint8_t cycles,
                                uint16_t hold_ms,
                                RouteStep_t *steps, uint8_t max_steps)
{
    uint8_t i;        // 循环次数计数器
    uint8_t cnt = 0U; // 当前已构建步骤数
    Point2f_t c;      // 当前区域中心坐标

    /* 参数保护：
       - A/B 必须是合法区域编号 1~9
       - cycles 必须大于 0 */
    if (a < 1U || a > 9U || b < 1U || b > 9U || cycles == 0U)
    {
        return 0U;
    }

    /* 按 cycles 次数重复构建 A->B */
    for (i = 0U; i < cycles; i++)
    {
        /* 先追加去 A 的步骤 */
        c = Region_GetCenter(a);
        if (!PushStep(steps, max_steps, &cnt,
                      c.x, c.y,                  // A 区域中心坐标
                      CTRL_MODE_HOLD,            // 使用保持模式
                      1U,                        // 到达后需要保持
                      hold_ms,                   // 保持时间
                      DEFAULT_STEP_TIMEOUT_MS,   // 步骤超时时间
                      1U,                        // 是区域目标
                      a))                        // 区域 A 编号
        {
            break;
        }

        /* 再追加去 B 的步骤 */
        c = Region_GetCenter(b);
        if (!PushStep(steps, max_steps, &cnt,
                      c.x, c.y,                  // B 区域中心坐标
                      CTRL_MODE_HOLD,            // 使用保持模式
                      1U,                        // 到达后需要保持
                      hold_ms,                   // 保持时间
                      DEFAULT_STEP_TIMEOUT_MS,   // 步骤超时时间
                      1U,                        // 是区域目标
                      b))                        // 区域 B 编号
        {
            break;
        }
    }

    return cnt;
}
