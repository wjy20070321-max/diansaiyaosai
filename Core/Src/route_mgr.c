#include "route_mgr.h"  // 路线管理模块头文件
#include "region.h"      // 区域管理模块

/**
 * @brief 向路线步骤数组中添加一个步骤
 * @param steps 路线步骤数组指针
 * @param max_steps 最大允许步骤数
 * @param count 当前已添加步骤计数指针
 * @param x 目标X坐标（毫米）
 * @param y 目标Y坐标（毫米）
 * @param mode 控制模式（如快速移动、制动等）
 * @param need_hold 是否需要保持位置
 * @param hold_ms 保持时间（毫秒）
 * @param timeout_ms 步骤超时时间（毫秒）
 * @param is_region_target 是否为区域目标
 * @param region_id 区域ID（1-9）
 * @return 1表示添加成功，0表示添加失败（超出最大步骤数）
 */
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
 * @brief 根据任务ID构建任务路线
 * @param task_id 任务ID
 * @param user_route 用户自定义路线数组（用于USER_ABCD任务）
 * @param steps 路线步骤数组指针
 * @param max_steps 最大允许步骤数
 * @return 构建的步骤数量
 * 
 * @note 支持的任务类型：
 *       1. TASK_ID_HOLD_2 - 在区域2保持
 *       2. TASK_ID_1_TO_5 - 从区域1到区域5
 *       3. TASK_ID_1_TO_4_TO_5 - 从区域1到区域4再到区域5
 *       4. TASK_ID_1_TO_9 - 从区域1到区域9（绕开中间区域）
 *       5. TASK_ID_1_2_6_9 - 从区域1到区域2到区域6再到区域9
 *       6. TASK_ID_USER_ABCD - 用户自定义路线（A->B->C->D）
 */
uint8_t RouteMgr_BuildTask(uint8_t task_id, const uint8_t *user_route,
                           RouteStep_t *steps, uint8_t max_steps)
{
    uint8_t cnt = 0U;
    Point2f_t c;

    // 根据任务ID构建不同的路线
    if (task_id == TASK_ID_HOLD_2)
    {
        // 任务1：在区域2保持
        c = Region_GetCenter(2);
        PushStep(steps, max_steps, &cnt, c.x, c.y, CTRL_MODE_HOLD, 1U, 5000U, 10000U, 1U, 2U);
    }
    else if (task_id == TASK_ID_1_TO_5)
    {
        // 任务2：从区域1到区域5
        c = Region_GetCenter(5);
        PushStep(steps, max_steps, &cnt, c.x, c.y, CTRL_MODE_HOLD, 1U, 2000U, 15000U, 1U, 5U);
    }
    else if (task_id == TASK_ID_1_TO_4_TO_5)
    {
        // 任务3：从区域1到区域4再到区域5
        c = Region_GetCenter(4);
        PushStep(steps, max_steps, &cnt, c.x, c.y, CTRL_MODE_HOLD, 1U, 2000U, 10000U, 1U, 4U);
        c = Region_GetCenter(5);
        PushStep(steps, max_steps, &cnt, c.x, c.y, CTRL_MODE_HOLD, 1U, 2000U, 10000U, 1U, 5U);
    }
    else if (task_id == TASK_ID_1_TO_9)
    {
        // 任务4：从区域1到区域9（绕开中间区域）
        /* 用边缘安全路径绕开中间区域 */
        PushStep(steps, max_steps, &cnt, 125.0f, 580.0f, CTRL_MODE_FAST, 0U, 0U, 5000U, 0U, 0U);
        PushStep(steps, max_steps, &cnt, 580.0f, 580.0f, CTRL_MODE_FAST, 0U, 0U, 7000U, 0U, 0U);
        PushStep(steps, max_steps, &cnt, 580.0f, 125.0f, CTRL_MODE_BRAKE, 0U, 0U, 7000U, 0U, 0U);
        c = Region_GetCenter(9);
        PushStep(steps, max_steps, &cnt, c.x, c.y, CTRL_MODE_HOLD, 1U, 2000U, 10000U, 1U, 9U);
    }
    else if (task_id == TASK_ID_1_2_6_9)
    {
        // 任务5：从区域1到区域2到区域6再到区域9
        c = Region_GetCenter(2);
        PushStep(steps, max_steps, &cnt, c.x, c.y, CTRL_MODE_BRAKE, 0U, 0U, 10000U, 1U, 2U);
        c = Region_GetCenter(6);
        PushStep(steps, max_steps, &cnt, c.x, c.y, CTRL_MODE_BRAKE, 0U, 0U, 12000U, 1U, 6U);
        c = Region_GetCenter(9);
        PushStep(steps, max_steps, &cnt, c.x, c.y, CTRL_MODE_HOLD, 1U, 2000U, 18000U, 1U, 9U);
    }
    else if (task_id == TASK_ID_USER_ABCD)
    {
        // 任务6：用户自定义路线（A->B->C->D）
        c = Region_GetCenter(user_route[1]);
        PushStep(steps, max_steps, &cnt, c.x, c.y, CTRL_MODE_BRAKE, 0U, 0U, 12000U, 1U, user_route[1]);
        c = Region_GetCenter(user_route[2]);
        PushStep(steps, max_steps, &cnt, c.x, c.y, CTRL_MODE_BRAKE, 0U, 0U, 12000U, 1U, user_route[2]);
        c = Region_GetCenter(user_route[3]);
        PushStep(steps, max_steps, &cnt, c.x, c.y, CTRL_MODE_HOLD, 1U, 2000U, 16000U, 1U, user_route[3]);
    }

    return cnt;
}
