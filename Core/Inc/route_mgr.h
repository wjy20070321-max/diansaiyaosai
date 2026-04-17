#ifndef __ROUTE_MGR_H__
#define __ROUTE_MGR_H__

/* -------------------- 依赖头文件 -------------------- */
#include "app_config.h"   // 系统全局配置头文件，提供基础类型、控制模式、时间参数等宏定义
#include "region.h"       // 区域管理模块，提供区域中心坐标等接口

/* -------------------- 路线步骤结构体 -------------------- */
/**
 * @brief 路线中的单个步骤
 * @note 任务管理器不会直接只认“区域编号”，
 *       而是把任务拆解成一个个具体步骤来执行。
 *
 *       每个步骤描述了：
 *       - 要去哪里（x_mm, y_mm）
 *       - 用什么控制模式去（mode）
 *       - 到了之后要不要停留（need_hold）
 *       - 要停多久（hold_ms）
 *       - 这一步最多允许多久完成（timeout_ms）
 *       - 这个目标是不是一个区域中心（is_region_target）
 *       - 如果是区域目标，对应哪个区域（region_id）
 */
typedef struct
{
    float x_mm;              // 该步骤目标点 X 坐标，单位：毫米
    float y_mm;              // 该步骤目标点 Y 坐标，单位：毫米

    uint8_t mode;            // 该步骤使用的控制模式，如 FAST / BRAKE / HOLD

    uint8_t need_hold;       // 到达后是否需要保持停留：1=需要，0=不需要
    uint16_t hold_ms;        // 如果需要停留，停留时间，单位：毫秒

    uint16_t timeout_ms;     // 该步骤允许的最大完成时间，超时则任务失败

    uint8_t is_region_target;// 是否是区域中心目标：1=是区域目标，0=普通坐标目标
    uint8_t region_id;       // 如果是区域目标，这里保存区域编号
} RouteStep_t;

/* -------------------- 路径构建接口 -------------------- */

/**
 * @brief 构建“区域顺序停留”任务步骤序列
 * @param route      区域编号数组，例如 [1, 2, 6, 9]
 * @param len        路径长度
 * @param hold_ms    每个区域到达后的停留时间，单位：毫秒
 * @param steps      输出步骤数组
 * @param max_steps  steps 数组最大可写入步数
 * @retval 实际构建出的步骤数量
 *
 * @note 这个接口适合“依次去多个区域，并在每个区域停一下”的任务。
 *
 *       例如：
 *       route = {1, 2, 6, 9}
 *       表示：
 *       先去 1 区停一下，再去 2 区停一下，再去 6 区停一下，最后去 9 区停一下
 */
uint8_t RouteMgr_BuildRegionSequence(const uint8_t *route, uint8_t len,
                                     uint16_t hold_ms,
                                     RouteStep_t *steps, uint8_t max_steps);

/**
 * @brief 构建“经过不停留，终点停下”任务步骤序列
 * @param route          区域编号数组
 * @param len            路径长度
 * @param final_hold_ms  最后一个目标点的停留时间，单位：毫秒
 * @param steps          输出步骤数组
 * @param max_steps      steps 数组最大可写入步数
 * @retval 实际构建出的步骤数量
 *
 * @note 这个接口适合“中间点只经过，不停留，最后一个点再停下”的任务。
 *
 *       例如：
 *       route = {1, 2, 6, 9}
 *       表示：
 *       经过 1、2、6，但不在这些中间点停留，
 *       最后到 9 再停住。
 */
uint8_t RouteMgr_BuildPassSequence(const uint8_t *route, uint8_t len,
                                   uint16_t final_hold_ms,
                                   RouteStep_t *steps, uint8_t max_steps);

/**
 * @brief 构建“两点往返”任务步骤序列
 * @param a          起点区域编号
 * @param b          终点区域编号
 * @param cycles     往返循环次数
 * @param hold_ms    每次到达 A 或 B 后的停留时间，单位：毫秒
 * @param steps      输出步骤数组
 * @param max_steps  steps 数组最大可写入步数
 * @retval 实际构建出的步骤数量
 *
 * @note 这个接口适合做 A、B 两点之间来回往返的任务。
 *
 *       例如：
 *       a = 2, b = 8, cycles = 3
 *       表示：
 *       A->B->A->B->A->B 这样循环往返 3 次
 *
 *       具体“1 次循环”在你的 route_mgr.c 里怎么定义，
 *       要以实现文件逻辑为准。
 */
uint8_t RouteMgr_BuildRoundTrip(uint8_t a, uint8_t b, uint8_t cycles,
                                uint16_t hold_ms,
                                RouteStep_t *steps, uint8_t max_steps);

#endif
