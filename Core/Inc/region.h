#ifndef __REGION_H__
#define __REGION_H__

/* -------------------- 依赖头文件 -------------------- */
#include "app_config.h"   // 系统全局配置头文件，提供基础类型、板子尺寸、区域参数等宏定义

/* -------------------- 二维坐标点结构体 -------------------- */
/**
 * @brief 二维坐标点结构体
 * @note 用于表示平面上的一个点，通常用来存放区域中心坐标
 */
typedef struct
{
    float x;   // X 坐标，单位：毫米
    float y;   // Y 坐标，单位：毫米
} Point2f_t;

/* -------------------- 区域管理模块接口 -------------------- */

/**
 * @brief 区域管理模块初始化
 * @note 用于初始化各个区域（如 1~9 号区域）的中心坐标
 *
 *       一般会在系统启动时调用一次，
 *       把九宫格区域或比赛题目规定区域的位置先配置好。
 */
void Region_Init(void);

/**
 * @brief 获取指定区域的中心坐标
 * @param idx 区域编号
 * @retval 返回该区域中心点坐标
 *
 * @note 例如：
 *       - 区域 1 的中心坐标
 *       - 区域 5（中心区）的中心坐标
 *       - 区域 9 的中心坐标
 *
 *       通常用于任务规划：
 *       “去某个区域”本质上就是“去这个区域中心点附近”。
 */
Point2f_t Region_GetCenter(uint8_t idx);

/**
 * @brief 判断某个坐标是否已经进入指定区域
 * @param idx 区域编号
 * @param x   当前点 X 坐标，单位：毫米
 * @param y   当前点 Y 坐标，单位：毫米
 * @retval 1=已进入该区域，0=未进入
 *
 * @note “进入区域”一般表示小球已经靠近到该区域中心一定范围内，
 *       判定半径通常会参考 REGION_ENTER_RADIUS_MM。
 */
uint8_t Region_IsEntered(uint8_t idx, float x, float y);

/**
 * @brief 判断某个坐标是否稳定保持在指定区域
 * @param idx 区域编号
 * @param x   当前点 X 坐标，单位：毫米
 * @param y   当前点 Y 坐标，单位：毫米
 * @retval 1=已稳定保持在该区域，0=未保持
 *
 * @note “保持”通常比“进入”要求更严格，
 *       也就是要求点更靠近区域中心，避免刚擦边进入就被判定为稳定到达。
 */
uint8_t Region_IsHeld(uint8_t idx, float x, float y);

/**
 * @brief 查找当前点所在的区域
 * @param x 当前点 X 坐标，单位：毫米
 * @param y 当前点 Y 坐标，单位：毫米
 * @retval 当前所在区域编号
 *
 * @note 如果点当前不在任何有效区域范围内，
 *       返回值通常取决于 region.c 里的具体实现。
 */
uint8_t Region_FindCurrent(float x, float y);

/**
 * @brief 计算当前点到指定区域中心的距离
 * @param idx 区域编号
 * @param x   当前点 X 坐标，单位：毫米
 * @param y   当前点 Y 坐标，单位：毫米
 * @retval 返回该点到指定区域中心的距离，单位：毫米
 *
 * @note 这个函数常用于：
 *       - 判断小球距离目标区域还有多远
 *       - 做到达判定
 *       - 做保持判定
 *       - 做任务切换条件判断
 */
float Region_DistToCenter(uint8_t idx, float x, float y);

#endif
