#ifndef __PROTOCOL_PI_H__
#define __PROTOCOL_PI_H__

/* -------------------- 依赖头文件 -------------------- */
#include "app_config.h"   // 系统全局配置头文件，提供基础类型和协议命令码等宏定义

/* -------------------- 树莓派接收数据结构体 -------------------- */
/**
 * @brief 树莓派（PI）接收到的数据总结构体
 * @note 该结构体用于统一保存从树莓派串口协议中解析出来的所有数据
 *
 *       主要分为 4 类内容：
 *       1. 小球状态数据（位置、速度、是否有效）
 *       2. 任务控制命令（任务号、开始、停止）
 *       3. 路径/目标设置命令
 *       4. 激光追踪目标数据
 *
 *       另外还带有若干时间戳字段，用于判断数据是否超时。
 */
typedef struct
{
    /* -------------------- 小球状态数据 -------------------- */
    float ball_x_mm;       // 小球当前 X 坐标，单位：毫米
    float ball_y_mm;       // 小球当前 Y 坐标，单位：毫米
    float ball_vx_mmps;    // 小球当前 X 方向速度，单位：毫米/秒
    float ball_vy_mmps;    // 小球当前 Y 方向速度，单位：毫米/秒
    uint8_t ball_valid;    // 小球状态是否有效：1=有效，0=无效

    /* -------------------- 任务控制命令 -------------------- */
    uint8_t task_id;       // 当前收到的任务 ID
    uint8_t start_cmd;     // 启动命令标志：1=收到启动命令
    uint8_t stop_cmd;      // 停止命令标志：1=收到停止命令

    /* -------------------- 路径设置命令 -------------------- */
    uint8_t route_a;       // 路径点 A（区域编号）
    uint8_t route_b;       // 路径点 B（区域编号）
    uint8_t route_c;       // 路径点 C（区域编号）
    uint8_t route_d;       // 路径点 D（区域编号）

    /* -------------------- 目标点设置命令 -------------------- */
    float target_x_mm;     // 目标点 X 坐标，单位：毫米
    float target_y_mm;     // 目标点 Y 坐标，单位：毫米
    uint8_t target_valid;  // 目标点是否有效：1=有效，0=无效

    /* -------------------- 通用更新时间戳 -------------------- */
    /**
     * @brief 最近一次收到任意 PI 协议帧的时间
     * @note 这是为了兼容旧代码保留的字段
     *       含义是：“只要树莓派发来任意一帧数据，这个时间就刷新”
     */
    uint32_t update_ms;

    /* -------------------- 分类时间戳 -------------------- */
    /**
     * @brief 最近一次收到“小球状态帧”的时间
     * @note 只有小球状态数据更新时才刷新
     */
    uint32_t ball_update_ms;

    /**
     * @brief 最近一次收到“命令类帧”的时间
     * @note 包括 task / route / target 等命令
     */
    uint32_t cmd_update_ms;

    /**
     * @brief 最近一次收到“激光追踪帧”的时间
     */
    uint32_t laser_update_ms;

    /* -------------------- 激光追踪数据 -------------------- */
    float laser_x_mm;      // 激光目标点 X 坐标，单位：毫米
    float laser_y_mm;      // 激光目标点 Y 坐标，单位：毫米
    uint8_t laser_valid;   // 激光目标点是否有效：1=有效，0=无效

} PiRxData_t;

/* -------------------- PI 协议模块接口函数 -------------------- */

/**
 * @brief PI 协议解析器初始化
 * @note 用于初始化内部状态机、缓冲区和接收数据结构体
 */
void ProtocolPi_Init(void);

/**
 * @brief PI 协议单字节接收处理函数
 * @param byte 串口收到的一个字节
 *
 * @note 一般在 USART 接收中断里调用。
 *       该函数会把收到的字节按协议格式逐个拼接，
 *       当拼成完整一帧后再进行校验和解析。
 */
void ProtocolPi_RxByte(uint8_t byte);

/**
 * @brief 安全拷贝一份当前树莓派数据
 * @param dest 目标结构体指针
 *
 * @note 由于树莓派数据可能在串口中断中被更新，
 *       主循环如果直接访问全局数据，可能出现“读到一半数据被改掉”的问题。
 *       所以这里提供 CopyData()，用于安全地整体复制当前数据快照。
 */
void ProtocolPi_CopyData(PiRxData_t *dest);

/**
 * @brief 获取 PI 全局数据结构体指针
 * @retval 指向当前 PI 接收数据结构体的指针
 *
 * @note 适合需要直接访问内部数据的场景。
 *       如果你更关心数据一致性，通常更建议优先使用 ProtocolPi_CopyData()。
 */
PiRxData_t* ProtocolPi_GetData(void);

/* -------------------- 原子消费接口 -------------------- */
/**
 * @note 下面这几个接口的特点是：
 *       “读取后立刻清空”
 *
 *       这样做的目的：
 *       - 防止命令被重复执行
 *       - 防止旧命令长时间残留
 *       - 避免比赛时出现误触发
 */

/**
 * @brief 原子读取并清空任务控制命令
 * @param task_id   返回任务 ID
 * @param start_cmd 返回启动命令
 * @param stop_cmd  返回停止命令
 * @retval 1=本次确实取到了新命令，0=没有新命令
 *
 * @note 调用成功后，内部保存的 task_id / start_cmd / stop_cmd 会被清空。
 */
uint8_t ProtocolPi_ConsumeTaskCtrl(uint8_t *task_id, uint8_t *start_cmd, uint8_t *stop_cmd);

/**
 * @brief 原子读取并清空路径设置命令
 * @param a 返回路径点 A
 * @param b 返回路径点 B
 * @param c 返回路径点 C
 * @param d 返回路径点 D
 * @retval 1=本次取到了有效路径，0=没有新路径
 *
 * @note 调用成功后，内部保存的 route_a / route_b / route_c / route_d 会被清空。
 */
uint8_t ProtocolPi_ConsumeRoute(uint8_t *a, uint8_t *b, uint8_t *c, uint8_t *d);

/**
 * @brief 原子读取并清空目标点设置命令
 * @param x_mm 返回目标点 X 坐标
 * @param y_mm 返回目标点 Y 坐标
 * @retval 1=本次取到了有效目标点，0=没有新目标点
 *
 * @note 调用成功后，内部保存的 target_x_mm / target_y_mm / target_valid 会被清空。
 */
uint8_t ProtocolPi_ConsumeTarget(float *x_mm, float *y_mm);

#endif
