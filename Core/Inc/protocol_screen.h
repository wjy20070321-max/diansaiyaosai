#ifndef __PROTOCOL_SCREEN_H__
#define __PROTOCOL_SCREEN_H__

/* -------------------- 依赖头文件 -------------------- */
#include "app_config.h"   // 系统全局配置头文件，提供基础类型、宏和路径长度定义

/* -------------------- 串口屏接收数据结构体 -------------------- */
/**
 * @brief 串口屏接收到的数据结构体
 * @note 串口屏主要承担“人机交互”功能，用来下发：
 *       - 任务启动 / 停止
 *       - IMU 置零
 *       - 单点目标（当前版本为区域号 1~9）
 *       - 指定区域
 *       - 区域路径任务
 *       - 两点往返任务
 */
typedef struct
{
    /* -------------------- 基础控制命令 -------------------- */
    uint8_t task_id;        // 当前收到的任务 ID
    uint8_t start_cmd;      // 启动命令标志：1=收到启动命令
    uint8_t stop_cmd;       // 停止命令标志：1=收到停止命令
    uint8_t imu_zero_cmd;   // IMU 置零命令标志：1=收到置零命令

    /* -------------------- 单点任务（区域 1~9） -------------------- */
    /**
     * @note 当前这版里，POINT=n 不再表示“坐标点”，
     *       而是表示“区域编号 n”。
     *       例如：
     *       - POINT=1 表示去 1 号区
     *       - POINT=5 表示去中心区
     */
    uint8_t point_valid;        // 单点命令是否有效：1=有效，0=无效
    uint8_t point_region_id;    // 单点目标区域编号（1~9）

    /* -------------------- 指定区域任务 -------------------- */
    /**
     * @note REGION=n 是兼容写法，本质上也是指定 1~9 号区域。
     */
    uint8_t region_valid;       // 指定区域命令是否有效：1=有效，0=无效
    uint8_t region_id;          // 目标区域编号（1~9）

    /* -------------------- 路径任务 -------------------- */
    /**
     * @note route_pass_mode 含义：
     *       - 0：逐点停留
     *       - 1：中间点只经过，最后一点停留
     */
    uint8_t route_valid;                // 路径命令是否有效：1=有效，0=无效
    uint8_t route_pass_mode;            // 路径模式：0=逐点停留，1=中间经过不停留
    uint8_t route_len;                  // 路径长度
    uint8_t route[USER_ROUTE_MAX_LEN];  // 路径点数组（区域编号）

    /* -------------------- 两点往返任务 -------------------- */
    uint8_t roundtrip_valid;    // 两点往返命令是否有效：1=有效，0=无效
    uint8_t roundtrip_a;        // 往返点 A
    uint8_t roundtrip_b;        // 往返点 B

    /* -------------------- 更新时间戳 -------------------- */
    uint32_t update_ms;         // 最近一次收到有效串口屏命令的时间戳，单位：毫秒

} ScreenRxData_t;

/* -------------------- 串口屏协议模块接口函数 -------------------- */

/**
 * @brief 串口屏协议模块初始化
 * @note 清空解析状态、接收缓冲和命令结构体
 */
void ProtocolScreen_Init(void);

/**
 * @brief 处理串口收到的单个字节
 * @param byte 当前收到的一个字节
 *
 * @note 当前协议是“按文本行解析”的：
 *       - '\r' 忽略
 *       - '\n' 表示一整条命令结束
 */
void ProtocolScreen_RxByte(uint8_t byte);

/**
 * @brief 获取当前串口屏解析结果结构体指针
 * @retval 指向内部 ScreenRxData_t 的指针
 */
ScreenRxData_t* ProtocolScreen_GetData(void);

/**
 * @brief 发送字符串到串口屏
 * @param text 要发送的字符串指针
 *
 * @note 具体走哪个串口，以 protocol_screen.c 里的实现为准。
 *       你当前这版实现已改为 huart2。 :contentReference[oaicite:2]{index=2}
 */
void ProtocolScreen_SendText(const char *text);

/**
 * @brief 主动发送当前系统状态到串口屏
 * @note 通常用于周期上报：
 *       - 小球坐标
 *       - 目标坐标
 *       - 当前区域
 *       - 当前任务号
 *       - 运行/完成/失败状态
 *       - 保持时间
 *       - 总运行时间
 */
void ProtocolScreen_SendStatus(void);

#endif
