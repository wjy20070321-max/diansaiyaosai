#ifndef __PROTOCOL_SCREEN_H__
#define __PROTOCOL_SCREEN_H__

/* -------------------- 依赖头文件 -------------------- */
#include "app_config.h"   // 系统全局配置头文件，提供基础类型、宏和路径长度定义

/* -------------------- 串口屏接收数据结构体 -------------------- */
/**
 * @brief 串口屏接收到的数据结构体
 * @note 该结构体用于统一保存从串口屏解析出来的控制命令和参数
 *
 *       串口屏在这个项目里主要承担“人机交互”功能，
 *       也就是让你通过屏幕下发：
 *       - 任务启动/停止
 *       - IMU 置零
 *       - 指定坐标点
 *       - 指定区域
 *       - 路径任务
 *       - 两点往返任务
 */
typedef struct
{
    /* -------------------- 基础控制命令 -------------------- */
    uint8_t task_id;        // 当前收到的任务 ID
    uint8_t start_cmd;      // 启动命令标志：1=收到启动命令
    uint8_t stop_cmd;       // 停止命令标志：1=收到停止命令
    uint8_t imu_zero_cmd;   // IMU 置零命令标志：1=收到置零命令

    /* -------------------- 指定坐标点任务 -------------------- */
    uint8_t point_valid;    // 指定点是否有效：1=有效，0=无效
    float point_x_mm;       // 指定目标点 X 坐标，单位：毫米
    float point_y_mm;       // 指定目标点 Y 坐标，单位：毫米

    /* -------------------- 指定区域任务 -------------------- */
    uint8_t region_valid;   // 指定区域命令是否有效：1=有效，0=无效
    uint8_t region_id;      // 目标区域编号

    /* -------------------- 路径任务 -------------------- */
    uint8_t route_valid;        // 路径命令是否有效：1=有效，0=无效
    uint8_t route_pass_mode;    // 路径模式标志：如“经过不停留”或“到点停留”
    uint8_t route_len;          // 路径长度，即路径中有效点个数
    uint8_t route[USER_ROUTE_MAX_LEN]; // 路径点数组，保存一串区域编号

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
 * @note 用于初始化内部状态机、接收缓冲区和 ScreenRxData_t 结构体
 */
void ProtocolScreen_Init(void);

/**
 * @brief 串口屏单字节接收处理函数
 * @param byte 串口收到的一个字节
 *
 * @note 一般在 USART 接收中断中调用。
 *       该函数负责把串口屏发来的字节流逐步解析成完整命令。
 */
void ProtocolScreen_RxByte(uint8_t byte);

/**
 * @brief 获取串口屏当前接收数据结构体指针
 * @retval 指向 ScreenRxData_t 全局对象的指针
 *
 * @note 通过这个接口，控制器模块可以读取串口屏当前解析出的命令状态。
 */
ScreenRxData_t* ProtocolScreen_GetData(void);

/**
 * @brief 向串口屏发送文本字符串
 * @param text 要发送的字符串指针
 *
 * @note 该接口通常用于简单调试输出或给串口屏发送提示信息。
 *       例如：
 *       - READY
 *       - 当前状态提示
 *       - 调试信息
 */
void ProtocolScreen_SendText(const char *text);

/* -------------------- 主动状态上报接口 -------------------- */
/**
 * @brief 主动发送系统状态到串口屏
 *
 * @note 这个函数是新增的主动上报接口，
 *       可用于定期把系统当前状态发送到屏幕显示，例如：
 *       - 当前任务 ID
 *       - 球是否有效
 *       - 当前坐标
 *       - 控制模式
 *       - 运行状态等
 *
 *       具体发送哪些内容，要看 protocol_screen.c 里的实现。
 */
void ProtocolScreen_SendStatus(void);

#endif
