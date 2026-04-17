#ifndef __JY61P_H__
#define __JY61P_H__

/* -------------------- 依赖头文件 -------------------- */
#include "app_config.h"   // 系统全局配置头文件，提供基础类型和宏定义

/* -------------------- JY61P 数据结构体 -------------------- */
/**
 * @brief JY61P 姿态传感器数据结构体
 * @note 用于保存从 JY61P 模块解析出来的姿态角、角速度、零偏和状态信息
 *
 *       其中：
 *       - roll / pitch / yaw 是姿态角
 *       - gyro_x / gyro_y / gyro_z 是角速度
 *       - roll_zero / pitch_zero 是当前设定的零点偏移
 *       - valid 表示当前数据是否有效
 *       - update_ms 记录最近一次收到有效数据的时间戳
 */
typedef struct
{
    float roll_deg;      // 横滚角 Roll，单位：度
    float pitch_deg;     // 俯仰角 Pitch，单位：度
    float yaw_deg;       // 航向角 Yaw，单位：度

    float gyro_x_dps;    // X 轴角速度，单位：度/秒
    float gyro_y_dps;    // Y 轴角速度，单位：度/秒
    float gyro_z_dps;    // Z 轴角速度，单位：度/秒

    float roll_zero;     // 横滚角零偏，用于平台水平校准
    float pitch_zero;    // 俯仰角零偏，用于平台水平校准

    uint8_t valid;       // 数据有效标志：1=有效，0=无效
    uint32_t update_ms;  // 最近一次收到有效 IMU 数据的时间戳，单位：毫秒
} JY61P_Data_t;

/* -------------------- JY61P 模块接口函数 -------------------- */

/**
 * @brief JY61P 模块初始化
 * @note 用于初始化内部数据结构、接收缓冲区和状态变量
 */
void JY61P_Init(void);

/**
 * @brief JY61P 单字节接收处理函数
 * @param byte 串口接收到的一个字节
 *
 * @note 一般在串口接收中断回调里调用，
 *       该函数会把收到的字节逐个拼成完整数据帧并解析出姿态数据
 */
void JY61P_RxByte(uint8_t byte);

/**
 * @brief 设置当前姿态为零点
 * @note 一般在平台放平后调用，
 *       调用后会把当前 roll / pitch 保存到 roll_zero / pitch_zero 中，
 *       之后控制器可用“当前角度 - 零偏”得到校准后的姿态角
 */
void JY61P_SetZero(void);

/**
 * @brief 安全拷贝一份 IMU 数据
 * @param dest 目标结构体指针
 *
 * @note 因为 IMU 数据可能在串口中断里被更新，
 *       主循环直接访问全局结构体时可能出现“读一半被中断改掉”的问题。
 *       所以这里提供 CopyData 接口，用于安全地整体拷贝一份当前数据。
 */
void JY61P_CopyData(JY61P_Data_t *dest);

/**
 * @brief 获取 IMU 全局数据结构体指针
 * @retval 指向 JY61P_Data_t 全局对象的指针
 *
 * @note 这个接口适合需要直接访问内部数据的场景，
 *       但如果你担心中断更新造成数据撕裂，优先建议用 JY61P_CopyData()。
 */
JY61P_Data_t* JY61P_GetData(void);

#endif
