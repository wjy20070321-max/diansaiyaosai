#include "jy61p.h"  // JY61P IMU传感器驱动头文件
#include <string.h>

// 静态全局变量定义
static JY61P_Data_t g_jy61p;  // 存储IMU数据的结构体
static uint8_t g_buf[JY61P_FRAME_LEN];  // 接收数据缓冲区
static uint8_t g_idx = 0;  // 缓冲区索引

// 外部声明的系统毫秒计时器
extern volatile uint32_t g_sys_ms;

/**
 * @brief 安全获取IMU数据（防止中断打断导致数据撕裂）
 * @param dest 目标结构体指针
 */
void JY61P_CopyData(JY61P_Data_t *dest)
{
    __disable_irq();        // 【关全局中断】进入临界区
    *dest = g_jy61p;        // 【修复】原子拷贝整个结构体，使用正确的真实变量名
    __enable_irq();         // 【开全局中断】退出临界区
}

/**
 * @brief 将两个字节转换为16位有符号整数
 */
static int16_t ReadS16(uint8_t lo, uint8_t hi)
{
    return (int16_t)((uint16_t)hi << 8 | lo);
}

/**
 * @brief 计算数据帧校验和
 */
static uint8_t CheckSum(const uint8_t *buf)
{
    uint8_t i;
    uint8_t sum = 0;
    for (i = 0; i < 10U; i++) sum += buf[i];
    return sum;
}

/**
 * @brief JY61P传感器初始化函数
 */
void JY61P_Init(void)
{
    memset(&g_jy61p, 0, sizeof(g_jy61p));  // 清零IMU数据结构
    g_idx = 0U;  // 重置缓冲区索引
}

/**
 * @brief 设置IMU零偏校准值
 */
void JY61P_SetZero(void)
{
    g_jy61p.roll_zero = g_jy61p.roll_deg;  // 保存当前横滚角为零偏
    g_jy61p.pitch_zero = g_jy61p.pitch_deg;  // 保存当前俯仰角为零偏
}

/**
 * @brief 获取IMU数据指针
 */
JY61P_Data_t* JY61P_GetData(void)
{
    return &g_jy61p;  // 返回IMU数据结构指针
}

/**
 * @brief 处理接收到的字节
 */
void JY61P_RxByte(uint8_t byte)
{
    if (g_idx == 0U && byte != 0x55U) return;

    g_buf[g_idx++] = byte;
    
    if (g_idx < JY61P_FRAME_LEN) return;

    g_idx = 0U;
    
    if (g_buf[0] != 0x55U || CheckSum(g_buf) != g_buf[10]) return;

    if (g_buf[1] == 0x52U)
    {
        g_jy61p.gyro_x_dps = (float)ReadS16(g_buf[2], g_buf[3]) / 32768.0f * 2000.0f;
        g_jy61p.gyro_y_dps = (float)ReadS16(g_buf[4], g_buf[5]) / 32768.0f * 2000.0f;
        g_jy61p.gyro_z_dps = (float)ReadS16(g_buf[6], g_buf[7]) / 32768.0f * 2000.0f;
        g_jy61p.update_ms = g_sys_ms;
        g_jy61p.valid = 1U;
    }
    else if (g_buf[1] == 0x53U)
    {
        g_jy61p.roll_deg  = (float)ReadS16(g_buf[2], g_buf[3]) / 32768.0f * 180.0f;
        g_jy61p.pitch_deg = (float)ReadS16(g_buf[4], g_buf[5]) / 32768.0f * 180.0f;
        g_jy61p.yaw_deg   = (float)ReadS16(g_buf[6], g_buf[7]) / 32768.0f * 180.0f;
        g_jy61p.update_ms = g_sys_ms;
        g_jy61p.valid = 1U;
    }
}
