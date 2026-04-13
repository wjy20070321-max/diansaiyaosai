#include "protocol_screen.h"  // 屏幕通信协议模块头文件
#include "usart.h"             // USART外设驱动
#include <stdio.h>              // 标准输入输出
#include <string.h>             // 字符串处理

// 静态全局变量定义
static ScreenRxData_t g_screen;  // 存储解析后的数据
static char g_line[64];         // 用于存储接收到的字符行
static uint8_t g_idx = 0U;       // 当前行的索引

// 外部声明的系统毫秒计时器
extern volatile uint32_t g_sys_ms;

/**
 * @brief 解析接收到的文本行
 * @param line 指向文本行的指针
 * 
 * @note 支持的命令格式：
 *       1. "START" - 启动命令
 *       2. "STOP" - 停止命令
 *       3. "ZEROIMU" - IMU归零命令
 *       4. "TASK=<task_id>" - 任务ID设置
 *       5. "ROUTE=<a>,<b>,<c>,<d>" - 路径设置
 */
static void ProtocolScreen_ParseLine(char *line)
{
    unsigned int t = 0U;
    unsigned int a = 0U, b = 0U, c = 0U, d = 0U;

    // 处理START命令
    if (strncmp(line, "START", 5) == 0)
    {
        g_screen.start_cmd = 1U;  // 设置启动标志
        g_screen.update_ms = g_sys_ms;  // 记录更新时间
    }
    // 处理STOP命令
    else if (strncmp(line, "STOP", 4) == 0)
    {
        g_screen.stop_cmd = 1U;  // 设置停止标志
        g_screen.update_ms = g_sys_ms;  // 记录更新时间
    }
    // 处理ZEROIMU命令
    else if (strncmp(line, "ZEROIMU", 7) == 0)
    {
        g_screen.imu_zero_cmd = 1U;  // 设置IMU归零标志
        g_screen.update_ms = g_sys_ms;  // 记录更新时间
    }
    // 处理TASK命令
    else if (sscanf(line, "TASK=%u", &t) == 1)
    {
        g_screen.task_id = (uint8_t)t;  // 设置任务ID
        g_screen.update_ms = g_sys_ms;  // 记录更新时间
    }
    // 处理ROUTE命令
    else if (sscanf(line, "ROUTE=%u,%u,%u,%u", &a, &b, &c, &d) == 4)
    {
        g_screen.route_a = (uint8_t)a;  // 设置路径点A
        g_screen.route_b = (uint8_t)b;  // 设置路径点B
        g_screen.route_c = (uint8_t)c;  // 设置路径点C
        g_screen.route_d = (uint8_t)d;  // 设置路径点D
        g_screen.route_valid = 1U;  // 标记路径有效
        g_screen.update_ms = g_sys_ms;  // 记录更新时间
    }
}

/**
 * @brief 屏幕协议解析器初始化函数
 * @details 初始化协议解析器的所有状态和数据结构
 */
void ProtocolScreen_Init(void)
{
    memset(&g_screen, 0, sizeof(g_screen));  // 清零数据结构
    memset(g_line, 0, sizeof(g_line));       // 清零行缓冲区
    g_idx = 0U;  // 重置索引
}

/**
 * @brief 获取解析后的数据指针
 * @return 指向解析数据结构的指针
 */
ScreenRxData_t* ProtocolScreen_GetData(void)
{
    return &g_screen;  // 返回解析数据结构指针
}

/**
 * @brief 处理接收到的字节
 * @details 核心协议解析函数，逐字节构建文本行并解析
 * 
 * @param byte 接收到的字节
 * 
 * @note 协议特点：
 *       1. 基于文本行的协议
 *       2. 行以'\n'结束
 *       3. '\r'字符被忽略
 *       4. 每行包含一个命令
 */
void ProtocolScreen_RxByte(uint8_t byte)
{
    // 忽略回车符
    if (byte == '\r')
    {
        return;
    }

    // 处理换行符，表示一行结束
    if (byte == '\n')
    {
        g_line[g_idx] = '\0';  // 添加字符串结束符
        ProtocolScreen_ParseLine(g_line);  // 解析整行
        g_idx = 0U;  // 重置索引
        return;
    }

    // 存储有效字符
    if (g_idx < (sizeof(g_line) - 1U))
    {
        g_line[g_idx++] = (char)byte;
    }
    else
    {
        g_idx = 0U;  // 缓冲区满，重置索引
    }
}

/**
 * @brief 发送文本到屏幕
 * @param text 指向文本的指针
 * 
 * @note 使用HAL_UART_Transmit通过USART6发送数据
 * @note 发送超时设置为50毫秒
 */
void ProtocolScreen_SendText(const char *text)
{
    if (text == NULL)
    {
        return;  // 空指针检查
    }

    // 通过USART6发送文本
    HAL_UART_Transmit(&huart6, (uint8_t *)text, (uint16_t)strlen(text), 50U);
}
