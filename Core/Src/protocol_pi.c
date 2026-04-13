#include "protocol_pi.h"  // PI通信协议解析模块头文件
#include "app_config.h"   // 包含 PI_CMD_LASER_TRACK 宏定义
#include <string.h>

// 静态全局变量定义
static PiRxData_t g_pi;  // 存储解析后的数据
static uint8_t state = 0U;  // 状态机当前状态
static uint8_t cmd = 0U;  // 当前命令
static uint8_t len = 0U;  // 数据长度
static uint8_t idx = 0U;  // 数据索引
static uint8_t buf[PI_FRAME_MAX_LEN];  // 数据缓冲区

// 外部声明的系统毫秒计时器
extern volatile uint32_t g_sys_ms;

/**
 * @brief 安全获取树莓派数据（防止数据撕裂）
 * @param dest 目标结构体指针
 */
void ProtocolPi_CopyData(PiRxData_t *dest)
{
    __disable_irq();        // 进入临界区
    *dest = g_pi;           // 【修复】原子拷贝，使用正确的真实变量名
    __enable_irq();         // 退出临界区
}

/**
 * @brief 计算XOR校验和
 */
static uint8_t XorSum(const uint8_t *data, uint8_t n)
{
    uint8_t i;
    uint8_t s = 0;
    for (i = 0U; i < n; i++) s ^= data[i];
    return s;
}

/**
 * @brief PI协议解析器初始化函数
 */
void ProtocolPi_Init(void)
{
    memset(&g_pi, 0, sizeof(g_pi));  // 清零数据结构
    state = 0U;  // 重置状态机
    cmd = 0U;    // 重置命令
    len = 0U;    // 重置长度
    idx = 0U;    // 重置索引
}

/**
 * @brief 获取解析后的数据指针
 */
PiRxData_t* ProtocolPi_GetData(void)
{
    return &g_pi;  // 返回解析数据结构指针
}

/**
 * @brief 处理接收到的字节
 */
void ProtocolPi_RxByte(uint8_t byte)
{
    uint8_t sum;

    switch (state)
    {
    case 0:
        if (byte == 0xAAU) state = 1U;
        break;
    case 1:
        if (byte == 0x55U) state = 2U;
        else state = 0U;
        break;
    case 2:
        cmd = byte;
        state = 3U;
        break;
    case 3:
        len = byte;
        if (len > PI_FRAME_MAX_LEN)
        {
            state = 0U;
            break;
        }
        idx = 0U;
        state = 4U;
        break;
    case 4:
        buf[idx++] = byte;
        if (idx >= len) state = 5U;
        break;
    case 5:
        sum = cmd ^ len ^ XorSum(buf, len);
        if (sum == byte)
        {
            // 解析球状态数据
            if (cmd == PI_CMD_BALL_STATE && len == 17U)
            {
                memcpy(&g_pi.ball_x_mm, &buf[0], 4);
                memcpy(&g_pi.ball_y_mm, &buf[4], 4);
                memcpy(&g_pi.ball_vx_mmps, &buf[8], 4);
                memcpy(&g_pi.ball_vy_mmps, &buf[12], 4);
                g_pi.ball_valid = buf[16];
                g_pi.update_ms = g_sys_ms;
            }
            // 解析任务控制命令
            else if (cmd == PI_CMD_TASK_CTRL && len == 3U)
            {
                g_pi.task_id = buf[0];
                g_pi.start_cmd = buf[1];
                g_pi.stop_cmd = buf[2];
                g_pi.update_ms = g_sys_ms;
            }
            // 解析路径设置
            else if (cmd == PI_CMD_SET_ROUTE && len == 4U)
            {
                g_pi.route_a = buf[0];
                g_pi.route_b = buf[1];
                g_pi.route_c = buf[2];
                g_pi.route_d = buf[3];
            }
            // 解析目标位置
            else if (cmd == PI_CMD_SET_TARGET && len == 8U)
            {
                memcpy(&g_pi.target_x_mm, &buf[0], 4);
                memcpy(&g_pi.target_y_mm, &buf[4], 4);
                g_pi.target_valid = 1U;
            }
            // 【新增】解析激光坐标数据！(长度为 4+4+1 = 9字节)
            else if (cmd == PI_CMD_LASER_TRACK && len == 9U)
            {
                memcpy(&g_pi.laser_x_mm, &buf[0], 4);
                memcpy(&g_pi.laser_y_mm, &buf[4], 4);
                g_pi.laser_valid = buf[8];
                g_pi.update_ms = g_sys_ms;
            }
        }
        state = 0U;
        break;
    default:
        state = 0U;
        break;
    }
}
