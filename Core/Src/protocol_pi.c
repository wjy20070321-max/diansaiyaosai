#include "protocol_pi.h"
#include "app_config.h"
#include <string.h>

/* 静态全局变量定义 */
static PiRxData_t g_pi;
static uint8_t state = 0U;
static uint8_t cmd = 0U;
static uint8_t len = 0U;
static uint8_t idx = 0U;
static uint8_t buf[PI_FRAME_MAX_LEN];

/* 外部声明的系统毫秒计时器 */
extern volatile uint32_t g_sys_ms;

/**
 * @brief 安全获取树莓派数据（防止数据撕裂）
 */
void ProtocolPi_CopyData(PiRxData_t *dest)
{
    if (dest == NULL)
    {
        return;
    }

    __disable_irq();
    *dest = g_pi;
    __enable_irq();
}

/**
 * @brief 清除任务控制命令（一次性消费）
 */
void ProtocolPi_ClearTaskCtrl(void)
{
    __disable_irq();
    g_pi.task_id = TASK_ID_NONE;
    g_pi.start_cmd = 0U;
    g_pi.stop_cmd = 0U;
    __enable_irq();
}

/**
 * @brief 清除路径设置命令（一次性消费）
 */
void ProtocolPi_ClearRoute(void)
{
    __disable_irq();
    g_pi.route_a = 0U;
    g_pi.route_b = 0U;
    g_pi.route_c = 0U;
    g_pi.route_d = 0U;
    __enable_irq();
}

/**
 * @brief 清除目标设置命令（一次性消费）
 */
void ProtocolPi_ClearTarget(void)
{
    __disable_irq();
    g_pi.target_x_mm = 0.0f;
    g_pi.target_y_mm = 0.0f;
    g_pi.target_valid = 0U;
    __enable_irq();
}

/**
 * @brief 计算XOR校验和
 */
static uint8_t XorSum(const uint8_t *data, uint8_t n)
{
    uint8_t i;
    uint8_t s = 0U;

    for (i = 0U; i < n; i++)
    {
        s ^= data[i];
    }

    return s;
}

/**
 * @brief PI协议解析器初始化函数
 */
void ProtocolPi_Init(void)
{
    memset(&g_pi, 0, sizeof(g_pi));
    state = 0U;
    cmd = 0U;
    len = 0U;
    idx = 0U;
}

/**
 * @brief 获取解析后的数据指针
 */
PiRxData_t* ProtocolPi_GetData(void)
{
    return &g_pi;
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
        if (byte == 0xAAU)
        {
            state = 1U;
        }
        break;

    case 1:
        if (byte == 0x55U)
        {
            state = 2U;
        }
        else
        {
            state = 0U;
        }
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
        if (idx >= len)
        {
            state = 5U;
        }
        break;

    case 5:
        sum = (uint8_t)(cmd ^ len ^ XorSum(buf, len));
        if (sum == byte)
        {
            /* 任意有效帧都刷新总更新时间（兼容旧逻辑） */
            g_pi.update_ms = g_sys_ms;

            /* 球状态帧 */
            if (cmd == PI_CMD_BALL_STATE && len == 17U)
            {
                memcpy(&g_pi.ball_x_mm,     &buf[0],  4);
                memcpy(&g_pi.ball_y_mm,     &buf[4],  4);
                memcpy(&g_pi.ball_vx_mmps,  &buf[8],  4);
                memcpy(&g_pi.ball_vy_mmps,  &buf[12], 4);
                g_pi.ball_valid = buf[16];

                g_pi.ball_update_ms = g_sys_ms;
            }
            /* 任务控制帧 */
            else if (cmd == PI_CMD_TASK_CTRL && len == 3U)
            {
                g_pi.task_id = buf[0];
                g_pi.start_cmd = buf[1];
                g_pi.stop_cmd = buf[2];

                g_pi.cmd_update_ms = g_sys_ms;
            }
            /* 路径设置帧 */
            else if (cmd == PI_CMD_SET_ROUTE && len == 4U)
            {
                g_pi.route_a = buf[0];
                g_pi.route_b = buf[1];
                g_pi.route_c = buf[2];
                g_pi.route_d = buf[3];

                g_pi.cmd_update_ms = g_sys_ms;
            }
            /* 目标位置帧 */
            else if (cmd == PI_CMD_SET_TARGET && len == 8U)
            {
                memcpy(&g_pi.target_x_mm, &buf[0], 4);
                memcpy(&g_pi.target_y_mm, &buf[4], 4);
                g_pi.target_valid = 1U;

                g_pi.cmd_update_ms = g_sys_ms;
            }
            /* 激光追踪帧：4 + 4 + 1 = 9字节 */
            else if (cmd == PI_CMD_LASER_TRACK && len == 9U)
            {
                memcpy(&g_pi.laser_x_mm, &buf[0], 4);
                memcpy(&g_pi.laser_y_mm, &buf[4], 4);
                g_pi.laser_valid = buf[8];

                g_pi.laser_update_ms = g_sys_ms;
            }
        }

        state = 0U;
        break;

    default:
        state = 0U;
        break;
    }
}
