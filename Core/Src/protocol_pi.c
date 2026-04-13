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
 * @brief 原子消费任务控制命令
 * @retval 1 表示本次取到了有效命令，0 表示没有
 */
uint8_t ProtocolPi_ConsumeTaskCtrl(uint8_t *task_id, uint8_t *start_cmd, uint8_t *stop_cmd)
{
    uint8_t has_cmd = 0U;

    if (task_id == NULL || start_cmd == NULL || stop_cmd == NULL)
    {
        return 0U;
    }

    __disable_irq();

    if ((g_pi.task_id != TASK_ID_NONE) || (g_pi.start_cmd != 0U) || (g_pi.stop_cmd != 0U))
    {
        *task_id = g_pi.task_id;
        *start_cmd = g_pi.start_cmd;
        *stop_cmd = g_pi.stop_cmd;

        g_pi.task_id = TASK_ID_NONE;
        g_pi.start_cmd = 0U;
        g_pi.stop_cmd = 0U;

        has_cmd = 1U;
    }
    else
    {
        *task_id = TASK_ID_NONE;
        *start_cmd = 0U;
        *stop_cmd = 0U;
    }

    __enable_irq();
    return has_cmd;
}

/**
 * @brief 原子消费路线设置命令
 * @retval 1 表示本次取到了有效路线，0 表示没有
 */
uint8_t ProtocolPi_ConsumeRoute(uint8_t *a, uint8_t *b, uint8_t *c, uint8_t *d)
{
    uint8_t has_cmd = 0U;

    if (a == NULL || b == NULL || c == NULL || d == NULL)
    {
        return 0U;
    }

    __disable_irq();

    if (g_pi.route_a >= 1U && g_pi.route_a <= 9U)
    {
        *a = g_pi.route_a;
        *b = g_pi.route_b;
        *c = g_pi.route_c;
        *d = g_pi.route_d;

        g_pi.route_a = 0U;
        g_pi.route_b = 0U;
        g_pi.route_c = 0U;
        g_pi.route_d = 0U;

        has_cmd = 1U;
    }
    else
    {
        *a = 0U;
        *b = 0U;
        *c = 0U;
        *d = 0U;
    }

    __enable_irq();
    return has_cmd;
}

/**
 * @brief 原子消费目标设置命令
 * @retval 1 表示本次取到了有效目标，0 表示没有
 */
uint8_t ProtocolPi_ConsumeTarget(float *x_mm, float *y_mm)
{
    uint8_t has_cmd = 0U;

    if (x_mm == NULL || y_mm == NULL)
    {
        return 0U;
    }

    __disable_irq();

    if (g_pi.target_valid)
    {
        *x_mm = g_pi.target_x_mm;
        *y_mm = g_pi.target_y_mm;

        g_pi.target_x_mm = 0.0f;
        g_pi.target_y_mm = 0.0f;
        g_pi.target_valid = 0U;

        has_cmd = 1U;
    }
    else
    {
        *x_mm = 0.0f;
        *y_mm = 0.0f;
    }

    __enable_irq();
    return has_cmd;
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
            /* 任意有效帧都刷新总更新时间 */
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
            /* 激光追踪帧 */
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
