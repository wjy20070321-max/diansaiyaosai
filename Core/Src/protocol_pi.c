#include "protocol_pi.h"   // 树莓派通信协议模块头文件
#include "app_config.h"    // 系统全局配置头文件，包含命令码、帧长度等宏定义
#include <string.h>        // memcpy / memset 用到的标准字符串处理库

/* -------------------- 模块内静态全局变量 -------------------- */
/* 这些变量只在本文件内部可见，用于保存协议解析状态和解析结果 */
static PiRxData_t g_pi;                 // 当前解析得到的树莓派数据总结构体
static uint8_t state = 0U;              // 协议解析状态机当前状态
static uint8_t cmd = 0U;                // 当前帧命令字
static uint8_t len = 0U;                // 当前帧数据长度
static uint8_t idx = 0U;                // 当前已接收 payload 字节数
static uint8_t buf[PI_FRAME_MAX_LEN];   // payload 数据缓存区

/* -------------------- 外部系统毫秒计数 -------------------- */
/* g_sys_ms 在 main.c 中定义，这里通过 extern 引用 */
extern volatile uint32_t g_sys_ms;

/**
 * @brief 安全复制一份当前树莓派数据
 * @param dest 目标结构体指针
 *
 * @note 因为 g_pi 可能会在串口接收中断中被更新，
 *       如果主循环直接访问，有机会读到“半新半旧”的数据。
 *       所以这里通过：
 *       1. 关闭中断
 *       2. 一次性整体拷贝结构体
 *       3. 恢复中断
 *       来保证读出的数据是一致的。
 */
void ProtocolPi_CopyData(PiRxData_t *dest)
{
    /* 先做空指针保护，防止误传空地址 */
    if (dest == NULL)
    {
        return;
    }

    /* 进入临界区，防止复制过程中串口中断修改 g_pi */
    __disable_irq();
    *dest = g_pi;      // 一次性整体复制树莓派数据结构体
    __enable_irq();
}

/**
 * @brief 原子读取并清空任务控制命令
 * @param task_id   返回任务 ID
 * @param start_cmd 返回启动命令
 * @param stop_cmd  返回停止命令
 * @retval 1=本次确实取到了有效命令，0=没有新命令
 *
 * @note 这里采用“原子消费”的思想：
 *       一旦读到命令，就立刻把内部保存的命令清空。
 *       这样可以避免：
 *       - 一条命令被主循环重复执行多次
 *       - 旧命令残留影响后续任务
 */
uint8_t ProtocolPi_ConsumeTaskCtrl(uint8_t *task_id, uint8_t *start_cmd, uint8_t *stop_cmd)
{
    uint8_t has_cmd = 0U; // 标记本次是否成功取到了有效命令

    /* 参数保护：任何一个返回指针为空，都直接失败返回 */
    if (task_id == NULL || start_cmd == NULL || stop_cmd == NULL)
    {
        return 0U;
    }

    /* 进入临界区，保证读取和清空过程是原子的 */
    __disable_irq();

    /* 只要任务号 / 启动 / 停止 中任意一项非空，就认为有新命令 */
    if ((g_pi.task_id != TASK_ID_NONE) || (g_pi.start_cmd != 0U) || (g_pi.stop_cmd != 0U))
    {
        /* 先把当前命令内容拷贝给调用者 */
        *task_id = g_pi.task_id;
        *start_cmd = g_pi.start_cmd;
        *stop_cmd = g_pi.stop_cmd;

        /* 再把内部命令清空，避免重复执行 */
        g_pi.task_id = TASK_ID_NONE;
        g_pi.start_cmd = 0U;
        g_pi.stop_cmd = 0U;

        has_cmd = 1U; // 表示这次确实读到了新命令
    }
    else
    {
        /* 如果没有新命令，则返回默认空值 */
        *task_id = TASK_ID_NONE;
        *start_cmd = 0U;
        *stop_cmd = 0U;
    }

    __enable_irq();
    return has_cmd;
}

/**
 * @brief 原子读取并清空路径设置命令
 * @param a 返回路径点 A
 * @param b 返回路径点 B
 * @param c 返回路径点 C
 * @param d 返回路径点 D
 * @retval 1=本次取到了有效路线，0=没有新路线
 *
 * @note 当前这里把“route_a 合法”作为路线有效的判据。
 *       读取成功后，会立刻把 route_a~route_d 清空。
 */
uint8_t ProtocolPi_ConsumeRoute(uint8_t *a, uint8_t *b, uint8_t *c, uint8_t *d)
{
    uint8_t has_cmd = 0U; // 标记本次是否成功取到新路线

    /* 参数保护 */
    if (a == NULL || b == NULL || c == NULL || d == NULL)
    {
        return 0U;
    }

    /* 进入临界区 */
    __disable_irq();

    /* 如果第一个区域点是合法的 1~9 号区，则认为收到了一条有效路线 */
    if (g_pi.route_a >= 1U && g_pi.route_a <= 9U)
    {
        /* 先返回当前路线 */
        *a = g_pi.route_a;
        *b = g_pi.route_b;
        *c = g_pi.route_c;
        *d = g_pi.route_d;

        /* 再把内部路线清空 */
        g_pi.route_a = 0U;
        g_pi.route_b = 0U;
        g_pi.route_c = 0U;
        g_pi.route_d = 0U;

        has_cmd = 1U;
    }
    else
    {
        /* 没有有效路线则返回 0 */
        *a = 0U;
        *b = 0U;
        *c = 0U;
        *d = 0U;
    }

    __enable_irq();
    return has_cmd;
}

/**
 * @brief 原子读取并清空目标点命令
 * @param x_mm 返回目标点 X 坐标
 * @param y_mm 返回目标点 Y 坐标
 * @retval 1=本次取到了有效目标点，0=没有新目标点
 *
 * @note 当 target_valid = 1 时，表示树莓派刚下发了一个新的直接目标点。
 *       读取成功后会把：
 *       - target_x_mm
 *       - target_y_mm
 *       - target_valid
 *       全部清空。
 */
uint8_t ProtocolPi_ConsumeTarget(float *x_mm, float *y_mm)
{
    uint8_t has_cmd = 0U; // 标记本次是否成功取到新目标点

    /* 参数保护 */
    if (x_mm == NULL || y_mm == NULL)
    {
        return 0U;
    }

    /* 进入临界区 */
    __disable_irq();

    /* 如果目标点有效，则返回并清空 */
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
        /* 没有新目标点则返回 0 值 */
        *x_mm = 0.0f;
        *y_mm = 0.0f;
    }

    __enable_irq();
    return has_cmd;
}

/**
 * @brief 计算 XOR 异或校验和
 * @param data 数据区指针
 * @param n    数据长度
 * @retval 所有字节按位异或后的结果
 *
 * @note 本协议使用的是简单 XOR 校验：
 *       checksum = cmd ^ len ^ payload[0] ^ payload[1] ^ ...
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
 * @brief PI 协议解析器初始化函数
 *
 * @note 初始化内容包括：
 *       - 清空解析结果结构体 g_pi
 *       - 重置协议状态机
 *       - 清空当前命令字、长度和索引
 */
void ProtocolPi_Init(void)
{
    memset(&g_pi, 0, sizeof(g_pi)); // 清空所有解析结果数据
    state = 0U;                     // 状态机回到等待帧头状态
    cmd = 0U;                       // 当前命令字清零
    len = 0U;                       // 当前数据长度清零
    idx = 0U;                       // 当前 payload 索引清零
}

/**
 * @brief 获取当前解析结果结构体指针
 * @retval 指向内部 PiRxData_t 结构体的指针
 *
 * @note 如果你只是想安全拿一份当前数据快照，
 *       更建议优先使用 ProtocolPi_CopyData()。
 */
PiRxData_t* ProtocolPi_GetData(void)
{
    return &g_pi;
}

/**
 * @brief 处理接收到的单个字节
 * @param byte 当前收到的一个字节
 *
 * @note 这是树莓派协议解析的核心函数，采用状态机方式逐字节解析。
 *
 *       当前协议格式为：
 *       [0] 0xAA       帧头1
 *       [1] 0x55       帧头2
 *       [2] cmd        命令字
 *       [3] len        payload 长度
 *       [4..] payload  数据区
 *       [end] checksum 校验字节
 *
 *       注意：
 *       - 本协议没有帧尾
 *       - 是通过“帧头 + 长度 + 校验”来确定一帧
 */
void ProtocolPi_RxByte(uint8_t byte)
{
    uint8_t sum; // 当前帧计算得到的校验值

    /* -------------------- 协议状态机 -------------------- */
    switch (state)
    {
    /* 状态 0：等待第一个帧头 0xAA */
    case 0:
        if (byte == 0xAAU)
        {
            state = 1U; // 收到第一个帧头，进入下一状态
        }
        break;

    /* 状态 1：等待第二个帧头 0x55 */
    case 1:
        if (byte == 0x55U)
        {
            state = 2U; // 帧头完整，准备接收命令字
        }
        else
        {
            state = 0U; // 第二个帧头不对，重新从头找帧头
        }
        break;

    /* 状态 2：接收命令字 cmd */
    case 2:
        cmd = byte;
        state = 3U;
        break;

    /* 状态 3：接收 payload 长度 len */
    case 3:
        len = byte;

        /* 如果长度超过协议允许的最大长度，则认为这是错误帧，丢弃 */
        if (len > PI_FRAME_MAX_LEN)
        {
            state = 0U;
            break;
        }

        idx = 0U;   // 清零 payload 索引
        state = 4U; // 进入接收 payload 状态
        break;

    /* 状态 4：逐字节接收 payload */
    case 4:
        buf[idx++] = byte;

        /* 当收满 len 个 payload 字节后，进入校验字节状态 */
        if (idx >= len)
        {
            state = 5U;
        }
        break;

    /* 状态 5：接收并校验 checksum */
    case 5:
        /* 计算本帧理论校验值：
           checksum = cmd ^ len ^ payload_xor */
        sum = (uint8_t)(cmd ^ len ^ XorSum(buf, len));

        /* 如果校验通过，则解析当前帧 */
        if (sum == byte)
        {
            /* 任何有效帧都会刷新“最近一次收到任意 PI 帧”的总时间 */
            g_pi.update_ms = g_sys_ms;

            /* -------------------- 球状态帧 -------------------- */
            /* payload 长度：17 字节
               结构：
               [0..3]   ball_x_mm
               [4..7]   ball_y_mm
               [8..11]  ball_vx_mmps
               [12..15] ball_vy_mmps
               [16]     ball_valid */
            if (cmd == PI_CMD_BALL_STATE && len == 17U)
            {
                memcpy(&g_pi.ball_x_mm,     &buf[0],  4);
                memcpy(&g_pi.ball_y_mm,     &buf[4],  4);
                memcpy(&g_pi.ball_vx_mmps,  &buf[8],  4);
                memcpy(&g_pi.ball_vy_mmps,  &buf[12], 4);
                g_pi.ball_valid = buf[16];

                /* 单独刷新球状态时间戳 */
                g_pi.ball_update_ms = g_sys_ms;
            }
            /* -------------------- 任务控制帧 -------------------- */
            /* payload 长度：3 字节
               [0] task_id
               [1] start_cmd
               [2] stop_cmd */
            else if (cmd == PI_CMD_TASK_CTRL && len == 3U)
            {
                g_pi.task_id = buf[0];
                g_pi.start_cmd = buf[1];
                g_pi.stop_cmd = buf[2];

                /* 刷新命令类时间戳 */
                g_pi.cmd_update_ms = g_sys_ms;
            }
            /* -------------------- 路径设置帧 -------------------- */
            /* payload 长度：4 字节
               [0] route_a
               [1] route_b
               [2] route_c
               [3] route_d */
            else if (cmd == PI_CMD_SET_ROUTE && len == 4U)
            {
                g_pi.route_a = buf[0];
                g_pi.route_b = buf[1];
                g_pi.route_c = buf[2];
                g_pi.route_d = buf[3];

                g_pi.cmd_update_ms = g_sys_ms;
            }
            /* -------------------- 目标点设置帧 -------------------- */
            /* payload 长度：8 字节
               [0..3] target_x_mm
               [4..7] target_y_mm */
            else if (cmd == PI_CMD_SET_TARGET && len == 8U)
            {
                memcpy(&g_pi.target_x_mm, &buf[0], 4);
                memcpy(&g_pi.target_y_mm, &buf[4], 4);
                g_pi.target_valid = 1U;

                g_pi.cmd_update_ms = g_sys_ms;
            }
            /* -------------------- 激光追踪帧 -------------------- */
            /* payload 长度：9 字节
               [0..3] laser_x_mm
               [4..7] laser_y_mm
               [8]    laser_valid */
            else if (cmd == PI_CMD_LASER_TRACK && len == 9U)
            {
                memcpy(&g_pi.laser_x_mm, &buf[0], 4);
                memcpy(&g_pi.laser_y_mm, &buf[4], 4);
                g_pi.laser_valid = buf[8];

                g_pi.laser_update_ms = g_sys_ms;
            }
        }

        /* 不管当前帧是否校验成功，解析完这一轮都回到初始状态，等待下一帧 */
        state = 0U;
        break;

    /* 任何异常状态都直接重置回初始状态 */
    default:
        state = 0U;
        break;
    }
}
