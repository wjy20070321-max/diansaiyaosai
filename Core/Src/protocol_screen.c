#include "protocol_screen.h"  // 串口屏协议模块头文件
#include "usart.h"            // 串口外设句柄声明，发送数据时要用 huart2
#include "controller_mgr.h"   // 系统总上下文 g_sys，用于状态上报
#include "task_mgr.h"         // 任务管理器接口，用于读取当前任务状态
#include "region.h"           // 区域管理模块，用于判断当前球处于哪个区域
#include <stdio.h>            // snprintf() 用到的标准输入输出库
#include <string.h>           // memset() / memcpy() / strncmp() 用到的字符串处理库
#include <stdlib.h>           // strtoul() 用到的标准库函数

/* -------------------- 模块内静态全局变量 -------------------- */
/* g_screen：保存当前解析得到的串口屏命令
   g_line  ：保存当前接收中的一整行文本命令
   g_idx   ：当前行缓冲区写入位置 */
static ScreenRxData_t g_screen;
static char g_line[96];
static uint8_t g_idx = 0U;

/* -------------------- 外部系统毫秒计数 -------------------- */
/* g_sys_ms 在 main.c 中定义，这里通过 extern 引用 */
extern volatile uint32_t g_sys_ms;

/**
 * @brief 解析“逗号分隔的无符号 8 位整数列表”
 * @param s       输入字符串起始地址
 * @param out     输出数组
 * @param max_cnt 输出数组最大可写入元素个数
 * @retval 实际解析出的数字个数
 *
 * @note 主要用于解析类似：
 *       "1,2,6,9"
 *       这样的区域编号序列。
 *
 *       特点：
 *       - 自动跳过空格和逗号
 *       - 按十进制解析
 *       - 最多只解析 max_cnt 个数
 */
static uint8_t ParseU8List(const char *s, uint8_t *out, uint8_t max_cnt)
{
    uint8_t cnt = 0U;      // 当前已成功解析的数字个数
    char *endptr;          // strtoul 解析结束位置指针
    unsigned long v;       // 临时保存解析出来的数值

    /* 参数保护 */
    if (s == NULL || out == NULL || max_cnt == 0U)
    {
        return 0U;
    }

    /* 循环解析整个字符串 */
    while (*s != '\0' && cnt < max_cnt)
    {
        /* 跳过前导空格和逗号 */
        while (*s == ' ' || *s == ',')
        {
            s++;
        }

        /* 如果已经到字符串结尾，则停止 */
        if (*s == '\0')
        {
            break;
        }

        /* 解析一个十进制无符号整数 */
        v = strtoul(s, &endptr, 10);

        /* 如果没有成功解析到数字，则退出 */
        if (endptr == s)
        {
            break;
        }

        /* 保存解析结果 */
        out[cnt++] = (uint8_t)v;
        s = endptr;

        /* 跳过数字后的空格 */
        while (*s == ' ')
        {
            s++;
        }

        /* 如果后面还有逗号，则跳过 */
        if (*s == ',')
        {
            s++;
        }
    }

    return cnt;
}

/**
 * @brief 解析一整行串口屏命令
 * @param line 一行以 '\0' 结尾的字符串
 *
 * @note 当前支持的命令有：
 *       START
 *       STOP
 *       ZEROIMU
 *       TASK=n
 *       POINT=n      （n 为 1~9 的区域号）
 *       REGION=n     （兼容别名，同样是 1~9 的区域号）
 *       ROUTE=a,b,c,...
 *       PASS=a,b,c,...
 *       ROUND=a,b
 *
 *       解析成功后，会把结果写入 g_screen，
 *       并刷新 update_ms 时间戳。
 */
static void ProtocolScreen_ParseLine(char *line)
{
    unsigned int t = 0U;                 // 临时任务号变量
    unsigned int rid = 0U;               // 临时区域号变量
    uint8_t temp_route[USER_ROUTE_MAX_LEN]; // 临时路径数组
    uint8_t len = 0U;                    // 临时路径长度

    /* -------------------- START -------------------- */
    /* 启动当前任务 */
    if (strncmp(line, "START", 5) == 0)
    {
        g_screen.start_cmd = 1U;
        g_screen.update_ms = g_sys_ms;
    }
    /* -------------------- STOP -------------------- */
    /* 停止当前任务 */
    else if (strncmp(line, "STOP", 4) == 0)
    {
        g_screen.stop_cmd = 1U;
        g_screen.update_ms = g_sys_ms;
    }
    /* -------------------- ZEROIMU -------------------- */
    /* 将当前姿态设为 IMU 零点 */
    else if (strncmp(line, "ZEROIMU", 7) == 0)
    {
        g_screen.imu_zero_cmd = 1U;
        g_screen.update_ms = g_sys_ms;
    }
    /* -------------------- TASK=n -------------------- */
    /* 设置简单任务号，例如去中心、激光追踪等 */
    else if (sscanf(line, "TASK=%u", &t) == 1)
    {
        g_screen.task_id = (uint8_t)t;
        g_screen.update_ms = g_sys_ms;
    }
    /* -------------------- POINT=n -------------------- */
    /* 当前版本里 POINT=n 表示“目标区域编号”，不再是坐标 */
    else if (sscanf(line, "POINT=%u", &rid) == 1)
    {
        if (rid >= 1U && rid <= 9U)
        {
            g_screen.point_region_id = (uint8_t)rid;
            g_screen.point_valid = 1U;
            g_screen.update_ms = g_sys_ms;
        }
    }
    /* -------------------- REGION=n -------------------- */
    /* 保留 REGION=n 作为兼容写法，本质同样是区域号 1~9 */
    else if (sscanf(line, "REGION=%u", &rid) == 1)
    {
        if (rid >= 1U && rid <= 9U)
        {
            g_screen.region_id = (uint8_t)rid;
            g_screen.region_valid = 1U;
            g_screen.update_ms = g_sys_ms;
        }
    }
    /* -------------------- ROUTE=a,b,c,... -------------------- */
    /* 逐点停留模式路径任务 */
    else if (strncmp(line, "ROUTE=", 6) == 0)
    {
        memset(temp_route, 0, sizeof(temp_route));
        len = ParseU8List(line + 6, temp_route, USER_ROUTE_MAX_LEN);

        if (len > 0U)
        {
            memcpy(g_screen.route, temp_route, len); // 保存路径点
            g_screen.route_len = len;                // 保存路径长度
            g_screen.route_pass_mode = 0U;           // 0=逐点停留模式
            g_screen.route_valid = 1U;               // 标记新路径命令有效
            g_screen.update_ms = g_sys_ms;
        }
    }
    /* -------------------- PASS=a,b,c,... -------------------- */
    /* 中间经过不停留、终点停下模式 */
    else if (strncmp(line, "PASS=", 5) == 0)
    {
        memset(temp_route, 0, sizeof(temp_route));
        len = ParseU8List(line + 5, temp_route, USER_ROUTE_MAX_LEN);

        if (len > 0U)
        {
            memcpy(g_screen.route, temp_route, len); // 保存路径点
            g_screen.route_len = len;                // 保存路径长度
            g_screen.route_pass_mode = 1U;           // 1=经过不停留模式
            g_screen.route_valid = 1U;               // 标记新路径命令有效
            g_screen.update_ms = g_sys_ms;
        }
    }
    /* -------------------- ROUND=a,b -------------------- */
    /* 设置一个两点往返任务 */
    else if (strncmp(line, "ROUND=", 6) == 0)
    {
        unsigned int a = 0U, b = 0U;

        if (sscanf(line, "ROUND=%u,%u", &a, &b) == 2)
        {
            if (a >= 1U && a <= 9U && b >= 1U && b <= 9U)
            {
                g_screen.roundtrip_a = (uint8_t)a;
                g_screen.roundtrip_b = (uint8_t)b;
                g_screen.roundtrip_valid = 1U;
                g_screen.update_ms = g_sys_ms;
            }
        }
    }
}

/**
 * @brief 串口屏协议模块初始化
 *
 * @note 初始化内容包括：
 *       - 清空串口屏命令结构体 g_screen
 *       - 清空文本接收缓冲区 g_line
 *       - 清零当前接收索引 g_idx
 */
void ProtocolScreen_Init(void)
{
    memset(&g_screen, 0, sizeof(g_screen));
    memset(g_line, 0, sizeof(g_line));
    g_idx = 0U;
}

/**
 * @brief 获取当前串口屏解析结果结构体指针
 * @retval 指向内部 ScreenRxData_t 结构体的指针
 *
 * @note 控制器模块通常通过这个接口读取串口屏当前解析出的命令状态。
 */
ScreenRxData_t* ProtocolScreen_GetData(void)
{
    return &g_screen;
}

/**
 * @brief 处理串口收到的单个字节
 * @param byte 当前收到的一个字节
 *
 * @note 当前采用“按行解析”的文本协议方式：
 *       - '\r'：忽略
 *       - '\n'：表示一整条命令结束，立即解析
 *       - 其他字符：追加到当前命令行缓冲区
 */
void ProtocolScreen_RxByte(uint8_t byte)
{
    /* 忽略回车符 */
    if (byte == '\r')
    {
        return;
    }

    /* 遇到换行符，表示一整行命令接收完成 */
    if (byte == '\n')
    {
        g_line[g_idx] = '\0';             // 补上字符串结束符
        ProtocolScreen_ParseLine(g_line); // 解析这一整行命令
        g_idx = 0U;                       // 准备接收下一行
        return;
    }

    /* 普通字符写入缓冲区 */
    if (g_idx < (sizeof(g_line) - 1U))
    {
        g_line[g_idx++] = (char)byte;
    }
    else
    {
        /* 如果缓冲区写满，则丢弃这一行剩余内容并重新开始 */
        g_idx = 0U;
    }
}

/**
 * @brief 发送字符串到串口屏
 * @param text 要发送的字符串指针
 *
 * @note 当前通过 USART2 发送，
 *       与你当前工程的 usart.c 中“USART2 对接串口屏”一致。
 */
void ProtocolScreen_SendText(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 50U);
}

/**
 * @brief 主动发送当前系统状态到串口屏
 *
 * @note 当前发送内容包括：
 *       - 小球实时坐标：X / Y
 *       - 当前目标坐标：TX / TY
 *       - 当前所在区域：REG
 *       - 当前任务号：TASK
 *       - 运行/完成/失败状态：RUN / FIN / FAIL
 *       - 当前保持时间：HOLD
 *       - 当前任务总运行时间：TIME
 */
void ProtocolScreen_SendStatus(void)
{
    char buf[160];       // 状态字符串缓冲区
    TaskContext_t *ctx;  // 当前任务上下文指针
    uint8_t region_now;  // 当前球所在区域编号

    /* 获取当前任务状态 */
    ctx = TaskMgr_GetContext();

    /* 根据当前球坐标判断它位于哪个区域 */
    region_now = Region_FindCurrent(g_sys.ball.x_mm, g_sys.ball.y_mm);

    /* 组装状态字符串 */
    snprintf(buf, sizeof(buf),
             "STAT,"
             "X=%.1f,Y=%.1f,"
             "TX=%.1f,TY=%.1f,"
             "REG=%u,"
             "TASK=%u,"
             "RUN=%u,FIN=%u,FAIL=%u,"
             "HOLD=%lu,TIME=%lu\r\n",
             g_sys.ball.x_mm,
             g_sys.ball.y_mm,
             g_sys.ref.target_x_mm,
             g_sys.ref.target_y_mm,
             region_now,
             ctx->task_id,
             ctx->running,
             ctx->finished,
             ctx->failed,
             (unsigned long)ctx->hold_count_ms,
             (unsigned long)ctx->total_time_ms);

    /* 把状态发给串口屏 */
    ProtocolScreen_SendText(buf);
}
