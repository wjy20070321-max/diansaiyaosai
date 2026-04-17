#include "jy61p.h"    // JY61P IMU 传感器头文件
#include <string.h>   // memset() 用到的标准字符串处理库

/* -------------------- 模块内静态全局变量 -------------------- */
/* 这些变量只在 jy61p.c 内部可见，用于保存 IMU 数据和串口接收状态 */
static JY61P_Data_t g_jy61p;              // IMU 当前解析结果
static uint8_t g_buf[JY61P_FRAME_LEN];    // 接收一帧数据的缓冲区
static uint8_t g_idx = 0;                 // 当前已接收字节数 / 缓冲区写入索引

/* -------------------- 外部系统时间 -------------------- */
/* g_sys_ms 在 main.c 中定义，这里通过 extern 引用 */
extern volatile uint32_t g_sys_ms;

/**
 * @brief 安全拷贝一份当前 IMU 数据
 * @param dest 目标结构体指针
 *
 * @note 这个函数的目的是防止“数据撕裂”。
 *       因为 JY61P 数据可能在串口中断里被逐步更新，
 *       如果主循环正好在更新过程中去读，可能会读到一半旧数据、一半新数据。
 *
 *       所以这里通过：
 *       1. 先关闭全局中断
 *       2. 一次性整体复制结构体
 *       3. 再恢复中断
 *       来保证复制出来的数据是一致的。
 */
void JY61P_CopyData(JY61P_Data_t *dest)
{
    __disable_irq();     // 进入临界区：关闭中断，防止复制过程中被串口中断打断
    *dest = g_jy61p;     // 一次性整体复制当前 IMU 数据
    __enable_irq();      // 退出临界区：重新打开中断
}

/**
 * @brief 将两个 8 位字节拼成一个 16 位有符号整数
 * @param lo 低字节
 * @param hi 高字节
 * @retval 拼接后的 int16_t 值
 *
 * @note JY61P 的数据通常以“低字节在前、高字节在后”的形式发送，
 *       所以这里要先把两个字节拼接起来，再解释成带符号 16 位整数。
 */
static int16_t ReadS16(uint8_t lo, uint8_t hi)
{
    return (int16_t)((uint16_t)hi << 8 | lo);
}

/**
 * @brief 计算 JY61P 一帧数据的校验和
 * @param buf 指向 11 字节数据帧的缓冲区
 * @retval 前 10 个字节的和（低 8 位）
 *
 * @note JY61P 协议里，第 11 个字节是校验和，
 *       校验方式是前 10 个字节相加后取低 8 位。
 */
static uint8_t CheckSum(const uint8_t *buf)
{
    uint8_t i;
    uint8_t sum = 0;

    /* 对前 10 个字节做累加 */
    for (i = 0; i < 10U; i++)
    {
        sum += buf[i];
    }

    return sum;
}

/**
 * @brief JY61P 模块初始化函数
 *
 * @note 初始化时需要：
 *       1. 清空当前 IMU 数据结构体
 *       2. 清空接收状态索引
 *
 *       这样可以保证系统上电后，JY61P 数据从一个干净状态开始。
 */
void JY61P_Init(void)
{
    memset(&g_jy61p, 0, sizeof(g_jy61p)); // 清零 IMU 数据结构体
    g_idx = 0U;                           // 清零接收索引
}

/**
 * @brief 设置当前姿态为零偏
 *
 * @note 该函数通常在平台放平后调用。
 *       调用后会把当前的：
 *       - roll_deg
 *       - pitch_deg
 *       保存到：
 *       - roll_zero
 *       - pitch_zero
 *
 *       后续控制器在计算平台实际姿态时，
 *       一般会使用：
 *       当前角度 - 零偏角度
 *       这样就能以“当前水平位置”为参考零点。
 */
void JY61P_SetZero(void)
{
    g_jy61p.roll_zero = g_jy61p.roll_deg;     // 将当前横滚角保存为零偏
    g_jy61p.pitch_zero = g_jy61p.pitch_deg;   // 将当前俯仰角保存为零偏
}

/**
 * @brief 获取 IMU 全局数据结构体指针
 * @retval 指向内部 IMU 数据结构体的指针
 *
 * @note 这个接口适合想直接访问内部数据的场景。
 *       但如果你担心中断更新造成数据不一致，
 *       更推荐用 JY61P_CopyData() 先安全复制一份。
 */
JY61P_Data_t* JY61P_GetData(void)
{
    return &g_jy61p;
}

/**
 * @brief 处理串口接收到的单个字节
 * @param byte 当前收到的一个字节
 *
 * @note 这是 JY61P 协议解析的核心函数。
 *
 *       JY61P 常见数据帧格式为 11 字节：
 *       - 第 1 字节：帧头 0x55
 *       - 第 2 字节：数据类型
 *           0x51 = 加速度
 *           0x52 = 角速度
 *           0x53 = 角度
 *       - 第 3~10 字节：数据内容
 *       - 第 11 字节：校验和
 *
 *       本函数当前处理了两类帧：
 *       1. 0x52：角速度帧
 *       2. 0x53：角度帧
 */
void JY61P_RxByte(uint8_t byte)
{
    /* -------------------- 等待帧头 -------------------- */
    /* 如果当前在等待一帧的第一个字节，
       那么只有收到 0x55 才认为可能是有效帧开始 */
    if (g_idx == 0U && byte != 0x55U)
    {
        return; // 不是帧头，直接丢弃
    }

    /* 把当前字节写入缓冲区，并让索引加 1 */
    g_buf[g_idx++] = byte;

    /* 如果还没收满一整帧（11 字节），继续等待后续字节 */
    if (g_idx < JY61P_FRAME_LEN)
    {
        return;
    }

    /* -------------------- 一帧收满，准备解析 -------------------- */
    /* 收满一帧后，立即把索引清零，为下一帧做准备 */
    g_idx = 0U;

    /* 先检查：
       1. 第 1 字节是不是帧头 0x55
       2. 校验和是否正确
       任意一项不对，就直接丢弃这一帧 */
    if (g_buf[0] != 0x55U || CheckSum(g_buf) != g_buf[10])
    {
        return;
    }

    /* -------------------- 解析角速度帧（0x52） -------------------- */
    if (g_buf[1] == 0x52U)
    {
        /* 原始 16 位数据范围是 -32768 ~ 32767
           JY61P 角速度量程通常对应 ±2000 deg/s */
        g_jy61p.gyro_x_dps = (float)ReadS16(g_buf[2], g_buf[3]) / 32768.0f * 2000.0f;
        g_jy61p.gyro_y_dps = (float)ReadS16(g_buf[4], g_buf[5]) / 32768.0f * 2000.0f;
        g_jy61p.gyro_z_dps = (float)ReadS16(g_buf[6], g_buf[7]) / 32768.0f * 2000.0f;

        /* 更新数据时间戳，并标记当前 IMU 数据有效 */
        g_jy61p.update_ms = g_sys_ms;
        g_jy61p.valid = 1U;
    }
    /* -------------------- 解析角度帧（0x53） -------------------- */
    else if (g_buf[1] == 0x53U)
    {
        /* 原始 16 位数据范围是 -32768 ~ 32767
           JY61P 角度量程通常对应 ±180 deg */
        g_jy61p.roll_deg  = (float)ReadS16(g_buf[2], g_buf[3]) / 32768.0f * 180.0f;
        g_jy61p.pitch_deg = (float)ReadS16(g_buf[4], g_buf[5]) / 32768.0f * 180.0f;
        g_jy61p.yaw_deg   = (float)ReadS16(g_buf[6], g_buf[7]) / 32768.0f * 180.0f;

        /* 更新数据时间戳，并标记当前 IMU 数据有效 */
        g_jy61p.update_ms = g_sys_ms;
        g_jy61p.valid = 1U;
    }

    /* 其他类型帧（如加速度 0x51）当前没有处理，直接忽略 */
}
