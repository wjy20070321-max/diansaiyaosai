#ifndef __DEBUG_PORT_H__
#define __DEBUG_PORT_H__

/* -------------------- 依赖头文件 -------------------- */
#include "app_config.h"   // 系统全局配置头文件，提供基础类型和宏定义

/* -------------------- 调试输出接口 -------------------- */
/**
 * @brief 调试打印函数
 * @param s 要输出的字符串指针
 *
 * @note 这是一个统一的调试输出接口。
 *       你可以在 debug_port.c 里把它重定向到：
 *       - 串口输出
 *       - 屏幕输出
 *       - USB 虚拟串口
 *       - 其他调试接口
 *
 *       这样做的好处是：
 *       上层代码只需要调用 DebugPort_Print()，
 *       不需要关心底层到底用哪个硬件输出调试信息。
 */
void DebugPort_Print(const char *s);

#endif
