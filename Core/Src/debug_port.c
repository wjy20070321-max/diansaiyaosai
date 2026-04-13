#include "debug_port.h"  // 调试端口模块头文件
#include <string.h>        // 标准字符串处理库

/**
 * @brief 调试信息打印函数
 * @details 该函数用于将字符串输出到调试端口，当前实现为空，可根据需要重定向到实际硬件接口
 * 
 * @param s 要打印的字符串，当前实现中该参数被忽略
 * 
 * @note 默认情况下，此函数不执行任何操作，开发者可以根据实际硬件配置
 *       将调试信息重定向到USART3或其他调试接口，便于系统调试和日志记录
 */
void DebugPort_Print(const char *s)
{
    (void)s;  // 显式忽略参数，避免编译器警告
    /* 可按需要重定向到 USART3 */
}
