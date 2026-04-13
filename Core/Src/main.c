/**
 ******************************************************************************
 * @file    main.c
 * @brief   球盘控制系统主程序
 * @details 本程序实现了一个基于STM32F407的球盘控制系统，通过定时器中断和任务管理器
 *          实现5ms和10ms周期的控制循环，控制球在平台上的运动。
 ******************************************************************************
 */

/* 包含系统头文件和外设驱动头文件 */
#include "main.h"                    // 主程序头文件，包含系统配置和全局定义
#include "gpio.h"                    // GPIO配置头文件
#include "tim.h"                     // 定时器配置头文件
#include "usart.h"                   // 串口配置头文件
#include "stm32f4xx_it.h"            // 中断处理头文件

/* 包含应用层模块头文件 */
#include "servo.h"                   // 伺服电机控制模块
#include "jy61p.h"                   // JY61P姿态传感器模块
#include "protocol_pi.h"             // 树莓派通信协议模块
#include "protocol_screen.h"         // 显示屏通信协议模块
#include "region.h"                  // 区域管理模块
#include "task_mgr.h"                // 任务管理器模块
#include "ball_outer_loop.h"         // 球位置外环控制模块
#include "plate_inner_loop.h"        // 平台姿态内环控制模块
#include "controller_mgr.h"          // 控制器管理模块
#include "debug_port.h"              // 调试端口模块

/* 全局标志变量，用于定时器中断和主循环之间的通信 */
volatile uint8_t g_flag_5ms = 0;     // 5ms周期标志位，定时器中断置1，主循环清0
volatile uint8_t g_flag_10ms = 0;    // 10ms周期标志位，定时器中断置1，主循环清0
volatile uint32_t g_sys_ms = 0;      // 系统运行时间计数器，单位：毫秒

/* 静态函数声明 */
static void SystemClock_Config(void);  // 系统时钟配置函数

/**
 * @brief  主函数
 * @note   程序入口点，负责系统初始化和主控制循环
 */
int main(void)
{
    /* HAL库初始化，必须在其他HAL函数调用前执行 */
    HAL_Init();
    
    /* 配置系统时钟，设置CPU主频为168MHz */
    SystemClock_Config();

    /* 初始化外设 */
    MX_GPIO_Init();                    // 初始化GPIO引脚
    MX_TIM2_Init();                    // 初始化定时器2（可能用于PWM输出）
    MX_TIM6_Init();                    // 初始化定时器6（用于系统时基，1ms中断）
    MX_USART1_UART_Init();             // 初始化USART1（串口1）
    MX_USART3_UART_Init();             // 初始化USART3（串口3）
    MX_USART6_UART_Init();             // 初始化USART6（串口6）

    /* 初始化应用层模块 */
    Servo_Init();                      // 伺服电机初始化
    JY61P_Init();                      // JY61P姿态传感器初始化
    ProtocolPi_Init();                 // 树莓派通信协议初始化
    ProtocolScreen_Init();             // 显示屏通信协议初始化
    Region_Init();                     // 区域管理初始化
    TaskMgr_Init();                    // 任务管理器初始化
    BallOuterLoop_Init();              // 球位置外环控制初始化
    PlateInnerLoop_Init();             // 平台姿态内环控制初始化
    ControllerMgr_Init();              // 控制器管理器初始化

    /* 启动外设功能 */
    BSP_UART_StartReceiveIT();         // 启动串口中断接收
    HAL_TIM_Base_Start_IT(&htim6);     // 启动定时器6中断（1ms周期）
    Servo_Center();                    // 将伺服电机归中（平台水平位置）

    /* 系统启动延迟和校准 */
    HAL_Delay(1500);                   // 延迟1.5秒，等待系统稳定
    JY61P_SetZero();                   // 设置JY61P传感器零点（平台水平校准）
    ProtocolScreen_SendText("READY\r\n");  // 向显示屏发送就绪信息

    /* 主控制循环 */
    while (1)
    {
        /* 更新控制器输入数据（如传感器数据、通信数据等） */
        ControllerMgr_UpdateInputs();

        /* 5ms控制周期处理 */
        if (g_flag_5ms)
        {
            g_flag_5ms = 0;            // 清除5ms标志位
            ControllerMgr_Run5ms();    // 执行5ms周期的控制任务
        }

        /* 10ms控制周期处理 */
        if (g_flag_10ms)
        {
            g_flag_10ms = 0;           // 清除10ms标志位
            ControllerMgr_Run10ms();   // 执行10ms周期的控制任务
        }
    }
}

/**
 * @brief  定时器周期结束回调函数
 * @param  htim: 定时器句柄指针
 * @note   此函数在定时器中断中被调用，用于实现系统时基和周期性任务调度
 *         定时器6配置为1ms中断，用于产生5ms和10ms的控制周期
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t cnt5 = 0;           // 5ms计数器
    static uint8_t cnt10 = 0;          // 10ms计数器

    /* 检查是否是定时器6的中断（系统时基定时器） */
    if (htim->Instance == TIM6)
    {
        /* 获取树莓派通信数据指针 */
        PiRxData_t *pi = ProtocolPi_GetData();
        
        /* 系统时间递增 */
        g_sys_ms++;
        
        /* 更新任务管理器，传入球的位置信息和有效性标志 */
        TaskMgr_Update1ms(pi->ball_x_mm, pi->ball_y_mm, pi->ball_valid);

        /* 5ms周期计数 */
        cnt5++;
        /* 10ms周期计数 */
        cnt10++;
        
        /* 检查是否达到5ms周期 */
        if (cnt5 >= 5U)
        {
            cnt5 = 0U;                 // 计数器清零
            g_flag_5ms = 1U;           // 置位5ms标志，通知主循环执行5ms任务
        }
        
        /* 检查是否达到10ms周期 */
        if (cnt10 >= 10U)
        {
            cnt10 = 0U;                // 计数器清零
            g_flag_10ms = 1U;          // 置位10ms标志，通知主循环执行10ms任务
        }
    }
}

/**
 * @brief  系统时钟配置函数
 * @note   配置STM32F407的系统时钟为168MHz
 *         使用HSI(16MHz)作为PLL输入源，通过PLL倍频到168MHz
 *         配置AHB、APB1、APB2总线时钟分频
 */
static void SystemClock_Config(void)
{
    /* 定义时钟配置结构体 */
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};    // 振荡器配置结构体
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};  // 时钟配置结构体

    /* 使能电源时钟并配置电压调节器 */
    __HAL_RCC_PWR_CLK_ENABLE();            // 使能PWR时钟
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  // 配置电压等级1（最高性能）

    /* 配置振荡器 */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;  // 使用内部高速时钟
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;       // 使能HSI
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;  // 使用默认校准值
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;   // 使能PLL
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;  // PLL输入源为HSI
    RCC_OscInitStruct.PLL.PLLM = 16;               // PLL输入分频系数 M=16 (16MHz/16=1MHz)
    RCC_OscInitStruct.PLL.PLLN = 336;              // PLL倍频系数 N=336 (1MHz*336=336MHz)
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;    // PLL输出分频系数 P=2 (336MHz/2=168MHz)
    RCC_OscInitStruct.PLL.PLLQ = 7;                // USB OTG FS、SDIO、随机数发生器时钟分频系数
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();                   // 配置失败则进入错误处理
    }

    /* 配置系统时钟 */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // 系统时钟源为PLL输出
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         // AHB时钟 = SYSCLK/1 = 168MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;          // APB1时钟 = HCLK/4 = 42MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;          // APB2时钟 = HCLK/2 = 84MHz
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();                   // 配置失败则进入错误处理
    }

    /* 配置SysTick定时器，用于HAL库的延时功能 */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000U);  // 配置1ms中断
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);  // SysTick时钟源为HCLK
}

/**
 * @brief  错误处理函数
 * @note   当系统发生严重错误时调用此函数
 *         禁用所有中断并进入死循环，防止系统继续运行
 */
void Error_Handler(void)
{
    __disable_irq();                       // 禁用全局中断
    while (1)
    {
        // 死循环，等待看门狗复位或手动复位
    }
}
