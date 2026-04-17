/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序入口文件（正式闭环版）
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
/* 这些是 CubeMX 生成的底层外设头文件 */
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 这些是你自己写的应用层模块头文件 */
#include "servo.h"             // 舵机控制模块
#include "jy61p.h"             // JY61P 姿态传感器模块
#include "protocol_pi.h"       // 树莓派通信协议模块
#include "protocol_screen.h"   // 串口屏通信协议模块
#include "region.h"            // 区域管理模块
#include "task_mgr.h"          // 任务管理模块
#include "ball_outer_loop.h"   // 小球位置外环控制模块
#include "plate_inner_loop.h"  // 平台姿态内环控制模块
#include "controller_mgr.h"    // 控制器总管理模块
#include "debug_port.h"        // 调试接口模块
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 这里可以放你自己定义的结构体类型，目前为空 */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 这里可以放你自己的宏定义，目前为空 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* 这里可以放你自己的宏函数，目前为空 */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* -------------------- 系统节拍标志 -------------------- */
/* 由 TIM6 的 1ms 中断置位，在主循环中读取并清零 */
volatile uint8_t g_flag_5ms = 0;      // 5ms 控制任务触发标志
volatile uint8_t g_flag_10ms = 0;     // 10ms 控制任务触发标志

/* -------------------- 系统毫秒计数 -------------------- */
/* 每进入一次 TIM6 中断就加 1，作为整个系统的时间基准 */
volatile uint32_t g_sys_ms = 0;

/* -------------------- 调试镜像变量 -------------------- */
/* 这几个变量主要用于你在调试窗口里观察节拍是否正常 */
volatile uint32_t dbg_sys_ms = 0U;    // 调试观察用系统毫秒数
volatile uint32_t dbg_5ms_cnt = 0U;   // 调试观察用 5ms 任务触发次数
volatile uint32_t dbg_10ms_cnt = 0U;  // 调试观察用 10ms 任务触发次数

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* 这里可以放你自己的函数声明，目前为空 */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 这里可以放用户自定义的私有函数，目前为空 */
/* USER CODE END 0 */

/**
  * @brief  程序入口函数
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* 如果你有非常早期的变量初始化，可以放在这里 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* 复位所有外设，初始化 Flash 接口和 SysTick */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* 这里可以放你自己的初始化代码，目前为空 */
  /* USER CODE END Init */

  /* 配置系统时钟 */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* 这里可以放系统级初始化代码，目前为空 */
  /* USER CODE END SysInit */

  /* 初始化所有已经配置好的底层外设 */
  MX_GPIO_Init();         // GPIO 初始化
  MX_TIM2_Init();         // TIM2 初始化（当前工程里用于舵机 PWM）
  MX_TIM6_Init();         // TIM6 初始化（1ms 定时节拍）
  MX_USART1_UART_Init();  // USART1 初始化（通常接树莓派）
  MX_USART3_UART_Init();  // USART3 初始化（通常接 JY61P）
  MX_TIM12_Init();        // TIM12 初始化（如果你工程里也用到了这个定时器）
  MX_USART2_UART_Init();  // USART2 初始化（当前工程里接串口屏）

  /* USER CODE BEGIN 2 */
  /* -------------------- 初始化应用层模块 -------------------- */
  Servo_Init();             // 初始化舵机模块，启动 PWM 输出
  JY61P_Init();             // 初始化 JY61P 姿态传感器模块
  ProtocolPi_Init();        // 初始化树莓派协议解析器
  ProtocolScreen_Init();    // 初始化串口屏协议解析器
  Region_Init();            // 初始化区域中心坐标表
  TaskMgr_Init();           // 初始化任务管理器
  BallOuterLoop_Init();     // 初始化小球位置外环控制器
  PlateInnerLoop_Init();    // 初始化平台姿态内环控制器
  ControllerMgr_Init();     // 初始化总控制器上下文

  /* 启动串口中断接收 */
  /* 这样 USART 收到数据后，就会进入中断回调函数 */
  BSP_UART_StartReceiveIT();

  /* 启动 TIM6 定时中断 */
  /* TIM6 在这里被当成系统 1ms 节拍器使用 */
  HAL_TIM_Base_Start_IT(&htim6);

  /* 让舵机先回中，防止上电时平台乱动 */
  Servo_Center();

  /* 等待系统稳定 */
  /* 例如舵机归中稳定、IMU 上电稳定、串口设备进入正常工作状态 */
  HAL_Delay(1500);

  /* 把当前姿态设置为 IMU 零点 */
  JY61P_SetZero();

  /* 给串口屏发送就绪提示 */
  /* 即使没接串口屏，这句通常也不会影响系统主流程 */
  ProtocolScreen_SendText("READY\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */

    /* 统一更新所有输入数据 */
    /* 包括：
       - 树莓派视觉数据
       - IMU 数据
       - 串口屏命令
       - 任务管理器状态 */
    ControllerMgr_UpdateInputs();

    /* 5ms 控制任务 */
    /* 一般用于内环：平台姿态控制、舵机控制等更快的闭环 */
    if (g_flag_5ms)
    {
      g_flag_5ms = 0U;      // 先清标志，防止重复执行
      ControllerMgr_Run5ms();
    }

    /* 10ms 控制任务 */
    /* 一般用于外环：小球位置控制、目标点更新等 */
    if (g_flag_10ms)
    {
      g_flag_10ms = 0U;     // 先清标志，防止重复执行
      ControllerMgr_Run10ms();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 串口屏联调阶段先关闭周期状态上报，避免干扰命令解析 */

    static uint32_t screen_last_ms = 0U;
    if ((g_sys_ms - screen_last_ms) >= 100U)
     {
       screen_last_ms = g_sys_ms;
       ProtocolScreen_SendStatus();
     }

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  *
  * @note 这里将系统时钟配置到 168MHz
  *       HSI = 16MHz
  *       PLLM = 8   -> 2MHz
  *       PLLN = 168 -> 336MHz
  *       PLLP = 2   -> SYSCLK = 168MHz
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};   // 振荡器配置结构体
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};   // 时钟树配置结构体

  /** 配置主内部稳压器输出电压 */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** 配置 RCC 振荡器 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;      // 使用内部高速时钟 HSI
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                        // 开启 HSI
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // 使用默认校准值
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    // 开启 PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;            // PLL 输入源选 HSI
  RCC_OscInitStruct.PLL.PLLM = 8;                                 // PLL 输入分频
  RCC_OscInitStruct.PLL.PLLN = 168;                               // PLL 倍频系数
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     // PLL 主输出分频
  RCC_OscInitStruct.PLL.PLLQ = 4;                                 // USB 等外设分频

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** 配置 CPU、AHB 和 APB 总线时钟 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟源选 PLL
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB = SYSCLK
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1 = HCLK / 4
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2 = HCLK / 2

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief 定时器周期中断回调函数
  * @param htim 当前触发中断的定时器句柄
  *
  * @note 当前工程里主要关心 TIM6：
  *       - TIM6 每 1ms 进入一次中断
  *       - 在这里维护系统毫秒计数
  *       - 在这里驱动任务管理器 1ms 更新
  *       - 在这里产生 5ms / 10ms 的控制节拍标志
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* 静态局部变量：用于在多次中断之间保留计数值 */
  static uint8_t cnt5 = 0U;    // 5ms 分频计数器
  static uint8_t cnt10 = 0U;   // 10ms 分频计数器

  /* 只处理 TIM6 的中断 */
  if (htim->Instance == TIM6)
  {
    /* 树莓派数据快照 */
    PiRxData_t pi_snapshot;

    /* -------------------- 系统时间更新 -------------------- */
    g_sys_ms++;                // 每 1ms 加 1
    dbg_sys_ms = g_sys_ms;     // 调试镜像

    /* -------------------- 任务层 1ms 更新 -------------------- */
    /* 安全拷贝一份树莓派的小球状态 */
    ProtocolPi_CopyData(&pi_snapshot);

    /* 用当前球坐标和有效标志去推进任务状态机 */
    TaskMgr_Update1ms(pi_snapshot.ball_x_mm, pi_snapshot.ball_y_mm, pi_snapshot.ball_valid);

    /* -------------------- 产生 5ms / 10ms 节拍 -------------------- */
    cnt5++;
    cnt10++;

    /* 每 5 次 1ms 中断，置位一次 5ms 标志 */
    if (cnt5 >= 5U)
    {
      cnt5 = 0U;
      g_flag_5ms = 1U;
      dbg_5ms_cnt++;
    }

    /* 每 10 次 1ms 中断，置位一次 10ms 标志 */
    if (cnt10 >= 10U)
    {
      cnt10 = 0U;
      g_flag_10ms = 1U;
      dbg_10ms_cnt++;
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief 发生错误时执行的函数
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();   // 关闭全局中断，防止系统继续失控运行
  while (1)
  {
    /* 死循环等待人工复位或看门狗复位 */
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief 断言失败时输出源文件名和代码行号
  * @param file 出错源文件名
  * @param line 出错代码行号
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* 如果你以后想调试断言失败位置，可以在这里加串口打印 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
