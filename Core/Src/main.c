/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序入口文件
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
#include "region.h"            // 区域坐标管理模块
#include "task_mgr.h"          // 任务管理模块
#include "ball_outer_loop.h"   // 小球位置外环控制模块
#include "plate_inner_loop.h"  // 平台姿态内环控制模块
#include "controller_mgr.h"    // 控制器总管理模块
#include "debug_port.h"        // 调试接口模块
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 这里可以放你自己的结构体类型定义，目前为空 */
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
volatile uint8_t g_flag_5ms = 0;     // 5ms 控制任务触发标志
volatile uint8_t g_flag_10ms = 0;    // 10ms 控制任务触发标志

/* -------------------- 系统毫秒计数 -------------------- */
/* 每进入一次 TIM6 中断就加 1，用作系统运行时间基准 */
volatile uint32_t g_sys_ms = 0;

/* -------------------- 调试用计数变量 -------------------- */
/* 这些变量主要用于你观察系统节拍是否正常 */
volatile uint32_t dbg_sys_ms = 0U;   // 调试观察用系统毫秒计数
volatile uint32_t dbg_5ms_cnt = 0U;  // 调试观察用 5ms 任务触发次数
volatile uint32_t dbg_10ms_cnt = 0U; // 调试观察用 10ms 任务触发次数
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* 系统时钟配置函数声明 */
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
  /* 在 HAL_Init() 前，如果你有特别早期的变量初始化，可放这里 */
  /* USER CODE END 1 */

  /* HAL 库初始化
     作用：
     1. 初始化 Flash 接口
     2. 初始化 SysTick
     3. 初始化 HAL 底层状态 */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* 这里可以放你自己的初始化代码，目前为空 */
  /* USER CODE END Init */

  /* 配置系统时钟 */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* 这里可以放系统级初始化代码，目前为空 */
  /* USER CODE END SysInit */

  /* -------------------- 初始化底层外设 -------------------- */
  MX_GPIO_Init();       // 初始化 GPIO
  MX_TIM2_Init();       // 初始化 TIM2（项目里保留的定时器）
  MX_TIM6_Init();       // 初始化 TIM6（1ms 定时节拍）
  MX_TIM12_Init();      // 初始化 TIM12（舵机 PWM 输出）
  MX_USART1_UART_Init();// 初始化 USART1（通常接树莓派）
  MX_USART3_UART_Init();// 初始化 USART3（通常接 JY61P）
  MX_USART6_UART_Init();// 初始化 USART6（通常接串口屏）

  /* USER CODE BEGIN 2 */
  /* -------------------- 初始化应用层模块 -------------------- */
  Servo_Init();             // 初始化舵机模块，启动 PWM 输出
  JY61P_Init();             // 初始化 JY61P 姿态传感器数据结构
  ProtocolPi_Init();        // 初始化树莓派通信协议解析器
  ProtocolScreen_Init();    // 初始化串口屏协议解析器
  Region_Init();            // 初始化区域坐标表
  TaskMgr_Init();           // 初始化任务管理器
  BallOuterLoop_Init();     // 初始化小球位置外环控制器
  PlateInnerLoop_Init();    // 初始化平台姿态内环控制器
  ControllerMgr_Init();     // 初始化总控制器上下文

  /* 启动串口中断接收
     这样 USART1 / USART3 / USART6 收到数据时会自动进入中断回调 */
  BSP_UART_StartReceiveIT();

  /* 启动 TIM6 的定时中断
     TIM6 在这里作为 1ms 的系统节拍器 */
  HAL_TIM_Base_Start_IT(&htim6);

  /* 上电后先让舵机回中，避免平台乱动 */
  Servo_Center();

  /* 等待系统稳定
     例如舵机稳定、IMU 上电稳定、串口设备进入正常状态 */
  HAL_Delay(1500);

  /* 以当前姿态作为 IMU 零点 */
  JY61P_SetZero();

  /* 给串口屏发送就绪提示
     即便没接串口屏，这句也通常不会影响主流程 */
  ProtocolScreen_SendText("READY\r\n");
  /* USER CODE END 2 */

  /* -------------------- 主循环 -------------------- */
  while (1)
  {
    /* USER CODE BEGIN WHILE */

    /* 先统一更新各类输入数据：
       - 树莓派视觉数据
       - IMU 数据
       - 串口屏命令
       - 任务状态 */
    ControllerMgr_UpdateInputs();

    /* 5ms 控制任务
       一般用于内环：平台姿态控制、舵机控制等更快的闭环 */
    if (g_flag_5ms)
    {
      g_flag_5ms = 0U;      // 先清标志，避免重复执行
      ControllerMgr_Run5ms();
    }

    /* 10ms 控制任务
       一般用于外环：小球位置控制、任务目标更新等 */
    if (g_flag_10ms)
    {
      g_flag_10ms = 0U;     // 先清标志，避免重复执行
      ControllerMgr_Run10ms();
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  * @note 这里把系统时钟配置到 168MHz
  *       HSI = 16MHz
  *       PLLM = 8  -> 2MHz
  *       PLLN = 168 -> 336MHz
  *       PLLP = 2  -> SYSCLK = 168MHz
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; // 振荡器配置结构体
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; // 时钟树配置结构体

  /* 使能电源控制时钟 */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* 配置电压调节器输出等级
     168MHz 运行通常需要 SCALE1 */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* -------------------- 配置主振荡器与 PLL -------------------- */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // 使用内部高速时钟 HSI
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                   // 开启 HSI
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // 使用默认校准值
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // 开启 PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;       // PLL 输入源选择 HSI
  RCC_OscInitStruct.PLL.PLLM = 8;                            // PLL 输入分频
  RCC_OscInitStruct.PLL.PLLN = 168;                          // PLL 倍频系数
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                // PLL 主输出分频
  RCC_OscInitStruct.PLL.PLLQ = 4;                            // USB 等外设时钟分频

  /* 如果时钟配置失败，进入错误处理 */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* -------------------- 配置总线时钟 -------------------- */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // 系统时钟源选 PLL
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        // AHB = SYSCLK
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;         // APB1 = HCLK/4
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;         // APB2 = HCLK/2

  /* 配置时钟树并设置 Flash 等待周期 */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief 定时器周期到达回调函数
  * @param htim: 当前触发中断的定时器句柄
  *
  * @note 本项目中主要关心 TIM6：
  *       - TIM6 每 1ms 进入一次中断
  *       - 在这里维护系统毫秒计数
  *       - 在这里驱动任务管理器 1ms 更新
  *       - 在这里产生 5ms / 10ms 的控制节拍标志
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* 静态局部变量：用于在多次中断之间保留计数值 */
  static uint8_t cnt5 = 0U;   // 5ms 分频计数器
  static uint8_t cnt10 = 0U;  // 10ms 分频计数器

  /* 只处理 TIM6 的中断 */
  if (htim->Instance == TIM6)
  {
    /* 临时快照变量
       用来拷贝当前树莓派发来的球位置信息 */
    PiRxData_t pi_snapshot;

    /* -------------------- 系统时间更新 -------------------- */
    g_sys_ms++;               // 每 1ms 加 1
    dbg_sys_ms = g_sys_ms;    // 调试观察用镜像变量

    /* -------------------- 任务层 1ms 更新 -------------------- */
    /* 先安全拷贝一份树莓派视觉数据 */
    ProtocolPi_CopyData(&pi_snapshot);

    /* 将当前小球位置和有效状态送入任务管理器
       任务管理器会判断：
       - 是否到达目标
       - 是否需要切换下一步
       - 是否超时失败 */
    TaskMgr_Update1ms(pi_snapshot.ball_x_mm, pi_snapshot.ball_y_mm, pi_snapshot.ball_valid);

    /* -------------------- 产生 5ms / 10ms 节拍 -------------------- */
    cnt5++;
    cnt10++;

    /* 每 5 次 1ms 中断，置位一次 5ms 标志 */
    if (cnt5 >= 5U)
    {
      cnt5 = 0U;
      g_flag_5ms = 1U;
      dbg_5ms_cnt++; // 调试统计 5ms 任务执行次数
    }

    /* 每 10 次 1ms 中断，置位一次 10ms 标志 */
    if (cnt10 >= 10U)
    {
      cnt10 = 0U;
      g_flag_10ms = 1U;
      dbg_10ms_cnt++; // 调试统计 10ms 任务执行次数
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  错误处理函数
  * @retval None
  *
  * @note 当系统发生严重错误时，会关闭中断并停在死循环里
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
  * @brief  断言失败回调
  * @param  file: 出错源文件名
  * @param  line: 出错代码行号
  * @retval None
  *
  * @note 只有启用了 USE_FULL_ASSERT 才会编译进来
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* 这里可以加串口打印，用于输出断言失败位置 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
