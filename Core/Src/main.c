/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "servo.h"
#include "jy61p.h"
#include "protocol_pi.h"
#include "protocol_screen.h"
#include "region.h"
#include "task_mgr.h"
#include "ball_outer_loop.h"
#include "plate_inner_loop.h"
#include "controller_mgr.h"
#include "debug_port.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t g_flag_5ms = 0;
volatile uint8_t g_flag_10ms = 0;
volatile uint32_t g_sys_ms = 0;

volatile uint32_t dbg_sys_ms = 0U;
volatile uint32_t dbg_5ms_cnt = 0U;
volatile uint32_t dbg_10ms_cnt = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* 初始化底层外设 */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */
  /* 初始化应用层模块 */
  Servo_Init();
  JY61P_Init();
  ProtocolPi_Init();
  ProtocolScreen_Init();
  Region_Init();
  TaskMgr_Init();
  BallOuterLoop_Init();
  PlateInnerLoop_Init();
  ControllerMgr_Init();

  /* 启动串口中断接收 */
  BSP_UART_StartReceiveIT();

  /* 启动 1ms 定时节拍 */
  HAL_TIM_Base_Start_IT(&htim6);

  /* 舵机归中 */
  Servo_Center();

  /* 等系统稳定 */
  HAL_Delay(1500);

  /* IMU 置零 */
  JY61P_SetZero();

  /* 没有串口屏也没关系，这句保留不会影响调试 */
  ProtocolScreen_SendText("READY\r\n");
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN WHILE */
    ControllerMgr_UpdateInputs();

    if (g_flag_5ms)
    {
      g_flag_5ms = 0U;
      ControllerMgr_Run5ms();
    }

    if (g_flag_10ms)
    {
      g_flag_10ms = 0U;
      ControllerMgr_Run10ms();
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint8_t cnt5 = 0U;
  static uint8_t cnt10 = 0U;

  if (htim->Instance == TIM6)
  {
    PiRxData_t pi_snapshot;

    g_sys_ms++;
    dbg_sys_ms = g_sys_ms;

    ProtocolPi_CopyData(&pi_snapshot);
    TaskMgr_Update1ms(pi_snapshot.ball_x_mm, pi_snapshot.ball_y_mm, pi_snapshot.ball_valid);

    cnt5++;
    cnt10++;

    if (cnt5 >= 5U)
    {
      cnt5 = 0U;
      g_flag_5ms = 1U;
      dbg_5ms_cnt++;
    }

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
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
