/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序入口文件（舵机/IMU 方向测试版）
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
#include "app_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* -------------------- 测试参数 -------------------- */
/* 小角度方向测试，先不要设太大 */
#define TEST_CMD_DEG       30.5f
#define TEST_STEP_HOLD_MS  3500U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* -------------------- 系统节拍标志 -------------------- */
volatile uint8_t g_flag_5ms = 0U;
volatile uint8_t g_flag_10ms = 0U;

/* -------------------- 系统毫秒计数 -------------------- */
volatile uint32_t g_sys_ms = 0U;

/* -------------------- 调试镜像变量 -------------------- */
volatile uint32_t dbg_sys_ms = 0U;
volatile uint32_t dbg_5ms_cnt = 0U;
volatile uint32_t dbg_10ms_cnt = 0U;

/* -------------------- 舵机/IMU 测试调试变量 -------------------- */
volatile uint8_t dbg_test_step = 0U;           // 当前测试步骤号
volatile float   dbg_test_cmd_x_deg = 0.0f;    // 当前输出给 X 轴的测试命令
volatile float   dbg_test_cmd_y_deg = 0.0f;    // 当前输出给 Y 轴的测试命令
volatile float   dbg_meas_x_deg = 0.0f;        // 按平台 X 轴定义整理后的测量角
volatile float   dbg_meas_y_deg = 0.0f;        // 按平台 Y 轴定义整理后的测量角
volatile uint8_t dbg_test_auto_run = 1U;       // 1=自动测试，0=停在当前状态

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void DirectionTest_RunStep(void);
static void DirectionTest_ApplyCmd(float x_deg, float y_deg);
static void DirectionTest_UpdateDebug(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief 应用测试舵机命令
  * @param x_deg X 轴相对命令角
  * @param y_deg Y 轴相对命令角
  *
  * @note 这里只直接驱动舵机，不跑闭环控制。
  */
static void DirectionTest_ApplyCmd(float x_deg, float y_deg)
{
  dbg_test_cmd_x_deg = x_deg;
  dbg_test_cmd_y_deg = y_deg;
  Servo_SetXYDeg(x_deg, y_deg);
}

/**
  * @brief 更新 IMU 调试量
  *
  * @note
  * 你当前实测关系是：
  * - 左边上 -> pitch 增加
  * - 下边上 -> roll 增加
  *
  * 所以这里直接按平台坐标定义：
  * - X轴（左右）看 pitch
  * - Y轴（上下）看 roll
  *
  * 同时，这里再按当前 IMU_REVERSE 宏做符号修正，
  * 方便你直接在 Watch 里看最终控制意义上的 X/Y 测量量。
  */
static void DirectionTest_UpdateDebug(void)
{
#if IMU_ROLL_REVERSE
  dbg_meas_x_deg = -(g_sys.imu.pitch_deg - g_sys.imu.pitch_zero); // 左右轴
#else
  dbg_meas_x_deg =  (g_sys.imu.pitch_deg - g_sys.imu.pitch_zero); // 左右轴
#endif

#if IMU_PITCH_REVERSE
  dbg_meas_y_deg = -(g_sys.imu.roll_deg - g_sys.imu.roll_zero);   // 上下轴
#else
  dbg_meas_y_deg =  (g_sys.imu.roll_deg - g_sys.imu.roll_zero);   // 上下轴
#endif
}

/**
  * @brief 方向测试状态机
  *
  * @note 测试顺序：
  *       0: 回中
  *       1: X = +TEST_CMD_DEG
  *       2: 回中
  *       3: X = -TEST_CMD_DEG
  *       4: 回中
  *       5: Y = +TEST_CMD_DEG
  *       6: 回中
  *       7: Y = -TEST_CMD_DEG
  *       8: 回中
  *       然后循环
  *
  *       用途：
  *       - 看平台实际往哪边动
  *       - 看按平台坐标整理后的 X / Y 测量量是否对应
  */
static void DirectionTest_RunStep(void)
{
  static uint32_t last_ms = 0U;

  if (!dbg_test_auto_run)
  {
    return;
  }

  if ((g_sys_ms - last_ms) < TEST_STEP_HOLD_MS)
  {
    return;
  }

  last_ms = g_sys_ms;

  switch (dbg_test_step)
  {
    case 0:
      DirectionTest_ApplyCmd(0.0f, 0.0f);
      dbg_test_step = 1U;
      break;

    case 1:
      DirectionTest_ApplyCmd(+TEST_CMD_DEG, 0.0f);
      dbg_test_step = 2U;
      break;

    case 2:
      DirectionTest_ApplyCmd(0.0f, 0.0f);
      dbg_test_step = 3U;
      break;

    case 3:
      DirectionTest_ApplyCmd(-TEST_CMD_DEG, 0.0f);
      dbg_test_step = 4U;
      break;

    case 4:
      DirectionTest_ApplyCmd(0.0f, 0.0f);
      dbg_test_step = 5U;
      break;

    case 5:
      DirectionTest_ApplyCmd(0.0f, +TEST_CMD_DEG);
      dbg_test_step = 6U;
      break;

    case 6:
      DirectionTest_ApplyCmd(0.0f, 0.0f);
      dbg_test_step = 7U;
      break;

    case 7:
      DirectionTest_ApplyCmd(0.0f, -TEST_CMD_DEG);
      dbg_test_step = 8U;
      break;

    default:
      DirectionTest_ApplyCmd(0.0f, 0.0f);
      dbg_test_step = 0U;
      break;
  }
}

/* USER CODE END 0 */

/**
  * @brief  程序入口函数
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

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM12_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  Servo_Init();
  JY61P_Init();
  ProtocolPi_Init();
  ProtocolScreen_Init();
  Region_Init();
  TaskMgr_Init();
  BallOuterLoop_Init();
  PlateInnerLoop_Init();
  ControllerMgr_Init();

  BSP_UART_StartReceiveIT();
  HAL_TIM_Base_Start_IT(&htim6);

  /* 上电先回中 */
  Servo_Center();

  /* 等待稳定 */
  HAL_Delay(1500);

  /* 以当前水平状态作为零点 */
  JY61P_SetZero();

  /* 再停一下，便于观察 */
  HAL_Delay(500);

  /* 再次回中 */
  DirectionTest_ApplyCmd(0.0f, 0.0f);

  ProtocolScreen_SendText("DIR_TEST_READY\r\n");
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN WHILE */

    /* 保持 IMU / 视觉 / 串口屏数据更新 */
    ControllerMgr_UpdateInputs();

    /* 更新调试量 */
    DirectionTest_UpdateDebug();

    /* 不跑闭环，只跑方向测试状态机 */
    DirectionTest_RunStep();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 这里故意不做 100ms 状态上报，避免串口屏回传污染 */
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
/**
  * @brief 定时器周期中断回调函数
  * @param htim 当前触发中断的定时器句柄
  */
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
  * @brief 发生错误时执行的函数
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
