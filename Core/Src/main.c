/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序入口文件（Y轴舵机单独测试版）
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
#include "controller_mgr.h"
#include "debug_port.h"
#include "app_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* -------------------- Y轴测试参数 -------------------- */
#define TEST_STEP_HOLD_MS      1800U

#define TEST_Y_CMD_1_DEG       20.0f
#define TEST_Y_CMD_2_DEG       30.0f
#define TEST_Y_CMD_3_DEG       50.0f

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

/* -------------------- Y轴测试调试变量 -------------------- */
volatile uint8_t dbg_test_step = 0U;         // 当前测试步骤号
volatile float   dbg_y_cmd_deg = 0.0f;       // 当前Y轴测试命令
volatile float   dbg_meas_y_deg = 0.0f;      // 按平台Y轴定义整理后的测量角
volatile float   dbg_roll_raw_deg = 0.0f;    // 原始roll
volatile float   dbg_pitch_raw_deg = 0.0f;   // 原始pitch
volatile uint8_t dbg_test_auto_run = 1U;     // 1=自动循环，0=停在当前状态

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void YServoTest_ApplyCmd(float y_deg);
static void YServoTest_UpdateDebug(void);
static void YServoTest_RunStep(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief 只给Y轴舵机下命令
  * @param y_deg Y轴相对平台中心的命令角
  */
static void YServoTest_ApplyCmd(float y_deg)
{
  dbg_y_cmd_deg = y_deg;

  /* X轴固定回中，只测Y轴 */
  Servo_SetXYDeg(0.0f, y_deg);
}

/**
  * @brief 更新Y轴测试调试量
  *
  * @note
  * 你当前平台定义里：
  * - Y轴 = 上下方向
  * - 实测对应 IMU 原始 roll
  *
  * 同时按当前 IMU_PITCH_REVERSE 做符号修正，
  * 这样 dbg_meas_y_deg 就表示“平台Y轴测量量”。
  */
static void YServoTest_UpdateDebug(void)
{
  dbg_roll_raw_deg  = g_sys.imu.roll_deg  - g_sys.imu.roll_zero;
  dbg_pitch_raw_deg = g_sys.imu.pitch_deg - g_sys.imu.pitch_zero;

#if IMU_PITCH_REVERSE
  dbg_meas_y_deg = -(g_sys.imu.roll_deg - g_sys.imu.roll_zero);
#else
  dbg_meas_y_deg =  (g_sys.imu.roll_deg - g_sys.imu.roll_zero);
#endif
}

/**
  * @brief Y轴舵机自动测试状态机
  *
  * @note 顺序：
  *       0:  回中
  *       1:  +1°
  *       2:  回中
  *       3:  +3°
  *       4:  回中
  *       5:  +5°
  *       6:  回中
  *       7:  -1°
  *       8:  回中
  *       9:  -3°
  *       10: 回中
  *       11: -5°
  *       12: 回中
  *       然后循环
  *
  *       你就能直观看出：
  *       - 哪个方向上去更顺
  *       - 哪个方向下不去 / 卡住 / 变小
  */
static void YServoTest_RunStep(void)
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
      YServoTest_ApplyCmd(0.0f);
      dbg_test_step = 1U;
      break;

    case 1:
      YServoTest_ApplyCmd(+TEST_Y_CMD_1_DEG);
      dbg_test_step = 2U;
      break;

    case 2:
      YServoTest_ApplyCmd(0.0f);
      dbg_test_step = 3U;
      break;

    case 3:
      YServoTest_ApplyCmd(+TEST_Y_CMD_2_DEG);
      dbg_test_step = 4U;
      break;

    case 4:
      YServoTest_ApplyCmd(0.0f);
      dbg_test_step = 5U;
      break;

    case 5:
      YServoTest_ApplyCmd(+TEST_Y_CMD_3_DEG);
      dbg_test_step = 6U;
      break;

    case 6:
      YServoTest_ApplyCmd(0.0f);
      dbg_test_step = 7U;
      break;

    case 7:
      YServoTest_ApplyCmd(-TEST_Y_CMD_1_DEG);
      dbg_test_step = 8U;
      break;

    case 8:
      YServoTest_ApplyCmd(0.0f);
      dbg_test_step = 9U;
      break;

    case 9:
      YServoTest_ApplyCmd(-TEST_Y_CMD_2_DEG);
      dbg_test_step = 10U;
      break;

    case 10:
      YServoTest_ApplyCmd(0.0f);
      dbg_test_step = 11U;
      break;

    case 11:
      YServoTest_ApplyCmd(-TEST_Y_CMD_3_DEG);
      dbg_test_step = 12U;
      break;

    default:
      YServoTest_ApplyCmd(0.0f);
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
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM12_Init();

  /* USER CODE BEGIN 2 */
  Servo_Init();
  JY61P_Init();
  ProtocolPi_Init();
  ProtocolScreen_Init();
  ControllerMgr_Init();

  BSP_UART_StartReceiveIT();
  HAL_TIM_Base_Start_IT(&htim6);

  /* 上电先回中 */
  Servo_Center();

  /* 等待机构和IMU稳定 */
  HAL_Delay(1500);

  /* 上电后以当前姿态为零点 */
  JY61P_SetZero();

  HAL_Delay(500);

  /* 再回中一次 */
  YServoTest_ApplyCmd(0.0f);

  ProtocolScreen_SendText("Y_SERVO_TEST_READY\r\n");
  /* USER CODE END 2 */

  while (1)
  {
    /* 保持IMU/视觉/串口数据更新 */
    ControllerMgr_UpdateInputs();

    /* 更新调试量 */
    YServoTest_UpdateDebug();

    /* 跑Y轴自动测试 */
    YServoTest_RunStep();
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
    g_sys_ms++;
    dbg_sys_ms = g_sys_ms;

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
