/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ����������ļ�
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
/* ��Щ�����Լ�д��Ӧ�ò�ģ��ͷ�ļ� */
#include "servo.h"             // �������ģ��
#include "jy61p.h"             // JY61P ��̬������ģ��
#include "protocol_pi.h"       // ��ݮ��ͨ��Э��ģ��
#include "protocol_screen.h"   // ������ͨ��Э��ģ��
#include "region.h"            // �����������ģ��
#include "task_mgr.h"          // �������ģ��
#include "ball_outer_loop.h"   // С��λ���⻷����ģ��
#include "plate_inner_loop.h"  // ƽ̨��̬�ڻ�����ģ��
#include "controller_mgr.h"    // �������ܹ���ģ��
#include "debug_port.h"        // ���Խӿ�ģ��
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* ������Է����Լ��Ľṹ�����Ͷ��壬ĿǰΪ�� */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ������Է����Լ��ĺ궨�壬ĿǰΪ�� */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* ������Է����Լ��ĺ꺯����ĿǰΪ�� */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* -------------------- ϵͳ���ı�־ -------------------- */
/* �� TIM6 �� 1ms �ж���λ������ѭ���ж�ȡ������ */
volatile uint8_t g_flag_5ms = 0;     // 5ms �������񴥷���־
volatile uint8_t g_flag_10ms = 0;    // 10ms �������񴥷���־

/* -------------------- ϵͳ������� -------------------- */
/* ÿ����һ�� TIM6 �жϾͼ� 1������ϵͳ����ʱ���׼ */
volatile uint32_t g_sys_ms = 0;

/* -------------------- �����ü������� -------------------- */
/* ��Щ������Ҫ������۲�ϵͳ�����Ƿ����� */
volatile uint32_t dbg_sys_ms = 0U;   // ���Թ۲���ϵͳ�������
volatile uint32_t dbg_5ms_cnt = 0U;  // ���Թ۲��� 5ms ���񴥷�����
volatile uint32_t dbg_10ms_cnt = 0U; // ���Թ۲��� 10ms ���񴥷�����
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* ������Է����Լ��ĺ���������ĿǰΪ�� */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ������Է��û��Զ����˽�к�����ĿǰΪ�� */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* �� HAL_Init() ǰ����������ر����ڵı�����ʼ�����ɷ����� */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* ������Է����Լ��ĳ�ʼ�����룬ĿǰΪ�� */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* ������Է�ϵͳ����ʼ�����룬ĿǰΪ�� */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM12_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* -------------------- ��ʼ��Ӧ�ò�ģ�� -------------------- */
  Servo_Init();             // ��ʼ�����ģ�飬���� PWM ���
  JY61P_Init();             // ��ʼ�� JY61P ��̬���������ݽṹ
  ProtocolPi_Init();        // ��ʼ����ݮ��ͨ��Э�������
  ProtocolScreen_Init();    // ��ʼ��������Э�������
  Region_Init();            // ��ʼ�����������
  TaskMgr_Init();           // ��ʼ�����������
  BallOuterLoop_Init();     // ��ʼ��С��λ���⻷������
  PlateInnerLoop_Init();    // ��ʼ��ƽ̨��̬�ڻ�������
  ControllerMgr_Init();     // ��ʼ���ܿ�����������

  /* ���������жϽ���
     ���� USART1 / USART3 / USART6 �յ�����ʱ���Զ������жϻص� */
  BSP_UART_StartReceiveIT();

  /* ���� TIM6 �Ķ�ʱ�ж�
     TIM6 ��������Ϊ 1ms ��ϵͳ������ */
  HAL_TIM_Base_Start_IT(&htim6);

  /* �ϵ�����ö�����У�����ƽ̨�Ҷ� */
  Servo_Center();

  /* �ȴ�ϵͳ�ȶ�
     �������ȶ���IMU �ϵ��ȶ��������豸��������״̬ */
  HAL_Delay(1500);

  /* �Ե�ǰ��̬��Ϊ IMU ��� */
  JY61P_SetZero();

  /* �����������;�����ʾ
     ����û�Ӵ����������Ҳͨ������Ӱ�������� */
  ProtocolScreen_SendText("READY\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    /* ��ͳһ���¸����������ݣ�
       - ��ݮ���Ӿ�����
       - IMU ����
       - ����������
       - ����״̬ */
    ControllerMgr_UpdateInputs();

    /* 5ms ��������
       һ�������ڻ���ƽ̨��̬���ơ�������Ƶȸ���ıջ� */
    if (g_flag_5ms)
    {
      g_flag_5ms = 0U;      // �����־�������ظ�ִ��
      ControllerMgr_Run5ms();
    }

    /* 10ms ��������
       һ�������⻷��С��λ�ÿ��ơ�����Ŀ����µ� */
    if (g_flag_10ms)
    {
      g_flag_10ms = 0U;     // �����־�������ظ�ִ��
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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  * @brief ��ʱ�����ڵ���ص�����
  * @param htim: ��ǰ�����жϵĶ�ʱ�����
  *
  * @note ����Ŀ����Ҫ���� TIM6��
  *       - TIM6 ÿ 1ms ����һ���ж�
  *       - ������ά��ϵͳ�������
  *       - ������������������� 1ms ����
  *       - ��������� 5ms / 10ms �Ŀ��ƽ��ı�־
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* ��̬�ֲ������������ڶ���ж�֮�䱣������ֵ */
  static uint8_t cnt5 = 0U;   // 5ms ��Ƶ������
  static uint8_t cnt10 = 0U;  // 10ms ��Ƶ������

  /* ֻ���� TIM6 ���ж� */
  if (htim->Instance == TIM6)
  {
    /* ��ʱ���ձ���
       ����������ǰ��ݮ�ɷ�������λ����Ϣ */
    PiRxData_t pi_snapshot;

    /* -------------------- ϵͳʱ����� -------------------- */
    g_sys_ms++;               // ÿ 1ms �� 1
    dbg_sys_ms = g_sys_ms;    // ���Թ۲��þ������

    /* -------------------- ����� 1ms ���� -------------------- */
    /* �Ȱ�ȫ����һ����ݮ���Ӿ����� */
    ProtocolPi_CopyData(&pi_snapshot);

    /* ����ǰС��λ�ú���Ч״̬�������������
       ������������жϣ�
       - �Ƿ񵽴�Ŀ��
       - �Ƿ���Ҫ�л���һ��
       - �Ƿ�ʱʧ�� */
    TaskMgr_Update1ms(pi_snapshot.ball_x_mm, pi_snapshot.ball_y_mm, pi_snapshot.ball_valid);

    /* -------------------- ���� 5ms / 10ms ���� -------------------- */
    cnt5++;
    cnt10++;

    /* ÿ 5 �� 1ms �жϣ���λһ�� 5ms ��־ */
    if (cnt5 >= 5U)
    {
      cnt5 = 0U;
      g_flag_5ms = 1U;
      dbg_5ms_cnt++; // ����ͳ�� 5ms ����ִ�д���
    }

    /* ÿ 10 �� 1ms �жϣ���λһ�� 10ms ��־ */
    if (cnt10 >= 10U)
    {
      cnt10 = 0U;
      g_flag_10ms = 1U;
      dbg_10ms_cnt++; // ����ͳ�� 10ms ����ִ�д���
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
  __disable_irq();   // �ر�ȫ���жϣ���ֹϵͳ����ʧ������
  while (1)
  {
    /* ��ѭ���ȴ��˹���λ���Ź���λ */
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
  /* ������ԼӴ��ڴ�ӡ�������������ʧ��λ�� */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
