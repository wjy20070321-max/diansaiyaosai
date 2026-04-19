/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   USART 外设配置与串口收发中断处理文件
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "main.h"             // Error_Handler() 和主工程全局定义
#include "jy61p.h"            // JY61P 串口接收解析
#include "protocol_pi.h"      // 树莓派协议解析
#include "protocol_screen.h"  // 串口屏协议解析

/* -------------------- 单字节接收缓存 -------------------- */
/* 每个串口都使用“1字节中断接收”方式 */
static uint8_t uart1_rx_byte = 0U;   // USART1 当前接收到的 1 个字节
static uint8_t uart2_rx_byte = 0U;   // USART2 当前接收到的 1 个字节
static uint8_t uart3_rx_byte = 0U;   // USART3 当前接收到的 1 个字节

/* -------------------- 调试统计变量 -------------------- */
/* 方便你在调试窗口观察串口是否正常收数、是否频繁报错 */
volatile uint32_t dbg_uart1_rx_cnt = 0U;   // USART1 收到字节计数
volatile uint32_t dbg_uart1_err_cnt = 0U;  // USART1 错误计数

volatile uint32_t dbg_uart2_rx_cnt = 0U;   // USART2 收到字节计数
volatile uint32_t dbg_uart2_err_cnt = 0U;  // USART2 错误计数

volatile uint32_t dbg_uart3_rx_cnt = 0U;   // USART3 收到字节计数
volatile uint32_t dbg_uart3_err_cnt = 0U;  // USART3 错误计数

/**
 * @brief 在串口异常后重新启动单字节中断接收
 * @param huart 当前发生异常的串口句柄
 *
 * @note 这个函数用于错误恢复。
 *       当串口出现 ORE / FE / NE / PE 等错误后，
 *       需要重新调用 HAL_UART_Receive_IT() 才能继续接收。
 */
static void UART_RestartRxIT(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
  }
  else if (huart->Instance == USART2)
  {
    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
  }
  else if (huart->Instance == USART3)
  {
    HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
  }
}
/* USER CODE END 0 */

/* -------------------- USART 句柄定义 -------------------- */
UART_HandleTypeDef huart1;   // USART1 句柄
UART_HandleTypeDef huart2;   // USART2 句柄
UART_HandleTypeDef huart3;   // USART3 句柄

/* USART1 init function */
/**
 * @brief USART1 初始化
 * @note 当前配置：
 *       - 波特率 115200
 *       - 8 数据位
 *       - 1 停止位
 *       - 无校验
 *       - 收发双向
 *
 *       在你这套工程里，USART1 通常接树莓派。
 */
void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USART2 init function */
/**
 * @brief USART2 初始化
 * @note 当前配置：
 *       - 波特率 115200
 *       - 8 数据位
 *       - 1 停止位
 *       - 无校验
 *       - 收发双向
 *
 *       在你这版工程里，USART2 被用于串口屏。
 */
void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USART3 init function */
/**
 * @brief USART3 初始化
 * @note 当前配置：
 *       - 波特率 115200
 *       - 8 数据位
 *       - 1 停止位
 *       - 无校验
 *       - 收发双向
 *
 *       在你这套工程里，USART3 通常接 JY61P。
 */
void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief USART 底层硬件初始化
 * @param uartHandle 当前要初始化的串口句柄
 *
 * @note 这里负责：
 *       - 开启串口时钟
 *       - 配置对应 GPIO 复用功能
 *       - 配置 NVIC 中断优先级
 */
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* -------------------- USART1 -------------------- */
  if (uartHandle->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA9  -> USART1_TX
       PA10 -> USART1_RX */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART1_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
  /* -------------------- USART2 -------------------- */
  else if (uartHandle->Instance == USART2)
  {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* PD5 -> USART2_TX
       PD6 -> USART2_RX */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART2_IRQn, 1, 3);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
  /* -------------------- USART3 -------------------- */
  else if (uartHandle->Instance == USART3)
  {
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PC10 -> USART3_TX
       PC11 -> USART3_RX */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART3_IRQn, 1, 2);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  }
}

/**
 * @brief USART 底层硬件反初始化
 * @param uartHandle 当前要反初始化的串口句柄
 *
 * @note 一般在外设关闭或重配置时使用。
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  if (uartHandle->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  }
  else if (uartHandle->Instance == USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5 | GPIO_PIN_6);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
  else if (uartHandle->Instance == USART3)
  {
    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_11);
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  }
}

/* USER CODE BEGIN 1 */
/**
 * @brief 启动所有串口的单字节中断接收
 *
 * @note 当前工程里：
 *       - USART1：树莓派
 *       - USART2：串口屏
 *       - USART3：JY61P
 */
void BSP_UART_StartReceiveIT(void)
{
  HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
  HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
  HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
}

/**
 * @brief 串口接收完成回调函数
 * @param huart 当前触发回调的串口句柄
 *
 * @note 当前采用“1 字节中断接收”方式，
 *       每收到 1 个字节就会进入这里。
 *
 *       不同串口收到的字节会交给不同模块处理：
 *       - USART1 -> ProtocolPi_RxByte()      树莓派协议解析
 *       - USART2 -> ProtocolScreen_RxByte()  串口屏文本命令解析
 *       - USART3 -> JY61P_RxByte()           IMU 协议解析
 *
 *       每次处理完后，都要立即再次启动下一字节接收。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    dbg_uart1_rx_cnt++;                     // 树莓派串口收字节计数+1
    ProtocolPi_RxByte(uart1_rx_byte);      // 把这个字节交给树莓派协议解析器
    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1); // 重新启动下一字节接收
  }
  else if (huart->Instance == USART2)
  {
    dbg_uart2_rx_cnt++;                     // 串口屏收字节计数+1
    ProtocolScreen_RxByte(uart2_rx_byte);  // 把这个字节交给串口屏协议解析器
    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1); // 重新启动下一字节接收
  }
  else if (huart->Instance == USART3)
  {
    dbg_uart3_rx_cnt++;                     // IMU 串口收字节计数+1
    JY61P_RxByte(uart3_rx_byte);            // 把这个字节交给 JY61P 协议解析器
    HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1); // 重新启动下一字节接收
  }
}

/**
 * @brief 串口错误回调函数
 * @param huart 当前发生错误的串口句柄
 *
 * @note 当串口出现 ORE / FE / NE / PE 等错误时会进入这里。
 *       处理逻辑是：
 *       1. 统计错误次数
 *       2. 清除错误标志
 *       3. 清空错误状态
 *       4. 重新启动单字节中断接收
 *
 *       这样可以尽量避免串口因为一次异常而永久卡死。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  /* 按串口分别统计错误次数 */
  if (huart->Instance == USART1)
  {
    dbg_uart1_err_cnt++;
  }
  else if (huart->Instance == USART2)
  {
    dbg_uart2_err_cnt++;
  }
  else if (huart->Instance == USART3)
  {
    dbg_uart3_err_cnt++;
  }

  /* 清除常见 UART 错误标志 */
  __HAL_UART_CLEAR_PEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_OREFLAG(huart);

  /* 清空错误状态，恢复接收状态机 */
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  huart->RxState = HAL_UART_STATE_READY;

  /* 重新启动单字节中断接收 */
  UART_RestartRxIT(huart);
}
/* USER CODE END 1 */
