#include "usart.h"
#include "main.h"
#include "jy61p.h"
#include "protocol_pi.h"
#include "protocol_screen.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

static uint8_t uart1_rx_byte = 0;
static uint8_t uart3_rx_byte = 0;
static uint8_t uart6_rx_byte = 0;

void MX_USART1_UART_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();

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

    HAL_NVIC_SetPriority(USART1_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void MX_USART3_UART_Init(void)
{
    __HAL_RCC_USART3_CLK_ENABLE();

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 9600;
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

    HAL_NVIC_SetPriority(USART3_IRQn, 1, 2);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void MX_USART6_UART_Init(void)
{
    __HAL_RCC_USART6_CLK_ENABLE();

    huart6.Instance = USART6;
    huart6.Init.BaudRate = 115200;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(USART6_IRQn, 1, 3);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (uartHandle->Instance == USART1)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /* 树莓派：USART1 TX/RX -> PB6/PB7 */
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if (uartHandle->Instance == USART3)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
        /* JY61P：USART3 TX/RX -> PC10/PC11 */
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
    else if (uartHandle->Instance == USART6)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
        /* 串口屏：USART6 TX/RX -> PG14/PG9 */
        GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    }
}

void BSP_UART_StartReceiveIT(void)
{
    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    HAL_UART_Receive_IT(&huart6, &uart6_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        ProtocolPi_RxByte(uart1_rx_byte);
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    }
    else if (huart->Instance == USART3)
    {
        JY61P_RxByte(uart3_rx_byte);
        HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    }
    else if (huart->Instance == USART6)
    {
        ProtocolScreen_RxByte(uart6_rx_byte);
        HAL_UART_Receive_IT(&huart6, &uart6_rx_byte, 1);
    }
}
