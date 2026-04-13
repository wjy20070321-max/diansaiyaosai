#include "stm32f4xx_it.h"
#include "tim.h"
#include "usart.h"

void NMI_Handler(void) {}
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

void USART6_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart6);
}
