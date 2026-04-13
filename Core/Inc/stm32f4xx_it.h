#ifndef __STM32F4XX_IT_H__
#define __STM32F4XX_IT_H__

#include "main.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void TIM6_DAC_IRQHandler(void);
void USART1_IRQHandler(void);
void USART3_IRQHandler(void);
void USART6_IRQHandler(void);

#endif
