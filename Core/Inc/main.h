#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4xx_hal.h"
#include "app_config.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

void Error_Handler(void);

#endif
