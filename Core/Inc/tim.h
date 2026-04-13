#ifndef __TIM_H__
#define __TIM_H__

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;

void MX_TIM2_Init(void);
void MX_TIM6_Init(void);

#endif
