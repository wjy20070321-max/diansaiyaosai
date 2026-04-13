#ifndef __REGION_H__
#define __REGION_H__

#include "app_config.h"

typedef struct
{
    float x;
    float y;
} Point2f_t;

void Region_Init(void);
Point2f_t Region_GetCenter(uint8_t idx);
uint8_t Region_IsEntered(uint8_t idx, float x, float y);
uint8_t Region_IsHeld(uint8_t idx, float x, float y);
uint8_t Region_FindCurrent(float x, float y);
float Region_DistToCenter(uint8_t idx, float x, float y);

#endif
