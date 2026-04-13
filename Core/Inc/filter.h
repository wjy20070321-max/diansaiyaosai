#ifndef __FILTER_H__
#define __FILTER_H__

#include "app_config.h"

typedef struct
{
    float alpha;
    float y;
    uint8_t inited;
} LPF1_t;

void LPF1_Init(LPF1_t *f, float alpha);
float LPF1_Run(LPF1_t *f, float x);
float Clampf(float x, float minv, float maxv);
float Deadbandf(float x, float band);

#endif
