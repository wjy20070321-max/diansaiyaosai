#ifndef __PID_H__
#define __PID_H__

#include "app_config.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integ;
    float prev_err;
    float out_min;
    float out_max;
    float integ_min;
    float integ_max;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max,
              float integ_min, float integ_max);
float PID_Run(PID_t *pid, float ref, float fdb, float dt);
float PID_RunErr(PID_t *pid, float err, float dt);
void PID_Reset(PID_t *pid);

#endif
