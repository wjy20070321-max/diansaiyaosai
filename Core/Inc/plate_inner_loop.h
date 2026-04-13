#ifndef __PLATE_INNER_LOOP_H__
#define __PLATE_INNER_LOOP_H__

#include "app_config.h"

typedef struct
{
    float servo_x_cmd_deg;
    float servo_y_cmd_deg;
} PlateInnerOutput_t;

void PlateInnerLoop_Init(void);
void PlateInnerLoop_Reset(void);
void PlateInnerLoop_Run(float theta_x_ref, float theta_y_ref,
                        float theta_x_meas, float theta_y_meas,
                        float gyro_x_meas, float gyro_y_meas,
                        PlateInnerOutput_t *out);

#endif
