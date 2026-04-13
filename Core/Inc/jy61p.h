#ifndef __JY61P_H__
#define __JY61P_H__

#include "app_config.h"

typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float roll_zero;
    float pitch_zero;
    uint8_t valid;
    uint32_t update_ms;
} JY61P_Data_t;

void JY61P_Init(void);
void JY61P_RxByte(uint8_t byte);
void JY61P_SetZero(void);
void JY61P_CopyData(JY61P_Data_t *dest);
JY61P_Data_t* JY61P_GetData(void);

#endif
