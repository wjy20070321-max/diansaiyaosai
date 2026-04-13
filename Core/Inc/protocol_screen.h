#ifndef __PROTOCOL_SCREEN_H__
#define __PROTOCOL_SCREEN_H__

#include "app_config.h"

typedef struct
{
    uint8_t task_id;
    uint8_t start_cmd;
    uint8_t stop_cmd;
    uint8_t route_valid;
    uint8_t route_a;
    uint8_t route_b;
    uint8_t route_c;
    uint8_t route_d;
    uint8_t imu_zero_cmd;
    uint32_t update_ms;
} ScreenRxData_t;

void ProtocolScreen_Init(void);
void ProtocolScreen_RxByte(uint8_t byte);
ScreenRxData_t* ProtocolScreen_GetData(void);
void ProtocolScreen_SendText(const char *text);

#endif
