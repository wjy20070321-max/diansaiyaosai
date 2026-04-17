#ifndef __PROTOCOL_SCREEN_H__
#define __PROTOCOL_SCREEN_H__

#include "app_config.h"

typedef struct
{
    uint8_t task_id;
    uint8_t start_cmd;
    uint8_t stop_cmd;
    uint8_t imu_zero_cmd;

    uint8_t point_valid;
    float point_x_mm;
    float point_y_mm;

    uint8_t region_valid;
    uint8_t region_id;

    uint8_t route_valid;
    uint8_t route_pass_mode;
    uint8_t route_len;
    uint8_t route[USER_ROUTE_MAX_LEN];

    uint8_t roundtrip_valid;
    uint8_t roundtrip_a;
    uint8_t roundtrip_b;

    uint32_t update_ms;
} ScreenRxData_t;

void ProtocolScreen_Init(void);
void ProtocolScreen_RxByte(uint8_t byte);
ScreenRxData_t* ProtocolScreen_GetData(void);
void ProtocolScreen_SendText(const char *text);

#endif
