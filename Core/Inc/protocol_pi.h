#ifndef __PROTOCOL_PI_H__
#define __PROTOCOL_PI_H__

#include "app_config.h"

typedef struct
{
    float ball_x_mm;
    float ball_y_mm;
    float ball_vx_mmps;
    float ball_vy_mmps;
    uint8_t ball_valid;
    uint8_t task_id;
    uint8_t start_cmd;
    uint8_t stop_cmd;
    uint8_t route_a;
    uint8_t route_b;
    uint8_t route_c;
    uint8_t route_d;
    float target_x_mm;
    float target_y_mm;
    uint8_t target_valid;
    uint32_t update_ms;
	
	// ======== 新增激光数据 ========
    float laser_x_mm;
    float laser_y_mm;
    uint8_t laser_valid; // 1: 视野内有激光, 0: 无激光
	
} PiRxData_t;

void ProtocolPi_Init(void);
void ProtocolPi_RxByte(uint8_t byte);
void ProtocolPi_CopyData(PiRxData_t *dest);
PiRxData_t* ProtocolPi_GetData(void);

#endif
