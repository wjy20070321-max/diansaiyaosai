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

    /* 兼容旧代码保留：表示“最近一次收到任意 PI 帧”的时间 */
    uint32_t update_ms;

    /* 新增：按数据类型拆分时间戳，避免超时判断误判 */
    uint32_t ball_update_ms;   /* 仅球状态帧更新 */
    uint32_t cmd_update_ms;    /* task/route/target 等命令帧更新 */
    uint32_t laser_update_ms;  /* 激光追踪帧更新 */

    /* 激光数据 */
    float laser_x_mm;
    float laser_y_mm;
    uint8_t laser_valid;

} PiRxData_t;

void ProtocolPi_Init(void);
void ProtocolPi_RxByte(uint8_t byte);
void ProtocolPi_CopyData(PiRxData_t *dest);
PiRxData_t* ProtocolPi_GetData(void);

/* 一次性命令消费接口，防止主循环反复执行同一命令 */
void ProtocolPi_ClearTaskCtrl(void);
void ProtocolPi_ClearRoute(void);
void ProtocolPi_ClearTarget(void);

#endif
