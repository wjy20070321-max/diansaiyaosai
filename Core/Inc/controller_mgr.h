#ifndef __CONTROLLER_MGR_H__
#define __CONTROLLER_MGR_H__

#include "app_config.h"
#include "jy61p.h"
#include "protocol_pi.h"
#include "task_mgr.h"
#include "ball_outer_loop.h"
#include "plate_inner_loop.h"

typedef struct
{
    float x_mm;
    float y_mm;
    float vx_mmps;
    float vy_mmps;
    uint8_t valid;
    uint32_t tick_ms;
} BallState_t;

typedef struct
{
    float target_x_mm;
    float target_y_mm;
    float theta_x_ref_deg;
    float theta_y_ref_deg;
    float servo_x_cmd_deg;
    float servo_y_cmd_deg;
    uint8_t mode;
} ControlRef_t;

typedef struct
{
    BallState_t ball;
    JY61P_Data_t imu;
    ControlRef_t ref;
    uint8_t emergency_stop;
} SystemContext_t;

extern SystemContext_t g_sys;

void ControllerMgr_Init(void);
void ControllerMgr_UpdateInputs(void);
void ControllerMgr_Run10ms(void);
void ControllerMgr_Run5ms(void);

#endif
