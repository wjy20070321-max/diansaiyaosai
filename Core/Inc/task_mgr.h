#ifndef __TASK_MGR_H__
#define __TASK_MGR_H__

#include "app_config.h"
#include "route_mgr.h"

typedef struct
{
    uint8_t running;
    uint8_t finished;
    uint8_t failed;

    uint8_t task_id;
    uint8_t current_step;
    uint8_t total_steps;

    uint32_t total_time_ms;
    uint32_t step_time_ms;
    uint32_t hold_count_ms;

    uint8_t route_len;
    uint8_t route_pass_mode;
    uint8_t route_region[USER_ROUTE_MAX_LEN];

    uint8_t roundtrip_a;
    uint8_t roundtrip_b;
    uint8_t roundtrip_cycles;

    uint8_t current_region_cmd;

    float point_x_mm;
    float point_y_mm;

    RouteStep_t steps[TASK_MAX_STEPS];
} TaskContext_t;

void TaskMgr_Init(void);
void TaskMgr_LoadTask(uint8_t task_id);

void TaskMgr_SetDirectPoint(float x_mm, float y_mm);
void TaskMgr_SetDirectRegion(uint8_t region_id);
void TaskMgr_SetRouteSequence(const uint8_t *route, uint8_t len, uint8_t pass_mode);
void TaskMgr_SetRoundTrip(uint8_t a, uint8_t b, uint8_t cycles);

void TaskMgr_Start(void);
void TaskMgr_Stop(void);
void TaskMgr_Update1ms(float ball_x, float ball_y, uint8_t ball_valid);
uint8_t TaskMgr_IsRunning(void);
uint8_t TaskMgr_IsFinished(void);
uint8_t TaskMgr_IsFailed(void);
void TaskMgr_GetTarget(float *x_mm, float *y_mm, uint8_t *mode);
TaskContext_t* TaskMgr_GetContext(void);

#endif
