#ifndef __ROUTE_MGR_H__
#define __ROUTE_MGR_H__

#include "app_config.h"
#include "region.h"

typedef struct
{
    float x_mm;
    float y_mm;
    uint8_t mode;
    uint8_t need_hold;
    uint16_t hold_ms;
    uint16_t timeout_ms;
    uint8_t is_region_target;
    uint8_t region_id;
} RouteStep_t;

uint8_t RouteMgr_BuildTask(uint8_t task_id, const uint8_t *user_route,
                           RouteStep_t *steps, uint8_t max_steps);

#endif
