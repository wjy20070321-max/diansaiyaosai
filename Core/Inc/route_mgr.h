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

uint8_t RouteMgr_BuildRegionSequence(const uint8_t *route, uint8_t len,
                                     uint16_t hold_ms,
                                     RouteStep_t *steps, uint8_t max_steps);

uint8_t RouteMgr_BuildPassSequence(const uint8_t *route, uint8_t len,
                                   uint16_t final_hold_ms,
                                   RouteStep_t *steps, uint8_t max_steps);

uint8_t RouteMgr_BuildRoundTrip(uint8_t a, uint8_t b, uint8_t cycles,
                                uint16_t hold_ms,
                                RouteStep_t *steps, uint8_t max_steps);

#endif
