#ifndef __SAFETY_MGR_H__
#define __SAFETY_MGR_H__

#include "app_config.h"

uint8_t SafetyMgr_CheckImuTimeout(uint32_t now_ms, uint32_t imu_ms);
uint8_t SafetyMgr_CheckPiTimeout(uint32_t now_ms, uint32_t pi_ms);
uint8_t SafetyMgr_CheckBallNearEdge(float x_mm, float y_mm);

#endif
