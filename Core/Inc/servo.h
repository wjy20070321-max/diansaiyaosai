#ifndef __SERVO_H__
#define __SERVO_H__

#include "app_config.h"

void Servo_Init(void);
void Servo_SetXDeg(float deg);
void Servo_SetYDeg(float deg);
void Servo_SetXYDeg(float x_deg, float y_deg);
void Servo_Center(void);

#endif
