#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f4xx.h"
#include "IMU.h"
#include "control.h"

#define micros() TIM5->CNT
extern float PIT,ROL,YAW;

void TIM2_Init(void);
void TIM3_Init(void);
void Initial_System_Timer(void);//ÓÃÓÚµÍÍ¨ÂË²¨
#endif
