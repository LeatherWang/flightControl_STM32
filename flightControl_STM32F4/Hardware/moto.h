#ifndef	Moto_H
#define Moto_H

#include "stm32f4xx.h"

#define Moto_PwmMax 3500   //占空比是10% 
#define Moto_PwmMin 0  //占空比是5%

void Moto_Init(void);
extern void Moto_PwmRflash(float MOTO1_PWM,float MOTO2_PWM,float MOTO3_PWM,float MOTO4_PWM);


#endif
