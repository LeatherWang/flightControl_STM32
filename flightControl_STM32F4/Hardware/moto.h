#ifndef	Moto_H
#define Moto_H

#include "stm32f4xx.h"

#define Moto_PwmMax 3500   //ռ�ձ���10% 
#define Moto_PwmMin 0  //ռ�ձ���5%

void Moto_Init(void);
extern void Moto_PwmRflash(float MOTO1_PWM,float MOTO2_PWM,float MOTO3_PWM,float MOTO4_PWM);


#endif
