#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"
#include "sys.h"

#define LED0 PDout(14)
#define LED1 PDout(15)
#define LED2 PEout(3)
#define LED3 PCout(13)

void LED_Init(void);

#endif
