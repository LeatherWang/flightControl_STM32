#ifndef __DELAY_H
#define __DELAY_H 	

#include "stm32f4xx.h"

void Systick_Init(void);
void delay_1ms(unsigned int time);
void delay_ms(u16 nms);
void delay_us(u32 nus);
void delay_xms(u16 nms);

void my_delay_us(u16 time);//自定义非精准延时
void my_delay_ms(u16 time);
#endif
