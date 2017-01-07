#ifndef __INTC_H
#define __INTC_H


void EXTIX_Init(void);
void Ultrasonic_NVIC_Config(void);

//引出的距离  单位 米。
extern float  Ultra_Distance;

//超声波模块引出的API 程序
void Ultrasonic_initial(void); //初始化，上电的时候需要调用一次
void Ultrasonic_Routine(void); //超声波线程，需要放到循环中，不断调用。

#endif

