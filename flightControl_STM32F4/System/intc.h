#ifndef __INTC_H
#define __INTC_H


void EXTIX_Init(void);
void Ultrasonic_NVIC_Config(void);

//�����ľ���  ��λ �ס�
extern float  Ultra_Distance;

//������ģ��������API ����
void Ultrasonic_initial(void); //��ʼ�����ϵ��ʱ����Ҫ����һ��
void Ultrasonic_Routine(void); //�������̣߳���Ҫ�ŵ�ѭ���У����ϵ��á�

#endif

