#include "intc.h"
#include "stm32f4xx_it.h"
#include "uart.h"
#include "sys.h"
#include "timer.h"
#include "delay.h"
#include "MS5611.h"

volatile uint32_t Rise_time,Fall_time;//������32λ�ģ�������

//------------����������״̬��--------------
#define Ultra_ST_Started   0x01	  //�������
#define Ultra_ST_RX1       0x02	  //�յ��ز�1
#define Ultra_ST_RX2       0x03	  //�յ��ز�2
#define Ultra_ST_Idle      0x04	  //��Ϣһ��
#define Ultra_Restart      0x1f	  //��������һ�β��
#define Ultra_ST_Error     0xf2	  //����

#define MOVAVG_SIZE  10	   //���������10������ ����ƽ���˲�

#define Max_Range          (uint32_t)5    //�������������  ��λ����
#define Time_outed        (uint32_t)(Max_Range*2)*1000000/340  // ��ʱʱ�� ��λus

volatile uint8_t Ultra_Stauts = Ultra_ST_Idle; //��ǰ״̬
volatile uint32_t Ultra_High_time = 0,
				   Ultra_Start_time = 0,	
				   Ultra_Low_time = 0;	
static  float Dist_buffer[MOVAVG_SIZE];
static uint8_t Dis_index = 0;
float  Ultra_Distance = 0 ;
int16_t Ultra_health = 0; 

//���һ���µ�ֵ�� ���� �����˲�
void Ultrasonic_NewDis(float val) 
{
  if(val > (float)Max_Range)
  	return;		
  Dist_buffer[Dis_index] = val;
  Dis_index = ((Dis_index + 1) % MOVAVG_SIZE);  
}

//��ȡ���� ��ƽ��ֵ
float Ultrasonic_getAvg(float * buff, int size) {
  float sum = 0.0;
  int i;
  for(i=0; i<size; i++) {
    sum += buff[i];
  }
  return (sum / (float)size);
}

void Ultrasonic_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;   //EXTI0_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void EXTIX_Init(void)
{
	uint8_t i;
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//����IO��
	Ultrasonic_NVIC_Config(); //�ж�����
	//�ⲿ�ж�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//���ģʽ
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	//Trig
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);//ʹ��SYSCFGʱ��
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource4);//PD0���ӵ��ⲿ�ж�0
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line4; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line4);  

	GPIO_ResetBits(GPIOB, GPIO_Pin_5);//�Ȳ�Ҫ��������
	for(i=0;i<MOVAVG_SIZE;i++)
	{
				Dist_buffer[i]=0; //����У�׼���ж���
  }
}

//���� ���ͳ������źš�
void Ultrasonic_Start(void)
{
   GPIO_SetBits(GPIOB, GPIO_Pin_5);
 	 my_delay_us(10);
	 GPIO_ResetBits(GPIOB, GPIO_Pin_5);
   Ultra_Stauts = Ultra_ST_Started;	 //��״̬��
   Ultra_Start_time = micros();	//ȡʱ��
}

//�����������̣߳� �˳�����Ҫ��ʱ���á��Ը��¾���
void Ultrasonic_Routine(void)
{
	uint32_t temp;
	static uint32_t HaltTime = 0;
	static int16_t Ultra_valid = 0;
	static float  Distance;
	//�����ڵ�ǰ�¶��� ��Ӧ�Ŀ����������Ĵ����ٶ�
	/*
	��������ʵ��ܶȺ͵��������йأ����Ҳ����ʵ��¶ȡ�
	ѹǿ��״̬�������ı䡣����������ÿ��Լ�����ף�
	���¶����߶�����0��ʱ����������Ϊ331��4�ף��룬
	15��ʱΪ340�ף��룬�¶�ÿ����1�棬����Լ����0��6�ף��롣
	*/
	float  Sound_Speed ;
switch(Ultra_Stauts)//��ѯ��ǰ״̬
{ 
 case Ultra_ST_Started: 	 //�Ѿ�����������
			temp = micros();
			if((temp - Ultra_Start_time)>Time_outed)
			{
			  Ultra_Stauts = Ultra_ST_Error;	 //��ʱ�ˣ�����
			}
			break;
 case Ultra_ST_RX1: 	//�Ѿ��յ��˸ߵ�ƽ
			temp = micros();
			if((temp - Ultra_Start_time)>Time_outed)
			{
			  Ultra_Stauts = Ultra_ST_Error; //��ʱ�ˣ�����	
			}
			break;
 case Ultra_ST_RX2: //�Ѿ��յ��͵�ƽ�����Լ��������
			Sound_Speed = (332.0f+ (MS5611_Temperature/100.0f)*0.607f);//��������
			Distance = (float)(Ultra_Low_time - Ultra_High_time);  //ʱ��   ��λ us
			Distance = (Distance/2000000.0f); //���� ������һ�룬��Ҫ��ʱ�䣬��λ S
			Distance = (Distance) * Sound_Speed; //���� 

			if((Distance > (float)Max_Range) || (Distance < 0.0f))
		  {
				Ultra_Stauts = Ultra_ST_Error; //��ʱ�ˣ�����
				break; //���������ˣ���β�����Ч��
			}
			Ultrasonic_NewDis(Distance);

			Ultra_Distance = Ultrasonic_getAvg(Dist_buffer,MOVAVG_SIZE);

			if(Ultra_valid++>MOVAVG_SIZE+2)//����MOVAVG_SIZE�γ������߶���Ч����ôӦ��������������ѹ�߶ȵ�Ư��
			{ 
				MS561101BA_SetAlt(Distance);  //������ �߶���Ч���궨��ѹ�߶�
				Ultra_valid = MOVAVG_SIZE;
			}
			Ultra_health++; //�˴βɼ��߶���Ч��������++
			if(Ultra_health > 100)Ultra_health = 100;
			if(Ultra_health > 90)	//���������߶ȼƽ����ȴﵽ90%ʱ��ʹ��
				Set_Ultra_Priority(1); //MS5611�ӳ���
			else 
				Set_Ultra_Priority(0);

			Ultra_Stauts = Ultra_ST_Idle;
			break;
	case Ultra_ST_Error:
			Ultra_valid = 0;  //�������߶���Ч���� ����
			Ultra_Stauts = Ultra_ST_Idle; //������Ϣ״̬
			Ultra_health--; //�˴βɼ��߶���Ч��������--
			if(Ultra_health < 0)Ultra_health = 0;
			if(Ultra_health < 90)Set_Ultra_Priority(0);//���������ͻȻʧ�飬�Ե�Ƭ��ʹ����ѹ��
				
			break;
	case Ultra_ST_Idle:		 //��Ϣʱ�䡣���ǲ�Ҫ�ɼ���ô�졣
		 if(HaltTime == 0)
		 {
		 	HaltTime =	micros();
		 }
		 else
		 {
			temp =	micros();
			if((temp - HaltTime)>Time_outed)
			{
				Ultra_Stauts = Ultra_Restart; //��Ϣ���ˣ���������һ�βɼ�
				HaltTime = 0;
			}
		 }
		 break;
	case Ultra_Restart:	   //����һ�β��
		 Ultrasonic_Start();  //���ͳ������ź�
		 Ultra_Stauts = Ultra_ST_Started; // �ı�״̬��
		 break;
	default: 
		   Ultrasonic_Start();
		   Ultra_Stauts = Ultra_ST_Started;
		   break;
	}
}

void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4)!=RESET)
	{
	 if( GPIOB->IDR & 0x0010)
	 {	//PD0   �ߵ�ƽ��
	  Ultra_High_time = micros();
	  Ultra_Stauts = Ultra_ST_RX1; //�����յ��˸ߵ�ƽ ״̬���仯	
	 }
	 else
	 {		//�͵�ƽ
	   Ultra_Low_time = micros();
	   Ultra_Stauts = Ultra_ST_RX2;	//��������  
	 }		
	EXTI_ClearITPendingBit(EXTI_Line4);
 }
}

/*void EXTI4_IRQHandler(void)
{
	float distance;
	if(EXTI_GetITStatus(EXTI_Line4)!=RESET)
	{
	 if( GPIOB->IDR & 0x0010)
	 {	//PD0   �ߵ�ƽ��
	   Rise_time = micros();
	 }
	 else
	 {		//�͵�ƽ
	   Fall_time = micros();
		 if(Fall_time <= Rise_time)
			{
				distance = (float)(0xffffffff - Rise_time + Fall_time);//pwmin_temp = (0xffffffff - CH5.RisingTime) + CH5.FallingTime;
				distance = distance/2000000.0f;
				chao_meter = 340.0f *distance;
		  } 
		  else
			{
	      distance = (float)(Fall_time - Rise_time); //340*32*0.001*0.5=5.44
				distance = distance/2000000.0f;
				chao_meter = 340.0f *distance;
			}
	 }		
	EXTI_ClearITPendingBit(EXTI_Line4);
 }
}*/

