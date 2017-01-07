
#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"
#include "uart.h"
#include "mpu6050.h"
#include "stdio.h"
#include "iic.h"
#include "oled.h"
#include "control.h"
#include "timer.h"
#include "string.h"
#include "led.h"
#include "moto.h"
#include "intc.h"
#include "pwmin.h"
#include "MS5611.h"

/*×¢Òâ£ºPB3,PB4,PA13~PA15²»ÄÜÊ¹ÓÃ£¬ÓÃÓÚJLINK£¬·ñÔò½«ÎÞ·¨ÏÂÔØµ÷ÊÔ*/
void RCC_Config(void);
void OLED_show(void);
unsigned char mpu6050_err;
extern u8  TIM5CH1_CAPTURE_STA; 	 	//²¶»ñ×´Ì¬ 	     	 	 	 
extern u32 	TIM5CH1_CAPTURE_VAL; //²¶»ñÖµ
long long temp=0; 
extern float MS5611_Pressure,MS5611_Altitude,MS5611_Temperature;
extern unsigned char Ctrol_flag,Ultrasonic_flag,MS5611BA_flag;
unsigned char show_angle = 1;
extern int fly_rol,fly_pit,fly_oil,fly_yaw,fly_gao;
unsigned char abc[6]={0};
extern float PIT,ROL,YAW,GAO;
//extern volatile uint32_t Rise_time,Fall_time;//²âÊÔ³¬Éù²¨×¨ÓÃ
int main()
{
	//RCC_Config();
	SystemInit();
	Systick_Init();//system tick timer£¬ÏµÍ³½ÚÅÄ¶¨Ê±Æ÷
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); //ÖÐ¶ÏÓÅÏÈ¼¶·Ö×é	
	LED_Init();
	PCout(2)=0;
	
	OLED_Init();//PA8~12
	OLED_Show_FONT_6x8(1,1,"P:");
	OLED_Show_FONT_6x8(1,3,"O.00");
  OLED_Show_FONT_6x8(1,12,"R:");
	OLED_Show_FONT_6x8(1,14,"O.00");
	
	OLED_Show_FONT_6x8(2,1,"Y:");
	OLED_Show_FONT_6x8(2,3,"O.00");
	OLED_Show_FONT_6x8(2,12,"H:");
	OLED_Show_FONT_6x8(2,14,"O.00");
	
	OLED_Show_FONT_6x8(4,1,"THR:");
	OLED_Show_FONT_6x8(4,5,"O.00");
	OLED_Show_FONT_6x8(4,12,"GAO:");
	OLED_Show_FONT_6x8(4,16,"O.00");
	myPid_init();  //PID parqameter initalize
  IIC_Init();//PB6/PB7
	mpu6050_err = 1;
	MPU6050_Init();			//6050 initalize
	MPU6050_Test();			//TEST initalize
	
  MPU6050_OFFESET(); //¼ÓËÙ¶È¼ÆÁãÆ¯Ð£×¼
	Gyro_OFFEST();  //ÍÓÂÝÒÇÁãÆ¯Ð£×¼
	
  MS561101BA_init();//³õÊ¼»¯ÆøÑ¹¼Æ
	//MS5611_OFFEST();//ÆøÑ¹¼ÆÐ£×¼
  Moto_Init();//PWM³õÊ¼»¯,PD12~15
	//OLED_Show_FONT_6x8(2,14,"complete");
	
  USART2_Init();//PA2/PA3£¬ÓÅÏÈ¼¶1£¬1,TX,RX
	EXTIX_Init();//³¬Éù²¨£¬ÓÅÏÈ¼¶0¡¢0,×î¸ß
	Initial_System_Timer();//ÓÃÓÚ¼ÆÊ±¡¢³¬Éù²¨¡¢ÂË²¨¶¼ÒªÓÃ
	TIM2_Init();//Ö÷¶¨Ê±Æ÷1¡¢2£¬µÚ¶þ¸ß	
	
  //Tim_Pwm_In_Init();//PB0,PB1,PB4,PB5,TIM3

 //Moto_PwmRflash(1400,1400,1400,1400);
	while(1)
	{	//Send_data_float(chao_meter,'c');
		
		if(Ctrol_flag == 1)
		{
			PCout(2)=~PCout(2);
		  Get_Attitude();           //×ËÌ¬½áËã
//			ROL = (float) fly_rol/12;
//		  PIT = (float) fly_pit/12;
//		  YAW = (float) fly_yaw/12;
		  GAO = (float) fly_gao/30;
		  ZHUKONG(PIT,ROL,YAW,GAO);	//Ö÷¿Øº¯Êý
			OLED_show();
			Ctrol_flag = 0;
		}
		if(Ultrasonic_flag == 1)
		{
			Ultrasonic_Routine();
			Ultrasonic_flag = 0;
		}
		if(MS5611BA_flag == 1)
		{
			MS5611BA_Routing();
			MS5611BA_flag = 0;
		}
		PCout(2)=~PCout(2);
		
		//printf("%c",'n');
    //printf("%f",chao_meter);
//	printf("%f",MS5611_Pressure);//²âÊÔ³¬Éù²¨×¨ÓÃ
//	printf("%s","  "); 
//	printf("%f",MS5611_Altitude);//²âÊÔ³¬Éù²¨×¨ÓÃ
//	printf("%s","  "); 
//	printf("%f",Ultra_Distance);
// 	printf("%c",'\n'); 
 	/*printf("%d",Rise_time);//²âÊÔ³¬Éù²¨×¨ÓÃ
	printf("%s","  "); 
	printf("%d",Fall_time);
	printf("%s","  "); 
	printf("%d",Fall_time-Rise_time);
  printf("%c",'\n');*/ 		

	}
}
void OLED_show(void)
{
	  if(show_angle == 1)
		{
		 Oled_data_float(angle.pitch,1,3);
		}
		else if(show_angle == 2)
		{
			Oled_data_float(angle.roll,1,14);
		}
		else if(show_angle == 3)
		{
		 Oled_data_float(angle.yaw,2,3);
		}
		else if(show_angle == 4)
		{
			Oled_data_float(MS5611_Altitude,2,15);			
		}
		else if(show_angle == 5)
		{
		  Oled_data_int(fly_oil,4,5);
		}
		else if(show_angle == 6)
		{
		  Oled_data_int(fly_gao,4,16);
			show_angle = 0;
		}
		
	 	show_angle++;
}
void RCC_Config(void)
 { 
	uint32_t           PLL_M;       
  uint32_t           PLL_N; 
  uint32_t           PLL_P; 
  uint32_t           PLL_Q; 

  RCC_DeInit();              //RCC¼Ä´æÆ÷³õÊ¼»¯
  RCC_HSEConfig(RCC_HSE_ON);  //Ê¹ÓÃÍâ²¿Ê±ÖÓ
	 
  if (RCC_WaitForHSEStartUp() == SUCCESS) //µÈ´ýÍâ²¿Ê±ÖÓÆô¶¯
  {
		PLL_M = 8; // PLL_VCO input clock = (HSE_VALUE or HSI_VALUE / PLL_M)£¬PLL_VCO input clock = 1MHz 
		PLL_N = 336;// PLL_VCO output clock = (PLL_VCO input clock) * PLL_N£¬PLL_VCO output clock = 336 MHz
		PLL_P = 2; //System Clock = (PLL_VCO output clock)/PLL_P £¬SystemClock = 168 Mhz
		PLL_Q = 7; // /ÅäÖÃSD,USBÊ±ÖÓÆµÂÊ£¬Ä¬ÈÏ
		
    RCC_PLLCmd(DISABLE);                    //Æô¶¯PLLÇ°Ó¦¹Ø±ÕPLL
		RCC_PLLConfig(RCC_PLLSource_HSE, PLL_M, PLL_N, PLL_P, PLL_Q); 
		RCC_PLLCmd(ENABLE);        //PLLÊ±ÖÓ¿ªÆô	
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){} //µÈ´ýPLLÊ±ÖÓ×¼±¸ºÃ
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while (RCC_GetSYSCLKSource() != 0x08){};
			
    RCC_HCLKConfig(RCC_SYSCLK_Div1);     //HCLK(AHB)Ê±ÖÓÎªÏµÍ³Ê±ÖÓµÄ1·ÖÆµ
    RCC_PCLK1Config(RCC_HCLK_Div4);     //PCLK1(APB1)Ê±ÖÓÎªHCLKÊ±ÖÓµÄ4·ÖÆµ£¬ÔòTIM2Ê±ÖÓÎªHCLKµÄ4·ÖÆµ
    RCC_PCLK2Config(RCC_HCLK_Div2);     //PCLK2(APB2)Ê±ÖÓÎªHCLKÊ±ÖÓ2·ÖÆµ
    //RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7); //PLLÊ±ÖÓÅäÖÃ£¬Ïê¼ûsystem_stm43f4xx.c’ Line149
  }
	
 }



 
 
 
 
 
 
 
 
