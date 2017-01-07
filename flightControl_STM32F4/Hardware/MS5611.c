/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��ms5611.c
 * ����    ��ms5611����         
 * ʵ��ƽ̨��HT�ɿ�
 * ��汾  ��ST3.5.0
 * ����    ��Air Nano Team 
**********************************************************************************/
#include "iic.h"
#include "math.h"
#include "MS5611.h"
#include "delay.h"
#include "timer.h"

#define MS5611Press_OSR  MS561101BA_OSR_4096  //��ѹ��������
#define MS5611Temp_OSR   MS561101BA_OSR_4096  //�¶Ȳ�������

// ��ѹ��״̬��
#define SCTemperature  0x01	  //��ʼ �¶�ת��
#define CTemperatureing  0x02  //����ת���¶�
#define SCPressure  0x03	  //��ʼת�� ��ѹ
#define SCPressureing  0x04	  //����ת����ѹֵ

#define MOVAVG_SIZE  10	   //������� 10������

const float  MS5611_Lowpass = 7.9577e-3f;  //10hz

static uint8_t  Now_doing = SCTemperature;	//��ǰת��״̬
static uint16_t PROM_C[MS561101BA_PROM_REG_COUNT]; //�궨ֵ���
static uint32_t Current_delay;	   //ת����ʱʱ�� us 
static uint32_t Start_Convert_Time;   //����ת��ʱ�� ʱ�� us 
static int32_t  tempCache;
uint8_t ALT_Updated = 0; //��ѹ�Ƹ߶ȸ�����ɱ�־��
uint8_t Ultra_Priority = 0;
static float Alt_Offset_cm = 0;
static float avg_Pressure;
float ALT_Update_Interval = 0.0; //���θ߶Ȳ�����֮���ʱ����

//units (Celsius degrees*100, mbar*100  ).
//��λ [�¶� 0.01��] [��ѹ ��]  [�߶�0.01��] 
float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude;

// ��ʱ��  ��λ us 	  ��ͬ�Ĳ������ȶ�Ӧ��ͬ����ʱֵ
uint32_t MS5611_Delay_us[9] = {
	1500,//MS561101BA_OSR_256 0.9ms  0x00
	1500,//MS561101BA_OSR_256 0.9ms  
	2000,//MS561101BA_OSR_512 1.2ms  0x02
	2000,//MS561101BA_OSR_512 1.2ms
	3000,//MS561101BA_OSR_1024 2.3ms 0x04
	3000,//MS561101BA_OSR_1024 2.3ms
	5000,//MS561101BA_OSR_2048 4.6ms 0x06
	5000,//MS561101BA_OSR_2048 4.6ms
	11000,//MS561101BA_OSR_4096 9.1ms 0x08
};

// FIFO ����					
static float Temp_buffer[MOVAVG_SIZE],Press_buffer[MOVAVG_SIZE],Alt_buffer[MOVAVG_SIZE];
static uint8_t temp_index=0,press_index=0; //����ָ��

//���ó������߶ȼ����ȡ����������߶ȼ���Чʱ����ʹ����ѹ�Ƶĸ߶�ֵ��
void Set_Ultra_Priority(uint8_t enable)
{
	Ultra_Priority = enable;	
}
//���һ���µ�ֵ�� �¶ȶ��� �����˲�
void MS561101BA_NewTemp(float val)
{
  Temp_buffer[temp_index] = val;
  temp_index = (temp_index + 1) % MOVAVG_SIZE;
}
//���һ���µ�ֵ�� ��ѹ���� �����˲�
void MS561101BA_NewPress(float val) 
{
  Press_buffer[press_index] = val;
  press_index = (press_index + 1) % MOVAVG_SIZE;
}
//���һ���µ�ֵ�� �߶ȶ��� �����˲�
void MS561101BA_NewAlt(float val) 
{
  int16_t i;
  static uint32_t alt_lastupdate , temp;
  temp = micros(); //�������θ߶ȵĸ��¼��
	if(temp > alt_lastupdate)
    ALT_Update_Interval = ((float)(temp - alt_lastupdate))/1000000.0f;
	else
		ALT_Update_Interval = ((float)(0xffffffff + temp - alt_lastupdate))/1000000.0f;
  alt_lastupdate = temp;
  for(i=1;i<MOVAVG_SIZE;i++)
  Alt_buffer[i-1] = Alt_buffer[i];
  Alt_buffer[MOVAVG_SIZE-1] = val;
}

//ȡ��ѹ�Ƶ�D�仯��
float MS5611BA_Get_D(void)
{
	float new=0,old=0;
	int16_t i;
	for(i=0;i<MOVAVG_SIZE/2;i++)
		old += Alt_buffer[i];
	old /= (MOVAVG_SIZE/2);

	for(i=MOVAVG_SIZE/2;i<MOVAVG_SIZE;i++)
	    new += Alt_buffer[i];
	new /= (MOVAVG_SIZE/2);

	return new - old;
}

//��ȡ���� ��ƽ��ֵ
float MS561101BA_getAvg(float * buff, int size) 
{
  float sum = 0.0;
  int i;
  for(i=0; i<size; i++) {
    sum += buff[i];
  }
  return sum / size;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_readPROM(void)
*��������:	    ��ȡ MS561101B �Ĺ����궨ֵ
��ȡ ��ѹ�Ƶı궨ֵ  ���������¶Ⱥ���ѹ�Ķ���
*******************************************************************************/
void MS561101BA_readPROM(void) 
{
  u8  inth,intl;
  int i;
  for (i=0;i<MS561101BA_PROM_REG_COUNT;i++) {
		IIC_Start();
    	IIC_Send_Byte(MS5611_ADDR);
		IIC_Wait_Ack();
    	IIC_Send_Byte(MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
		IIC_Wait_Ack();	
    	IIC_Stop();
		delay_us(5);
   		IIC_Start();
		IIC_Send_Byte(MS5611_ADDR+1);  //�������ģʽ	
		delay_us(1);
		IIC_Wait_Ack();
		inth = IIC_Read_Byte(1);  //��ACK�Ķ�����
		delay_us(1);
		intl = IIC_Read_Byte(0);	 //���һ���ֽ�NACK
		IIC_Stop();
    PROM_C[i] = (((uint16_t)inth << 8) | intl);
  }
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_reset(void)
*��������:	    ���͸�λ��� MS561101B 
*******************************************************************************/
void MS561101BA_reset(void) 
{
	IIC_Start();
    IIC_Send_Byte(MS5611_ADDR); //д��ַ
	IIC_Wait_Ack();
    IIC_Send_Byte(MS561101BA_RESET);//���͸�λ����
	IIC_Wait_Ack();	
    IIC_Stop();
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_startConversion(uint8_t command)
*��������:	    ��������ת����� MS561101B
��ѡ�� ת������Ϊ MS561101BA_D1  ת����ѹ
				  MS561101BA_D2  ת���¶�	 
*******************************************************************************/
void MS561101BA_startConversion(uint8_t command) 
{
  // initialize pressure conversion
  IIC_Start();
  IIC_Send_Byte(MS5611_ADDR); //д��ַ
  IIC_Wait_Ack();
  IIC_Send_Byte(command); //дת������
  IIC_Wait_Ack();	
  IIC_Stop();
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned long MS561101BA_getConversion(void)
*��������:	    ��ȡ MS561101B ��ת�����	 
*******************************************************************************/
unsigned long MS561101BA_getConversion(void) 
{
		unsigned long conversion = 0;
		u8 temp[3];
		IIC_Start();
    IIC_Send_Byte(MS5611_ADDR); //д��ַ
		IIC_Wait_Ack();
    IIC_Send_Byte(0);// start read sequence
		IIC_Wait_Ack();	
    IIC_Stop();
		
		IIC_Start();
		IIC_Send_Byte(MS5611_ADDR+1);  //�������ģʽ	
		IIC_Wait_Ack();
		temp[0] = IIC_Read_Byte(1);  //��ACK�Ķ�����  bit 23-16
		temp[1] = IIC_Read_Byte(1);  //��ACK�Ķ�����  bit 8-15
		temp[2] = IIC_Read_Byte(0);  //��NACK�Ķ����� bit 0-7
		IIC_Stop();
		conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
		return conversion;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_init(void)
*��������:	    ��ʼ�� MS561101B 
*******************************************************************************/
void MS561101BA_init(void) 
{  
  MS561101BA_reset(); // ��λ MS561101B 
  delay_ms(100); // ��ʱ 
  MS561101BA_readPROM(); // ��ȡEEPROM �еı궨ֵ ����	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_GetTemperature(void)
*��������:	    ��ȡ �¶�ת�����	 
*******************************************************************************/
void MS561101BA_GetTemperature(void)
{	
	tempCache = MS561101BA_getConversion();	
}

float Alt_offset_Pa=0; //�����0��ʱ ��Ӧ����ѹֵ  ���ֵ����ϵ�ʱ����ѹֵ 
uint8_t  Covert_count=0;
/**************************ʵ�ֺ���********************************************
*����ԭ��:		float MS561101BA_get_altitude(void)
*��������:	    ����ǰ����ѹֵת�� �߶ȡ�	 
*******************************************************************************/
float MS561101BA_get_altitude(void)
{
	static float Altitude;
	float temp;
	if(Alt_offset_Pa==0){ // �Ƿ��ʼ����0����ѹֵ��
		if(Covert_count++<50);  //�ȴ���ѹ�ȶ� �� ��ȡ����ʱ����ѹֵ
		else Alt_offset_Pa = MS5611_Pressure; //�� ��ǰ��ѹֵ����� 0 ��ʱ����ѹ
		avg_Pressure = MS5611_Pressure;
		Altitude = 0; //�߶� Ϊ 0
		return Altitude;
	}
	//��������� �ϵ�ʱ��λ�õ� �߶�ֵ ��
	temp = 4433000.0 * (1 - pow((MS5611_Pressure / Alt_offset_Pa), 0.1903));
	temp = temp + Alt_Offset_cm ;  //��ƫ��
	Altitude = Altitude +	  //��ͨ�˲�   20hz
	 (ALT_Update_Interval/(ALT_Update_Interval + MS5611_Lowpass))*(temp - Altitude);
	if(Ultra_Priority == 0)	//���������ȣ���ʹ����ѹ��
		MS561101BA_NewAlt(Altitude);
	return (MS561101BA_getAvg(Alt_buffer,MOVAVG_SIZE));
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_ResetAlt(void)
*��������:	    ����ǰ����ѹ��Ϊ0��ʱ����ѹ��	 
*******************************************************************************/
void MS561101BA_ResetAlt(void)
{
	Alt_offset_Pa = MS5611_Pressure; //�� ��ǰ��ѹֵ����� 0 ��ʱ����ѹ	
	Alt_Offset_cm = 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_SetAlt(void)
*��������:	    ����ǰ����ѹ��Ϊ Current ��ʱ����ѹ��	 
*******************************************************************************/
void MS561101BA_SetAlt(float Current)
{
	Alt_offset_Pa = (avg_Pressure); //�� ��ǰ��ѹֵ����� 0 ��ʱ����ѹ	
	Alt_Offset_cm = Current*100.0f; //��ת�� CM
	MS561101BA_NewAlt(Current*100.0f);	 //�µĸ߶�ֵ
	ALT_Updated = 1; //�߶ȸ��� ��ɡ�
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_getPressure(void)
*��������:	    ��ȡ ��ѹת����� ������������	 
*******************************************************************************/
void MS561101BA_getPressure(void) 
{
	int64_t off,sens;
	int64_t TEMP,T2,Aux_64,OFF2,SENS2;  // 64 bits
	int32_t rawPress = MS561101BA_getConversion();
	int64_t dT  = tempCache - (((int32_t)PROM_C[4]) << 8);
	float temp;
	TEMP = 2000 + (dT * (int64_t)PROM_C[5])/8388608;
	off  = (((int64_t)PROM_C[1]) << 16) + ((((int64_t)PROM_C[3]) * dT) >> 7);
	sens = (((int64_t)PROM_C[0]) << 15) + (((int64_t)(PROM_C[2]) * dT) >> 8);
	
	if (TEMP < 2000) // second order temperature compensation,�¶Ȳ���
	{  
		T2 = (((int64_t)dT)*dT) >> 31;
		Aux_64 = (TEMP-2000)*(TEMP-2000);
		OFF2 = (5*Aux_64)>>1;
		SENS2 = (5*Aux_64)>>2;
		TEMP = TEMP - T2;
		off = off - OFF2;
		sens = sens - SENS2;
	}

	MS561101BA_NewPress((((((int64_t)rawPress) * sens) >> 21) - off) / 32768);
    MS5611_Pressure = MS561101BA_getAvg(Press_buffer,MOVAVG_SIZE); //0.01mbar����ƽ��ֵ
	
	avg_Pressure = avg_Pressure + (MS5611_Pressure - avg_Pressure)*0.1f;//�����˲�

	MS561101BA_NewTemp(TEMP);
	MS5611_Temperature = MS561101BA_getAvg(Temp_buffer,MOVAVG_SIZE); //0.01c
	
	temp = MS561101BA_get_altitude(); // 0.01meter
							  
	MS5611_Altitude = MS5611_Altitude +	  //��ͨ�˲�   20hz
	 (ALT_Update_Interval/(ALT_Update_Interval + MS5611_Lowpass))*(temp - MS5611_Altitude);
	
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS5611BA_Routing(void)
*��������:	    MS5611BA �����г��� ����Ҫ���ڵ��� �Ը�����ѹֵ���¶�ֵ 	 
*******************************************************************************/
void MS5611BA_Routing(void) 
{
	switch(Now_doing){ //��ѯ״̬ ������������ ����Щʲô��
		case SCTemperature:  //�����¶�ת��
			MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);
			Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//ת��ʱ��
			Start_Convert_Time = micros(); //��ʱ��ʼ
			Now_doing = CTemperatureing;//��һ��״̬
			break;
		case CTemperatureing:  //����ת���� 
			if((micros()-Start_Convert_Time) > Current_delay){ //��ʱʱ�䵽����
			MS561101BA_GetTemperature(); //ȡ�¶�	
			Now_doing = SCPressure;	
			}
			break;
		case SCPressure: //������ѹת��
			MS561101BA_startConversion(MS561101BA_D1 + MS5611Press_OSR);
			Current_delay = MS5611_Delay_us[MS5611Press_OSR];//ת��ʱ��
			Start_Convert_Time = micros();//��ʱ��ʼ
			Now_doing = SCPressureing;//��һ��״̬
			break;
		case SCPressureing:	 //����ת����ѹֵ
			if((micros()-Start_Convert_Time) > Current_delay){ //��ʱʱ�䵽����
			MS561101BA_getPressure();  //���� 	
			ALT_Updated = 1; //�߶ȸ��� ��ɡ�
			Now_doing = SCTemperature; //��ͷ����	
			}
			break;
		default: 
			Now_doing = SCTemperature;
			break;
	}
}

//------------------End of File----------------------------
