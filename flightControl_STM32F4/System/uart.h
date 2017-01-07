#ifndef __UART_H
#define __UART_H

#include "stm32f4xx.h"

void USART2_Init(void);
void USART2_Send1Byte(u8 byte);
void USART2_SendNByte(u8 *data, u16 num);
void USART2_SendString(u8 *str);
void Send_data_float(float axis,unsigned char flog);
void Send_data_int(int axis,unsigned char flog);
void Oled_data_int(int axis,unsigned char flog,unsigned char LIE);
void Oled_data_float(float axis,unsigned char flog,unsigned char LIE);
#endif

