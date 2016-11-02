#include "stm32f10x.h"
#include <stdio.h>

void USART1_Config(void);
void USART2_Config(void);
int fputc(int ch, FILE *f);
//void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...);
void Serial1PutChar(u8 c);
void Serial1_PutString(u8 *s);
void Serial2PutChar(u8 c);
void Serial2_PutString(u8 *s);
