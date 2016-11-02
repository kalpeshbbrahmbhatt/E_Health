//         | PA9  - USART1(Tx)      |
//         | PA10 - USART1(Rx)      |
#include "usart1.h"
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
 
void Serial1PutChar(u8 c);
void Serial1_PutString(u8 *s);
void Serial2PutChar(u8 c);
void Serial2_PutString(u8 *s);

void USART1_Config(void)
{	
	GPIO_InitTypeDef USART1_GPIO_init;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	//Enable PCI clock
	//Initializing a push-pull output PA9
	USART1_GPIO_init.GPIO_Speed=GPIO_Speed_50MHz;
	USART1_GPIO_init.GPIO_Mode=GPIO_Mode_AF_PP;		//Push-Pull
	USART1_GPIO_init.GPIO_Pin=GPIO_Pin_9;
	GPIO_Init(GPIOA,&USART1_GPIO_init);
	//PA10 initialize a floating input
	USART1_GPIO_init.GPIO_Mode=GPIO_Mode_IN_FLOATING;	//Floating input
	USART1_GPIO_init.GPIO_Pin=GPIO_Pin_10;
	GPIO_Init(GPIOA,&USART1_GPIO_init);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);	//Enable USART1 clock
	// USART1 mode config 
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);		//Initialization USART1
	USART_Cmd(USART1, ENABLE);				 //Enable	 
}

void USART2_Config(void)
{	
	GPIO_InitTypeDef USART2_GPIO_init;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	//Enable PCI clock
	//Initializing a push-pull output PA9
	USART2_GPIO_init.GPIO_Speed=GPIO_Speed_50MHz;
	USART2_GPIO_init.GPIO_Mode=GPIO_Mode_AF_PP;		//Push-Pull
	USART2_GPIO_init.GPIO_Pin=GPIO_Pin_2;
	GPIO_Init(GPIOA,&USART2_GPIO_init);
	//PA10 initialize a floating input
	USART2_GPIO_init.GPIO_Mode=GPIO_Mode_IN_FLOATING;	//Floating input
	USART2_GPIO_init.GPIO_Pin=GPIO_Pin_3;
	GPIO_Init(GPIOA,&USART2_GPIO_init);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);	//Enable USART2 clock
	// USART2 mode config 
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);		//Initialization USART2
	USART_Cmd(USART2, ENABLE);				 //Enable	 
}


/*Reorientation of the C library function printf to USART1, the printf call*/
int fputc(int ch, FILE *f)
{
	/* Printf contents will be sent to the serial port */
	USART_SendData(USART1, (unsigned char) ch);
	while (!(USART1->SR & USART_FLAG_TXE));
	return (ch);
}


/*
 * Function name: itoa 
 * Description: Converts the integer data to a string 
 * Input: -radix = 10 decimal representation, the result is a number of other plastic 0 
 * -value to convert 
 * -buf converted string 
 * -radix = 10 
 * Output: None 
 * returns: None * Call: The USART1_printf () call
 */
static char *itoa(int value,char *string,int radix)
{
int i,d;
int flag = 0;
char *ptr = string;

/* This implementation only works for decimal numbers. */
if (radix != 10)
{
	*ptr = 0;
	return string;
}

if (!value)
{
	*ptr++ = 0x30;
	*ptr = 0;
	return string;
}

/* if this is a negative value insert the minus sign. */
if (value < 0)
{
	*ptr++ = '-';
/* Make the value positive. */
	value *= -1;
}

for (i = 10000; i > 0; i /= 10)
{
	d = value / i;
	if (d || flag)
	{
		*ptr++ = (char)(d + 0x30);
		value -= (d * i);
		flag = 1;
	}
}

/* Null terminate the string. */
*ptr = 0;
return string;
} 


/*
 * Function name: USART1_printf
 * Description: The formatted output, similar to the C library printf, but there did not use C library
 * Input: -USARTx serial channel, where only uses serial port 1, that USART1
 * -Data Port content to be sent to a pointer
 * Other parameters -...
 * Output: None
 * Returns: None
 * Call: External call
 * Typical applications USART1_printf (USART1, "\ r \ n this is a demo \ r \ n");
 *            		 USART1_printf( USART1, "\r\n %d \r\n", i );
 *            		 USART1_printf( USART1, "\r\n %s \r\n", j );
 */
void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...)
{
	const char *s;
  int d;   
  char buf[16];

  va_list ap;
  va_start(ap, Data);

	while ( *Data != 0)     // Determine whether the arrival string terminator
	{				                          
		if ( *Data == 0x5c )  //'\'
		{									  
			switch ( *++Data )
			{
				case 'r':							          //Carriage
					USART_SendData(USARTx, 0x0d);
					Data ++;
					break;

				case 'n':							          //Newline
					USART_SendData(USARTx, 0x0a);	
					Data ++;
					break;
				
				default:
					Data ++;
				    break;
			}			 
		}
		else if ( *Data == '%')
		{									  //
			switch ( *++Data )
			{				
				case 's':										  //String
					s = va_arg(ap, const char *);
          for ( ; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
          }
					Data++;
          break;

        case 'd':										//Decimal
          d = va_arg(ap, int);
          itoa(d, buf, 10);
          for (s = buf; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
          }
					Data++;
          break;
				 default:
						Data++;
				    break;
			}		 
		} /* end of else if */
		else USART_SendData(USARTx, *Data++);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
}

/* Usart functions ---------------------------------  ------------------------*/
/*******************************************************************************
* Function Name  : Serial1PutChar
* Description    : Print a character on the HyperTerminal
* Input          : - c: The character to be printed
* Output         : None
* Return         : None
*******************************************************************************/
void Serial1PutChar(u8 c)
{
        USART_SendData(USART1, c);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

/*******************************************************************************
* Function Name  : Serial_PutString
* Description    : Print a string on the HyperTerminal
* Input          : - s: The string to be printed
* Output         : None
* Return         : None
*******************************************************************************/
void Serial1_PutString(u8 *s)
{
        while (*s != '\0')
        {
        Serial1PutChar(*s);
        s ++;
        }
}

/* Usart functions ---------------------------------  ------------------------*/
/*******************************************************************************
* Function Name  : Serial1PutChar
* Description    : Print a character on the HyperTerminal
* Input          : - c: The character to be printed
* Output         : None
* Return         : None
*******************************************************************************/
void Serial2PutChar(u8 c)
{
        USART_SendData(USART2, c);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/*******************************************************************************
* Function Name  : Serial_PutString
* Description    : Print a string on the HyperTerminal
* Input          : - s: The string to be printed
* Output         : None
* Return         : None
*******************************************************************************/
void Serial2_PutString(u8 *s)
{
        while (*s != '\0')
        {
        Serial2PutChar(*s);
        s ++;
        }
}
