#include "config.h"
#include "stm32f10x.h"
#include <stm32f10x_gpio.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_bkp.h>
#include <stm32f10x_pwr.h>
#include <stm32f10x_rtc.h>

void Pin_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIOMode_TypeDef Mode,GPIOSpeed_TypeDef Speed);
void lcd_out_data4(unsigned char val);
void lcd_write_byte(unsigned char val);
void lcd_write_ascii(unsigned char c);
void lcd_write_control(unsigned char val);
void goto_cursor(unsigned char i);
void lcd_initialize(void);
void enable_lcd(void);
char busy_lcd(void); 
void lcd_print(unsigned char* str);
void lcd_printx(unsigned char *str);
unsigned char BCD_CONV(unsigned long int CONV_DATA,unsigned char CONV_CNT);
void DelayuS(vu32 nCount);
void delay(__IO u32 nCount);





GPIO_TypeDef* PORT[lcd_lines] = {	lcd_line_1_port, lcd_line_2_port, 
									lcd_line_3_port, lcd_line_4_port, 
								};
const uint16_t PIN[lcd_lines] = {	lcd_line_1_pin, lcd_line_2_pin, 
									lcd_line_3_pin, lcd_line_4_pin,
								};

RCC_ClocksTypeDef RCC_ClockFreq;


unsigned long millis;
volatile u16 Timer1;
volatile u8 Mode;
//void SysTickDelay_us(u16 dly_us);
//void SysTickDelay_ms(u16 dly_ms);
/***********************************************************************
	 Peripheral Clock Enable
************************************************************************/
void RCC_Configuration(void)
{
	/**************************************************
	Get information of RCC debugging
	Please refer contents of RCC_ClocksTypeDef structural body, after the configuration is completed when the clock,
	The inside variable value directly reflects the frequency of the operation of the various parts of the device
	***************************************************/
	RCC_GetClocksFreq(&RCC_ClockFreq);
	millis = 0;
	//SYSTICK divider - 1ms system clock interrupt
	if (SysTick_Config (SystemFrequency / 1000)) //1ms per interrupt
	while (1);
	//if (SysTick_Config (SystemFrequency / 25)) //0.04s per interrupt
	//while (1);
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);	//DMA clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC,ENABLE);	//PA and ADC1 clock
}

/*******************************************************************************
*      
        All pins used in the configuration
*
*******************************************************************************/
void GPIO_Configuration(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
  /*Analog Input*/	
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4;				//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//Analog input pins
	GPIO_Init(GPIOA, &GPIO_InitStructure);		
}

static void ADC_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;
		
	/* ADC1 configuration */
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; 	//ADC in Single Conversion Mode			 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 3;
	ADC_Init(ADC1, &ADC_InitStructure);		 //Initialization ADC1
	
	/* ADC2 configuration */
/*	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; 	//ADC in Single Conversion Mode			 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);		 //Initialization ADC2  */

	/* ADC1 regular channel1 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5); //ADC_SampleTime_28Cycles5); //ADC_SampleTime_55Cycles5 //ADC_SampleTime_239Cycles5
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5); //ADC_SampleTime_28Cycles5); //ADC_SampleTime_55Cycles5 ADC_SampleTime_239Cycles5	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));
	
	/* Enable ADC2 */
//	ADC_Cmd(ADC2, ENABLE);

	/* Enable ADC1 reset calibaration register */   
//	ADC_ResetCalibration(ADC2);
	/* Check the end of ADC2 reset calibration register */
//	while(ADC_GetResetCalibrationStatus(ADC2));

	/* Start ADC2 calibaration */
//	ADC_StartCalibration(ADC2);
	/* Check the end of ADC1 calibration */
//	while(ADC_GetCalibrationStatus(ADC2));
	
	

	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void config()
{
	RCC_Configuration();
	GPIO_Configuration();
	ADC_Mode_Config();
	Pin_Init(Relay1_Port,Relay1_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
  Pin_Init(Relay2_Port,Relay2_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
}

/****************************/
/* Strobe 4-Bit Data to LCD */
/****************************/
void lcd_out_data4(unsigned char val)
{   
  if((val&0x01)==0x01)	// Bit[0] 
  {
    LCD_D4_HI();  
  }  
  else
  {
    LCD_D4_LO();
  }
  
  if((val&0x02)==0x02)  // Bit[1] 
  {
    LCD_D5_HI();  
  }  
  else
  {
    LCD_D5_LO();
  }
  
  if((val&0x04)==0x04)  // Bit[2] 
  {
    LCD_D6_HI();  
  }  
  else
  {
    LCD_D6_LO();
  } 
  
  if((val&0x08)==0x08)  // Bit[3]
  {
    LCD_D7_HI();  
  }  
  else
  {
    LCD_D7_LO();
  } 
  DelayuS(500);	
}

/****************************/
/* Write Data 1 Byte to LCD */
/****************************/
void lcd_write_byte(unsigned char val)
{  
  lcd_out_data4((val>>4)&0x0F);							// Strobe 4-Bit High-Nibble to LCD
  enable_lcd();											// Enable Pulse
  
  lcd_out_data4(val&0x0F);				  				// Strobe 4-Bit Low-Nibble to LCD
  enable_lcd();											// Enable Pulse  

  //while(busy_lcd());      								// Wait LCD Execute Complete  
}

/****************************/
/* Write Data(ASCII) to LCD */
/****************************/
void lcd_write_ascii(unsigned char c)
{  
  LCD_RS_HI();											// RS = 1 = Data Select
  lcd_write_byte(c);		   							// Strobe 1 Byte to LCD    
}

/****************************/
/* Write Instruction to LCD */
/****************************/
void lcd_write_control(unsigned char val)
{ 
  LCD_RS_LO();											// RS = 0 = Instruction Select
  lcd_write_byte(val);									// Strobe Command Byte	    
}

/***************************/
/* Set LCD Position Cursor */
/***************************/
void goto_cursor(unsigned char i)
{
  i |= 0x80;											// Set DD-RAM Address Command
  lcd_write_control(i);  
}


void lcd_initialize(void)
{
	DelayuS(15000);										// Power-On Delay (15 mS)	

  Pin_Init(lcd_line_1_port,lcd_line_1_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
  Pin_Init(lcd_line_2_port,lcd_line_2_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
  Pin_Init(lcd_line_3_port,lcd_line_3_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
  Pin_Init(lcd_line_4_port,lcd_line_4_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
//	Pin_Init(lcd_on_off_port,lcd_on_off_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
  Pin_Init(en_port,en_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
  Pin_Init(rw_port,rw_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
  Pin_Init(rs_port,rs_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
//	GPIO_WriteBit(lcd_on_off_port,lcd_on_off_pin, Bit_SET);
	//GPIO_ResetBits(rw_port,rw_pin);
//	GPIO_WriteBit(lcd_on_off_port,lcd_on_off_pin, Bit_RESET);
	LCD_RW_LO();
	LCD_RS_LO();
	DelayuS(7200);
	lcd_write_control(0x03); 
	DelayuS(7200);
	lcd_write_control(0x03);
	DelayuS(7200);
	lcd_write_control(0x03);
  DelayuS(7200);
	lcd_write_control(0x02); 
  DelayuS(7200);
	lcd_write_control(0x02); 
  DelayuS(7200);   
  lcd_write_control(0x28);  							// Function Set (DL=0 4-Bit,N=1 2 Line,F=0 5X7)
	DelayuS(7200);
  lcd_write_control(0x0C);  							// Display on/off Control (Entry Display,Cursor off,Cursor not Blink)
	DelayuS(7200);
  lcd_write_control(0x06);  							// Entry Mode Set (I/D=1 Increment,S=0 Cursor Shift)
	DelayuS(7200);
  lcd_write_control(0x01);  							// Clear Display  (Clear Display,Set DD RAM Address=0) 
  DelayuS(15000);  	
}

/************************************/
/* Print Display Data(ASCII) to LCD */
/************************************/
void lcd_print(unsigned char *str)
{
	while(*str!='\0')
	{
		lcd_write_ascii(*str);
		str++;
		delay(1500); delay(100);
	}
}

/************************************/
/* Print Display Data(ASCII) to LCD */
/************************************/
void lcd_printx(unsigned char *str)
{
	while(*str!=0x0a) 					//(*str!='\0')
	{
		lcd_write_ascii(*str);
		str++;
		delay(1500); delay(100);
	}
}


void delay(__IO u32 nCount)
{
  for(; nCount != 0; nCount--);
} 
/******************/
/* Wait LCD Ready */
/******************/

char busy_lcd(void) //void lcd_print(unsigned char* str)
{ 
  Pin_Init(lcd_line_4_port,lcd_line_4_pin,GPIO_Mode_IPD,GPIO_Speed_50MHz);	

  LCD_RS_LO();		 									// Instruction Select
  LCD_RW_HI(); 											// Read Direction
  LCD_EN_HI();											// Start Read Busy

  DelayuS(100);	  										// Delay Before Read
  if (GPIO_ReadInputDataBit(lcd_line_4_port,lcd_line_4_pin) == Bit_SET)
  {
    LCD_EN_LO();  										// Disable Read
  	LCD_RW_LO();										// Default = Write Direction
  	
		Pin_Init(lcd_line_4_port,lcd_line_4_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);	
    return 1;											// LCD Busy Status
  }
  else
  {
    LCD_EN_LO();  										// Disable Read
  	LCD_RW_LO();										// Default = Write Direction
  	 Pin_Init(lcd_line_4_port,lcd_line_4_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);	
	  return 0;											// LCD Ready Status
  }  
}


/***********************/
/* Enable Pulse to LCD */
/***********************/
void enable_lcd(void)	 								// Enable Pulse
{  
  LCD_EN_HI();  										// Enable ON
  DelayuS(50);  
  LCD_EN_LO();  										// Enable OFF 
}

/********************************************
**Function name:SysTickDelay
**Function: using the system clock hard delay
**Note: In general, do not interrupt the function is called, otherwise there will be reentrant. Shielding the global interrupt, do not use this function
********************************************/

/*void SysTickDelay_us(u16 dly_us)
{
	Timer1=dly_us;
	while(Timer1);
} */

/********************************************
**Function name:SysTickDelay
**Function: using the system clock hard delay
**Note: In general, do not interrupt the function is called, otherwise there will be reentrant. Shielding the global interrupt, do not use this function
********************************************/

/*void SysTickDelay_ms(u16 dly_ms)
{
	Timer1=dly_ms * 1000;
	while(Timer1);
} */


void Pin_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIOMode_TypeDef Mode,GPIOSpeed_TypeDef Speed)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  /*Enable Port RCC_APB2 configurations*/
  if(GPIOx == GPIOA)
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  else if(GPIOx == GPIOB)
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  else if(GPIOx == GPIOC)
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  else if(GPIOx == GPIOD)
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  else if(GPIOx == GPIOE)
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
  else if(GPIOx == GPIOF)
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
  

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
  
  GPIO_InitStructure.GPIO_Mode = Mode;
  GPIO_InitStructure.GPIO_Speed = Speed;
  GPIO_Init(GPIOx, &GPIO_InitStructure);
}

unsigned char BCD_CONV(unsigned long int CONV_DATA,unsigned char CONV_CNT)
{
	unsigned long int TEMP_VALUE;
	unsigned char TEMP_CNT;

	for(TEMP_CNT=0;TEMP_CNT<CONV_CNT;TEMP_CNT++)
	{
		CONV_DATA					=	(((CONV_DATA/0x0A)*0x10)+(CONV_DATA%0x0A));
		TEMP_VALUE					=	CONV_DATA%0x10;
		CONV_DATA					=	CONV_DATA/0x10;
	}

	return(TEMP_VALUE+0x30);
}

void DelayuS(vu32 nCount)
{  
  while (nCount--);
}
