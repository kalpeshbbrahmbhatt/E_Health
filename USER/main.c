//Description: receiving the serial assistant, baud rate notes 115200
// Dual ADC mode, the results of the conversion are stored in ADC_DR, and 16 for the ADC2 high and low of 16 ADC1
#include "stm32f10x.h"
#include "config.h"
#include "usart1.h"
#include "E_Mon.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

void SerialPrint(void);
void Drive_Relay(void);

// Software delay
void Delay(__IO u32 nCount)
{
  for(; nCount != 0; nCount--);
} 

u16 Get_ADC1_Convert_Value(u8 ADC_Channel_x);

int main(void)
{
  //u8 i,tmp,*p,id[8];
	Delay(0xff);
  SystemInit();
  config();
  USART1_Config();
	USART2_Config();
	lcd_initialize();
	Relay1_Off();
	Relay2_Off();
	//lcd_initialize();
	voltage(ADC_Channel_0, 280.26, 1.7);  // Voltage: input pin, calibration, phase_shift   2, 234.26, 1.7, //49000.26, 7.5
  current1(ADC_Channel_1, 15.1);       // Current: input pin, calibration.  111.1
	current2(ADC_Channel_4, 15.1);       // Current: input pin, calibration.
	lcd_write_control(0x80);
	lcd_print("      EMON      ");
	Delay(0xfffff);
	lcd_write_control(0x80);
	lcd_print("                ");
	lcd_write_control(0xC0);
	lcd_print("                ");
  while (1)
  {
		calcVI(20,2000);         // Calculate all. No.of half wavelengths (crossings), time-out //2000
		SerialPrint();
		Delay(0xfffff);  		// Delay
		Drive_Relay();
		
  }
}

void SerialPrint(void)
{
	u8 buff[40];
	unsigned int n;
	//n = sprintf(buff,"%3.2f \n", realPower);
	//Serial_PutString(buff);
	//n = sprintf(buff,"%3.2f \n", apparentPower);
	//Serial_PutString(buff);
	n = sprintf(buff,"%3.2f\n", Vrms);
	Serial1_PutString(buff);
	Serial2_PutString(buff);
	lcd_write_control(0x80);
	//buff[6] = '\0';
	lcd_printx(buff);
	n = sprintf(buff,"%3.2f\n", I1rms);
	Serial1_PutString(buff);
	Serial2_PutString(buff);
	//buff[4] = '\0';
	//lcd_write_control(0xC0);
	lcd_print(" ");
	lcd_printx(buff);
	n = sprintf(buff,"%3.2f\n", I2rms);
	Serial1_PutString(buff);
	Serial2_PutString(buff);
	//buff[4] = '\0';
  lcd_print(" ");
	lcd_printx(buff);
	n = sprintf(buff,"%3.2f\n", Watt1);
	Serial1_PutString(buff);
	Serial2_PutString(buff);
	lcd_write_control(0xC0);
	lcd_print("                ");
	lcd_write_control(0xC0);
	lcd_printx(buff);
	n = sprintf(buff,"%3.1f\n", Watt2);
	Serial1_PutString(buff);
	Serial2_PutString(buff);
	lcd_print(" ");
	lcd_printx(buff);
	n = sprintf(buff,"%3.2f\n", powerFactor);
	Serial1_PutString(buff);
	Serial2_PutString(buff);
	n = sprintf(buff,"%3.2f\n", pf);
	Serial1_PutString(buff);
	Serial2_PutString(buff);
	Serial1_PutString("\n");
	Serial2_PutString("\n");
}

void Drive_Relay(void)
{
	if((millis > 60000) && (millis < 120000))
    {	Relay1_On();}
	else
		{ Relay1_Off(); }
	if((millis > 180000) && (millis < 240000))
		{	Relay2_On();}
	else
		{ Relay2_Off(); }
}

