#include "config.h"
#include "oximeter.h"
#include <stm32f10x.h>


#define Theta 0.6

void delayM(int jj);
void ResetValue(void);
void Init_Oxi(void);
void read_SPO2(void)
{
	ResetValue();
	GPIO_ResetBits(Led1_port,Led1_pin);
	delayM(20000);
	Mode = 1;
	while(!((lastcount>average )&& (count<average)) ){ }
	GPIO_ResetBits(Led1_port,Led1_pin);
	ResetValue();
	while(!((lastcount>average )&& (count<average)) ){ }
	Mode = 0;
	Rmax = maxTemp;
	Rmin = minTemp;
	delayM(200);
	HeartR_frq = 1/(0.04*interrupts_counter);
	Mode = 1;
	HeartRate = HeartR_frq * 60;
	if(HeartRate> 60 && HeartRate< 120){
	HeartR = Theta * HeartRate + (1 - Theta)*LastHeartRate; //Use theta to smooth
	LastHeartRate = HeartR;
	}
	GPIO_SetBits(Led1_port,Led1_pin);
	Mode = 0;
	/*ResetValue();
	GPIO_ResetBits(Led2_port,Led2_pin);
	delayM(20000);
	Mode = 1;
	while(!((lastcount>average )&& (count<average)) ){ }
	GPIO_ResetBits(Led2_port,Led2_pin);
	ResetValue();
	while(!((lastcount>average )&& (count<average)) ){ }
	Mode = 0;
	IRmax = maxTemp;
	IRmin = minTemp;
	delayM(200);
	GPIO_SetBits(Led2_port,Led2_pin); */
		
	R = (Rmax - Rmin);
	Spo2 = (R-180)*0.01 +97.838;
	//(int)
	Sp02_int = (int)Spo2; //float Spo2 to int Spo2_int
}

void delayM(int jj)
{
	for(;jj>0;jj--)
	{}
}

void ResetValue(void)
{
	maxTemp = 0;
	minTemp = 1023;
	count = 0;
	lastcount =0;
	interrupts_counter = 0;
}

void Init_Oxi(void)
{
	Pin_Init(Led1_port,Led1_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	Pin_Init(Led2_port,Led2_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	GPIO_SetBits(Led1_port,Led1_pin);
	GPIO_SetBits(Led2_port,Led2_pin);
}
