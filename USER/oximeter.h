#include "stm32f10x.h"
#include "config.h"

extern void read_SPO2(void);
extern void delayM(int jj);
extern void ResetValue(void);
extern void Init_Oxi(void);

#define Led1_port					GPIOA
#define Led1_pin					GPIO_Pin_12
#define Led2_port					GPIOA
#define Led2_pin					GPIO_Pin_11

extern unsigned char  SystemBuf[];  //Save export receive data
extern GPIO_TypeDef* PORT[];
extern const uint16_t PIN[];
//extern struct Date_s s_DateStructVar;
extern GPIO_TypeDef* Port[];
extern const uint16_t Pin[];
