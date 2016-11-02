#include "stm32f10x.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_rtc.h"

extern unsigned long millis;
extern volatile u16 Timer1;
extern volatile u8 Mode;
//extern void SysTickDelay_us(u16 dly_us);
//extern void SysTickDelay_ms(u16 dly_ms);

void config(void);

extern void Pin_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIOMode_TypeDef Mode,GPIOSpeed_TypeDef Speed);
extern void lcd_out_data4(unsigned char val);
extern void goto_cursor(unsigned char i);
//extern void lcd_write_string(unsigned char  *ptr );
extern void lcd_write_control(unsigned char val);
extern void lcd_write_ascii(unsigned char c);
extern void lcd_write_byte(unsigned char val);
extern void lcd_initialize(void);
extern void enable_lcd(void);
extern char busy_lcd(void);
extern void lcd_printx(unsigned char *str);
extern unsigned char BCD_CONV(unsigned long int CONV_DATA, unsigned char CONV_CNT);
extern void DelayuS(vu32 nCount);
void delay(__IO u32 nCount);

#define Led1_port					GPIOA
#define Led1_pin					GPIO_Pin_12
#define Led2_port					GPIOA
#define Led2_pin					GPIO_Pin_11

#define en_pin						GPIO_Pin_13
#define rw_pin						GPIO_Pin_14
#define rs_pin						GPIO_Pin_15

// LCD Port Init
#define  LCD_DATA_PORT	 	GPIOB
#define en_port						GPIOB
#define rw_port						GPIOB
#define rs_port						GPIOB

#define	lcd_lines							4
//#define lcd_on_off_port			GPIOA
#define lcd_line_1_port			GPIOB
#define lcd_line_2_port			GPIOB
#define lcd_line_3_port			GPIOB
#define lcd_line_4_port			GPIOB
//#define lcd_on_off_pin				GPIO_Pin_1  
#define lcd_line_1_pin		GPIO_Pin_9	//GPIO_Pin_0  
#define lcd_line_2_pin		GPIO_Pin_10	//GPIO_Pin_1
#define lcd_line_3_pin    GPIO_Pin_11	//GPIO_Pin_2
#define lcd_line_4_pin		GPIO_Pin_12	//GPIO_Pin_3

// Control Pins
#define	LCD_EN_LO()			GPIO_ResetBits(en_port,en_pin);
#define LCD_EN_HI()			GPIO_SetBits(en_port,en_pin);
#define	LCD_RS_LO()			GPIO_ResetBits(rs_port,rs_pin);
#define LCD_RS_HI()			GPIO_SetBits(rs_port,rs_pin);
#define	LCD_RW_LO()			GPIO_ResetBits(rw_port,rs_pin);
#define LCD_RW_HI()			GPIO_SetBits(rw_port,rs_pin);

//Data Pins
#define	LCD_D4_LO()			GPIO_ResetBits(lcd_line_1_port,lcd_line_1_pin);
#define LCD_D4_HI()			GPIO_SetBits(lcd_line_1_port,lcd_line_1_pin);
#define	LCD_D5_LO()			GPIO_ResetBits(lcd_line_2_port,lcd_line_2_pin);
#define LCD_D5_HI()			GPIO_SetBits(lcd_line_2_port,lcd_line_2_pin);
#define	LCD_D6_LO()			GPIO_ResetBits(lcd_line_3_port,lcd_line_3_pin);
#define LCD_D6_HI()			GPIO_SetBits(lcd_line_3_port,lcd_line_3_pin);
#define	LCD_D7_LO()			GPIO_ResetBits(lcd_line_4_port,lcd_line_4_pin);
#define LCD_D7_HI()			GPIO_SetBits(lcd_line_4_port,lcd_line_4_pin);

#define  lcd_clear()                 lcd_write_control(0x01)	// Clear Display
#define  lcd_cursor_home()           lcd_write_control(0x02)	// Set Cursor = 0
#define  lcd_display_on()            lcd_write_control(0x0E)	// LCD Display Enable
#define  lcd_display_off()           lcd_write_control(0x08)	// LCD Display Disable
#define  lcd_cursor_blink()          lcd_write_control(0x0F)	// Set Cursor = Blink
#define  lcd_cursor_on()             lcd_write_control(0x0E)	// Enable LCD Cursor
#define  lcd_cursor_off()            lcd_write_control(0x0C)	// Disable LCD Cursor
#define  lcd_cursor_left()           lcd_write_control(0x10)	// Shift Left Cursor
#define  lcd_cursor_right()          lcd_write_control(0x14)	// Shift Right Cursor
#define  lcd_display_sleft()         lcd_write_control(0x18)	// Shift Left Display
#define  lcd_display_sright()        lcd_write_control(0x1C)	// Shift Right Display

#define Relay1_Port				GPIOA  
#define Relay1_pin				GPIO_Pin_11	//GPIO_Pin_0 

#define Relay2_Port				GPIOA  
#define Relay2_pin				GPIO_Pin_12	//GPIO_Pin_0

#define	Relay1_On()			GPIO_ResetBits(Relay1_Port,Relay1_pin);
#define Relay1_Off()			GPIO_SetBits(Relay1_Port,Relay1_pin);

#define	Relay2_On()			GPIO_ResetBits(Relay2_Port,Relay2_pin);
#define Relay2_Off()			GPIO_SetBits(Relay2_Port,Relay2_pin);
