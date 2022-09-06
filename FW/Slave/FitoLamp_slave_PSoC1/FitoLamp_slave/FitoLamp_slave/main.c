//----------------------------------------------------------------------------
// C main line
//----------------------------------------------------------------------------

#include <m8c.h>        // part specific constants and macros
#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
#include "delay.h"

#define DEBUG

#define POWER_MAX	14000
#define POWER_STEP	1

#define UPDATE_PERIOD		2

unsigned int power_target = 0;

void set_power(unsigned int pwr);
void update_power(void);

void main(void)
{
	// M8C_EnableGInt ; // Uncomment this line to enable Global Interrupts
	// Insert your main routine code here.
	RTC_SetHour(0x08);
	RTC_SetMinute(0x00);
	RTC_SetSecond(0x00);
	RTC_Start();
	PWM16_CH0_Start();
	PWM16_CH1_Start();
#ifdef DEBUG
	LCD_Init();
#endif // DEBUG
		
	set_power(POWER_MAX);
	
	while (1)
	{
		// Scheduler
		switch (RTC_bReadHour())
		{
			case 0x05:
				set_power(POWER_MAX);
				break;
			case 0x19:
				set_power(0);
				break;
		}
		update_power();
		Delay10msTimes(UPDATE_PERIOD);
#ifdef DEBUG
		LCD_Position(0,0);
		LCD_PrHexByte(RTC_bReadHour());
		LCD_PrCString(":");
		LCD_PrHexByte(RTC_bReadMinute());
		LCD_PrCString(":");
		LCD_PrHexByte(RTC_bReadSecond());
		LCD_Position(1, 0);
		LCD_PrCString("PT:");
		LCD_PrHexInt(power_target);
		LCD_PrCString("      ");		
#endif // DEBUG
	}
}

void set_power(unsigned int pwr)
{
	if(pwr > POWER_MAX) pwr = POWER_MAX;
	power_target = pwr;
}

void update_power(void)
{
	unsigned int pwr;
	
	// CH0
	pwr = PWM16_CH0_wReadPulseWidth();
	if(pwr < power_target) pwr += POWER_STEP;
	if(pwr > power_target) pwr -= POWER_STEP;
	PWM16_CH0_WritePulseWidth(pwr);
	
	// CH1
	pwr = PWM16_CH1_wReadPulseWidth();
	if(pwr < power_target) pwr += POWER_STEP;
	if(pwr > power_target) pwr -= POWER_STEP;
	PWM16_CH1_WritePulseWidth(pwr);
}