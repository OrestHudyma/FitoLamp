#ifndef TEST_PSOC_API_H
#define TEST_PSOC_API_H
#define RX8_GPS_PARITY_NONE 0
#define RX8_GPS_PARITY_ODD 6
void RTC_SetHour(unsigned char value);
void RTC_SetMinute(unsigned char value);
void RTC_SetSecond(unsigned char value);
unsigned char RTC_bReadHour(void);
void RTC_Start(void);
void RTC_Stop(void);
void PWM16_CH0_Start(void);
void PWM16_CH1_Start(void);
unsigned int PWM16_CH0_wReadPulseWidth(void);
void PWM16_CH0_WritePulseWidth(unsigned int value);
void PWM16_CH1_WritePulseWidth(unsigned int value);
void Counter16_PwrUpd_Start(void);
void Counter16_PwrUpd_WritePeriod(unsigned int value);
void Counter16_PwrUpd_EnableInt(void);
void Counter16_PwrUpd_DisableInt(void);
void Counter8_RF_clk_Start(void);
void RX8_GPS_Start(unsigned char value);
void RX8_RF_Start(unsigned char value);
void RX8_GPS_EnableInt(void);
void RX8_RF_EnableInt(void);
void RX8_RF_DisableInt(void);
unsigned char RX8_GPS_bReadRxData(void);
unsigned char RX8_RF_bReadRxData(void);
void LED_Blue_On(void);
void LED_Blue_Off(void);
#endif
