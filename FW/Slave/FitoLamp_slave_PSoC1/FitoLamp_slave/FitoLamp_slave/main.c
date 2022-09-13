//----------------------------------------------------------------------------
// C main line
//----------------------------------------------------------------------------

#include <m8c.h>        // part specific constants and macros
#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
#include "delay.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h> 
#include <stdbool.h>
#include <string.h>

#define DEBUG

#define DECIMAL_COUNT_SYSTEM_BASIS  10
#define DECIMAL_NUMBER_SIZE         4
#define HOUR_MAX   	          	    23
#define DELAYS_IN_SECOND         	100

// NMEA definitions
#define NMEA_MAX_SIZE             82
#define NMEA_START_DELIMITER      '$'
#define NMEA_END_DELIMITER        0x0A
#define NMEA_CHECKSUM_DELIMITER   '*'
#define NMEA_FIELD_DELIMITER      ','
#define NMEA_HEADER_SIZE          3

#define NMEA_GPRMC_UTC              1
#define NMEA_GPRMC_SATELLITES       7
#define NMEA_GPRMC_HDOP             8
#define NMEA_GPRMC_ALTITUDE         9

#define NMEA_GPRMC_VALID            'A'
#define NMEA_GPRMC_INVALID          'V'

// System settings
#define POWER_MAX	14000
#define POWER_STEP	1
#define GMT_OFFSET	3

#define WAIT_PERIOD			2		// Global non critical tasks execution period in seconds
#define POWER_UPDATE_SLOW   2000
#define POWER_UPDATE_FAST   200

unsigned int const schedule[2][2] = {
		                                {5, POWER_MAX},
		                                {20, POWER_MAX}
                              		};

struct datetime {
	unsigned char sec;
	unsigned char min;
	unsigned char hour;
	unsigned char day;
	unsigned char month;
	unsigned char year;
	bool valid;
};

unsigned int power_target = 0;

// NMEA variables
char NMEA_buffer[NMEA_MAX_SIZE] = "NMEA_buffer";
char NMEA_GPRMC[NMEA_MAX_SIZE] = "GPRMC";
unsigned char NMEA_pointer;

struct datetime gps_datetime = {0, 0, 0, 0, 0, 0, false};
struct datetime local_datetime = {0, 0, 0, 0, 0, 0, false};

void set_power(unsigned int pwr);
void update_power(void);
void schedule_processing(unsigned char hour);
void schedule_init(void);
void rtc_update(struct datetime *datetime);

// NMEA functions
void NMEA_handle_packet(char *packet, char *NMEA_data);
void NMEA_GetField(char *packet, unsigned char field, char *result);
void NMEA_GetTimeUTC(char *gprmc, struct datetime *gps_datetime);

unsigned char str_cmp(char *str1, char *str2, unsigned char stop);
unsigned char byte_to_bcd(unsigned char byte);
unsigned char bcd_to_byte(unsigned char reg);
void utc_to_local(struct datetime *gps_datetime, struct datetime *local_datetime);

void nmea_signal(void)
{
	unsigned char i;
	if (NMEA_pointer >= NMEA_MAX_SIZE) NMEA_pointer = 0;
    NMEA_buffer[NMEA_pointer] = RX8_GPS_bReadRxData();	
    NMEA_buffer[NMEA_pointer + 1] = 0;	
    switch(NMEA_buffer[NMEA_pointer])
    {
        case NMEA_START_DELIMITER:
        NMEA_pointer = 0;
        break;
        
        case NMEA_END_DELIMITER:
        NMEA_handle_packet(NMEA_buffer, NMEA_GPRMC);
        break;
        
        default:
        NMEA_pointer++;
        break;
    }
}

void main(void)
{
	unsigned char t;
	
	M8C_EnableGInt; // Uncomment this line to enable Global Interrupts

	RTC_SetHour(0x08);
	RTC_SetMinute(0x00);
	RTC_SetSecond(0x00);
	RTC_Start();
	PWM16_CH0_Start();
	PWM16_CH1_Start();
	Counter16_PwrUpd_Start();
	RX8_GPS_Start(RX8_GPS_PARITY_NONE);
	
#ifdef DEBUG
	LCD_Init();
	LCD_Position(0, 0);
	LCD_PrCString(" ");
#endif // DEBUG
	
	RX8_GPS_EnableInt();
	Counter16_PwrUpd_EnableInt();
		
	Counter16_PwrUpd_WritePeriod(POWER_UPDATE_SLOW);
	set_power(POWER_MAX);
	
	while (1)
	{
		M8C_DisableGInt;
		
		// Get datetime
		local_datetime.valid = false;
		NMEA_GetTimeUTC(NMEA_GPRMC, &gps_datetime);
		if(gps_datetime.valid) 
		{
			utc_to_local(&gps_datetime, &local_datetime);
			rtc_update(&local_datetime);
		}
		
		// Scheduler
		schedule_init();
			
		#ifdef DEBUG
			LCD_Position(0, 0);
			LCD_PrHexByte(RTC_bReadHour());
			LCD_Position(0, 3);
			LCD_PrHexByte(RTC_bReadMinute());
			LCD_Position(0, 6);
			LCD_PrHexByte(RTC_bReadSecond());
			
			LCD_Position(1, 0);
			LCD_PrHexInt(PWM16_CH0_wReadPulseWidth());			
		#endif // DEBUG
			
		M8C_EnableGInt;
		for (t=0; t<=DELAYS_IN_SECOND; t++)	Delay10msTimes(WAIT_PERIOD);
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

void schedule_processing(unsigned char hour)
{
    unsigned char i; 
    for(i = 0; i < sizeof(schedule); i++)
    {
        if(hour == schedule[i][0])
        {
            set_power(schedule[i][1]);
        }
    }
}

void schedule_init(void)
{
    unsigned char hour;
    for(hour = 0; hour <= bcd_to_byte(RTC_bReadHour()); hour++)
    {
        schedule_processing(hour);
    }
}

void rtc_update(struct datetime *datetime)
{
	RTC_Stop();
	RTC_SetHour(byte_to_bcd(datetime->hour));
	RTC_SetMinute(byte_to_bcd(datetime->min));
	RTC_SetSecond(byte_to_bcd(datetime->sec));
	RTC_Start();
}

void NMEA_GetField(char *packet, unsigned char field, char *result)
{
    unsigned char i;
    unsigned char count = 0;
    
    // Search field
    for (i = 0; (i < NMEA_MAX_SIZE) & (count < field); i++)
    {
        if (packet[i] == NMEA_FIELD_DELIMITER) count++;
		if (packet[i] == 0) break;
    }
    
    // Measure field size
    for (count = 0; count < NMEA_MAX_SIZE; count++)
    {
        if (packet[i + count] == NMEA_FIELD_DELIMITER) break;
        if (packet[i + count] == 0u) break;
    }
    strncpy(result, packet + i, count + 1);  // Add 1 to count for null terminator
	result[count] = 0u;	// Add null terminator
}

void NMEA_handle_packet(char *packet, char *NMEA_data)
{
    unsigned char i, n;
    unsigned char error = 0;
    unsigned char checksum = 0;
    char *packet_checksum;
    char calculated_checksum[3];
        
    // Check if appropriate packet is handled
	if (str_cmp(packet, NMEA_data, NMEA_HEADER_SIZE) == 0u)
    {
		// Check for receive errors
        for(i = 0; i < NMEA_MAX_SIZE; i++)
        {
            if ((packet[i] < 32) & (packet[i] != 0x0D) & (packet[i] != NMEA_END_DELIMITER)) 
            {
                error++;
                break;
            }
            if (packet[i] != NMEA_END_DELIMITER) break;
        }
		
        // Copy buffer to NMEA packet if no errors found
        if (!error) strncpy(NMEA_data, packet, NMEA_MAX_SIZE);
    }
}

void NMEA_GetTimeUTC(char *gprmc, struct datetime *gps_datetime)
{
	#define POS_SIZE	2
	#define POS_HOUR	0
	#define POS_MIN		2
	#define POS_SEC		4
	
	char field[NMEA_MAX_SIZE];
	char buf[NMEA_MAX_SIZE];
	
	NMEA_GetField(gprmc, NMEA_GPRMC_UTC, field);
	if (field[0] != 0)
	{
		// Hour
		strncpy(buf, field + POS_HOUR, POS_SIZE);
		buf[POS_SIZE] = 0;	// Add null terminator
		gps_datetime->hour = atoi(buf);
		
		// Minutes
		strncpy(buf, field + POS_MIN, POS_SIZE);
		buf[POS_SIZE] = 0;	// Add null terminator
		gps_datetime->min = atoi(buf);
		
		// Seconds
		strncpy(buf, field + POS_SEC, POS_SIZE);
		buf[POS_SIZE] = 0;	// Add null terminator
		gps_datetime->sec = atoi(buf);
		
		gps_datetime->valid = true;
	}
	else gps_datetime->valid = false;
}

unsigned char str_cmp(char *str1, char *str2, unsigned char stop)
{
    unsigned char i;
    for(i = 0u; i <= stop; i++)
    {
        if (str1[i] != str2[i]) return 1u;
    }
    return 0u;
}

unsigned char byte_to_bcd(unsigned char byte)
{
    unsigned char high, low, reg;
    
    high = byte / DECIMAL_COUNT_SYSTEM_BASIS;
    low = byte - high * DECIMAL_COUNT_SYSTEM_BASIS;
    reg = (high << DECIMAL_NUMBER_SIZE) + low;
    
    return reg;
}

unsigned char bcd_to_byte(unsigned char reg)
{
    unsigned char high, low, byte;
    
    high = (reg >> DECIMAL_NUMBER_SIZE);
    low = reg - (high << DECIMAL_NUMBER_SIZE);
    byte = low + high * DECIMAL_COUNT_SYSTEM_BASIS;

    return byte;
}

void utc_to_local(struct datetime *utc_datetime, struct datetime *local_datetime)
{
	local_datetime->valid = utc_datetime->valid;
	local_datetime->sec = utc_datetime->sec;
	local_datetime->min = utc_datetime->min;
	local_datetime->hour = utc_datetime->hour + GMT_OFFSET;
	if (local_datetime->hour > HOUR_MAX) local_datetime->hour -= (HOUR_MAX + 1);
}