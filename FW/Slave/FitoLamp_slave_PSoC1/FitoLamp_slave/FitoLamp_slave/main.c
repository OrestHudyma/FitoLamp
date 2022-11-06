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

//#define DEBUG

#define DECIMAL_COUNT_SYSTEM_BASIS  10
#define DECIMAL_NUMBER_SIZE         4
#define HOUR_MAX   	          	    23

// NMEA definitions
#define NMEA_MAX_SIZE             82
#define NMEA_START_DELIMITER      '$'
#define NMEA_END_DELIMITER        0x0A
#define NMEA_CHECKSUM_DELIMITER   '*'
#define NMEA_FIELD_DELIMITER      ','
#define NMEA_HEADER_SIZE          3

#define NMEA_GPRMC_UTC              1
#define NMEA_GPRMC_DATE      		7
#define NMEA_GPRMC_HDOP             8
#define NMEA_GPRMC_ALTITUDE         9

#define NMEA_GPRMC_VALID            'A'
#define NMEA_GPRMC_INVALID          'V'

// System settings
#define POWER_MAX	14000
#define POWER_STEP	1
#define GMT_OFFSET	3
#define WAIT_PERIOD			2			// Global non critical tasks execution period in x10 miliseconds
#define OVERRIDE_TIMEOUT	540000		// x20 miliseconds (540000 = 3 hours)
#define POWER_UPDATE_SLOW   2000
#define POWER_UPDATE_FAST   100
#define HW_ID				"1"

#define NMEA_GPRMC_EMPTY            "GPRMC"
#define NMEA_SHFTL_EMPTY            "SHFTL"
#define NMEA_FIELD_CMD              1
#define NMEA_FIELD_ID               2

const char hw_id[] = HW_ID;

const char cmd_on[] = "ON";
const char cmd_off[] = "OFF";
const char cmd_fon[] = "FON";
const char cmd_foff[] = "FOFF";
char nmea_gprmc_empty[] = NMEA_GPRMC_EMPTY;
char nmea_shftl_empty[] = NMEA_SHFTL_EMPTY;
char fld_buf[NMEA_MAX_SIZE];
bool override = false;
unsigned long override_counter;

unsigned int const schedule[2][2] = {
		                                {5, POWER_MAX},
		                                {20, 0}
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
char NMEA_buffer_gps[NMEA_MAX_SIZE] = "NMEA_buffer_gps";
char NMEA_buffer_rf[NMEA_MAX_SIZE] = "NMEA_buffer_rf";
char NMEA_GPRMC[NMEA_MAX_SIZE] = NMEA_GPRMC_EMPTY;
char NMEA_SHFTL[NMEA_MAX_SIZE] = NMEA_SHFTL_EMPTY;
bool NMEA_cmd_received = false;
unsigned char NMEA_pointer_gps;
unsigned char NMEA_pointer_rf;

struct datetime gps_datetime = {0, 0, 0, 0, 0, 0, false};
struct datetime local_datetime = {0, 0, 0, 0, 0, 0, false};

void set_power(unsigned int pwr);
void override_enable(void);
void update_power(void);
void schedule_processing(unsigned char hour);
void schedule_init(void);
void rtc_update(struct datetime *datetime);
bool check_fld(const char *cmd);

// NMEA functions
bool NMEA_handle_packet(char *packet, char *NMEA_data);
void NMEA_GetField(char *packet, unsigned char field, char *result);
void NMEA_GetTimeUTC(char *gprmc, struct datetime *gps_datetime);

unsigned char str_cmp(char *str1, char *str2, unsigned char stop);
unsigned char str_cmp_const(char *str1, const char *str2, unsigned char stop);
unsigned char byte_to_bcd(unsigned char byte);
unsigned char bcd_to_byte(unsigned char reg);
void utc_to_local(struct datetime *gps_datetime, struct datetime *local_datetime);

void gps_signal(void)
{
	M8C_DisableGInt;
	if (NMEA_pointer_gps >= NMEA_MAX_SIZE) NMEA_pointer_gps = 0;
    NMEA_buffer_gps[NMEA_pointer_gps] = RX8_GPS_bReadRxData();	
    NMEA_buffer_gps[NMEA_pointer_gps + 1] = 0;	
    switch(NMEA_buffer_gps[NMEA_pointer_gps])
    {
        case NMEA_START_DELIMITER:
        NMEA_pointer_gps = 0;
        break;
        
        case NMEA_END_DELIMITER:
        NMEA_handle_packet(NMEA_buffer_gps, NMEA_GPRMC);
        break;
        
        default:
        NMEA_pointer_gps++;
        break;
    }
	M8C_EnableGInt;
}

void rf_signal(void)
{	
	M8C_DisableGInt;
	if (NMEA_pointer_rf >= NMEA_MAX_SIZE) NMEA_pointer_rf = 0;
    NMEA_buffer_rf[NMEA_pointer_rf] = RX8_RF_bReadRxData();	
    NMEA_buffer_rf[NMEA_pointer_rf + 1] = 0;	
    switch(NMEA_buffer_rf[NMEA_pointer_rf])
    {
        case NMEA_START_DELIMITER:
        NMEA_pointer_rf = 0;
        break;
        
        case NMEA_END_DELIMITER:
        NMEA_cmd_received = NMEA_handle_packet(NMEA_buffer_rf, NMEA_SHFTL);
        NMEA_buffer_rf[0] = 0;
        break;
        
        default:
        NMEA_pointer_rf++;
        break;
    }
	M8C_EnableGInt;
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
	Counter8_RF_clk_Start();
	RX8_GPS_Start(RX8_GPS_PARITY_NONE);
	RX8_RF_Start(RX8_GPS_PARITY_ODD);
	
	RX8_GPS_EnableInt();
	RX8_RF_EnableInt();
	Counter16_PwrUpd_EnableInt();
		
	Counter16_PwrUpd_WritePeriod(POWER_UPDATE_SLOW);
	set_power(POWER_MAX);
	LED_Blue_Off();
	
	while (1)
	{		
		// Handle commands
		if (NMEA_cmd_received)
        {
			LED_Blue_On();
			NMEA_cmd_received = false;		
            
            // NMEA_SHFTL handle
			NMEA_GetField(NMEA_SHFTL, NMEA_FIELD_ID, fld_buf);
			if(check_fld(hw_id) || check_fld("0"))	// Check ID
			{			
	            NMEA_GetField(NMEA_SHFTL, NMEA_FIELD_CMD, fld_buf);
	            if(check_fld(cmd_on))
	            {
	                Counter16_PwrUpd_WritePeriod(POWER_UPDATE_SLOW);
					set_power(POWER_MAX);
					override_enable();	
	            }
	            else if(check_fld(cmd_off))
	            {
	                Counter16_PwrUpd_WritePeriod(POWER_UPDATE_SLOW);
					set_power(0);
					override_enable();	
	            }
				else if(check_fld(cmd_fon))
	            {
	                Counter16_PwrUpd_WritePeriod(POWER_UPDATE_FAST);
					set_power(POWER_MAX);
					override_enable();	
	            }
	            else if(check_fld(cmd_foff))
	            {
	                Counter16_PwrUpd_WritePeriod(POWER_UPDATE_FAST);
					set_power(0);
					override_enable();	
	            }
				else LED_Blue_Off();
				
	            NMEA_SHFTL[0] = 0;
	            strncat(NMEA_SHFTL, nmea_shftl_empty, NMEA_MAX_SIZE);
			}
		}		
				
		if(!override)
		{
			// Get datetime
			local_datetime.valid = false;
			NMEA_GetTimeUTC(NMEA_GPRMC, &gps_datetime);
			if(gps_datetime.valid) 
			{
				utc_to_local(&gps_datetime, &local_datetime);
				rtc_update(&local_datetime);
			}
			
			// Scheduler
			Counter16_PwrUpd_WritePeriod(POWER_UPDATE_SLOW);
			schedule_init();
		}
		
		Delay10msTimes(WAIT_PERIOD);
		if (override_counter > 0) override_counter--;
		else override = false;
		LED_Blue_Off();
	}
}

void set_power(unsigned int pwr)
{
	if(pwr > POWER_MAX) pwr = POWER_MAX;
	power_target = pwr;
}

void override_enable(void)
{
	override = true;
	override_counter = OVERRIDE_TIMEOUT;
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
	
	for(hour = 0; hour <= HOUR_MAX; hour++)
    {
        schedule_processing(hour);
    }
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

bool check_fld(const char *cmd)
{
    return !str_cmp_const(fld_buf, cmd, strlen(fld_buf) - 1);
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

bool NMEA_handle_packet(char *packet, char *NMEA_data)
{
    unsigned char i, n;
    unsigned char error = 0;
	        
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
	else error++;
	
	if(error) return false;
	else return true;	
}

void NMEA_GetTimeUTC(char *gprmc, struct datetime *gps_datetime)
{
	#define POS_SIZE	2
	#define POS_HOUR	0
	#define POS_MIN		2
	#define POS_SEC		4	
	#define POS_DAY		0
	#define POS_MON		2
	#define POS_YEAR	4
	
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
	
	NMEA_GetField(gprmc, NMEA_GPRMC_DATE, field);
	if (field[0] != 0)
	{
		// Day
		strncpy(buf, field + POS_DAY, POS_SIZE);
		buf[POS_SIZE] = 0;	// Add null terminator
		gps_datetime->day = atoi(buf);
		
		// Month
		strncpy(buf, field + POS_MON, POS_SIZE);
		buf[POS_SIZE] = 0;	// Add null terminator
		gps_datetime->month = atoi(buf);
		
		// Year
		strncpy(buf, field + POS_YEAR, POS_SIZE);
		buf[POS_SIZE] = 0;	// Add null terminator
		gps_datetime->year = atoi(buf);
		
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

unsigned char str_cmp_const(char *str1, const char *str2, unsigned char stop)
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