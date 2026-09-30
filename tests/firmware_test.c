#include <assert.h>
#include <string.h>
#define main firmware_main
#include "../FW/Slave/FitoLamp_slave_PSoC1/FitoLamp_slave/FitoLamp_slave/main.c"
#undef main

static unsigned char rx_byte;
static unsigned int pwm, timer_period, alarm_transitions;
static int rf_masked, pwm_masked;
static const char *inject_on_enable;

static void feed(const char *wire)
{
    assert(!rf_masked);
    while (*wire) { rx_byte = (unsigned char)*wire++; rf_signal(); }
}

void RTC_SetHour(unsigned char value) { (void)value; }
void RTC_SetMinute(unsigned char value) { (void)value; }
void RTC_SetSecond(unsigned char value) { (void)value; }
unsigned char RTC_bReadHour(void) { return 0; }
void RTC_Start(void) {}
void RTC_Stop(void) {}
void PWM16_CH0_Start(void) {}
void PWM16_CH1_Start(void) {}
/* Simulate completion of a PWM ramp, not physical timing or ISR execution. */
unsigned int PWM16_CH0_wReadPulseWidth(void)
{
    assert(!pwm_masked);
    pwm = power_target;
    return pwm;
}
void PWM16_CH0_WritePulseWidth(unsigned int value) { pwm = value; }
void PWM16_CH1_WritePulseWidth(unsigned int value) { (void)value; }
void Counter16_PwrUpd_Start(void) {}
void Counter16_PwrUpd_WritePeriod(unsigned int value)
{
    timer_period = value;
    if (value == POWER_UPDATE_ALARM) alarm_transitions++;
}
void Counter16_PwrUpd_EnableInt(void) { pwm_masked = 0; }
void Counter16_PwrUpd_DisableInt(void) { pwm_masked = 1; }
void Counter8_RF_clk_Start(void) {}
void RX8_GPS_Start(unsigned char value) { (void)value; }
void RX8_RF_Start(unsigned char value) { (void)value; }
void RX8_GPS_EnableInt(void) {}
void RX8_RF_EnableInt(void)
{
    const char *wire = inject_on_enable;
    rf_masked = 0;
    inject_on_enable = NULL;
    if (wire != NULL) feed(wire);
}
void RX8_RF_DisableInt(void) { rf_masked = 1; }
unsigned char RX8_GPS_bReadRxData(void) { return rx_byte; }
unsigned char RX8_RF_bReadRxData(void) { return rx_byte; }
void LED_Blue_On(void) {}
void LED_Blue_Off(void) {}
void Delay10msTimes(unsigned char value) { (void)value; }

static void execute(const char *wire)
{
    feed(wire);
    process_pending_rf_command();
}

static void assert_alarm(unsigned int previous)
{
    set_power(previous);
    execute("$SHGLB,ALARM,*01\n");
    assert(alarm_transitions == ALARM_CYCLES * 2u + 1u);
    assert(power_target == previous);
    assert(timer_period == POWER_UPDATE_ALARM);
    assert(override && override_counter == OVERRIDE_TIMEOUT);
    assert(!NMEA_cmd_received);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    if (strcmp(argv[1], "global_alarm") == 0) assert_alarm(POWER_MAX);
    else if (strcmp(argv[1], "restore_off") == 0) assert_alarm(0);
    else if (strcmp(argv[1], "restore_partial") == 0) assert_alarm(POWER_MAX / 2u);
    else if (strcmp(argv[1], "addressed_power") == 0)
    {
        execute("$SHFTL,ON,1,*59\n");
        assert(power_target == POWER_MAX && timer_period == POWER_UPDATE_SLOW);
        execute("$SHFTL,OFF,1,*17\n");
        assert(power_target == 0 && timer_period == POWER_UPDATE_SLOW);
        execute("$SHFTL,FON,0,*1E\n");
        assert(power_target == POWER_MAX && timer_period == POWER_UPDATE_FAST);
        execute("$SHFTL,FOFF,1,*51\n");
        assert(power_target == 0 && timer_period == POWER_UPDATE_FAST);
    }
    else if (strcmp(argv[1], "wrong_address") == 0)
    {
        execute("$SHFTL,ON,2,*5A\n");
        assert(!override && power_target == 0);
    }
    else if (strcmp(argv[1], "legacy_alarm_ignored") == 0)
    {
        execute("$SHFTL,ALARM,1,*0B\n");
        execute("$SHFTL,ALARM,0,*0A\n");
        assert(!override && alarm_transitions == 0);
    }
    else if (strcmp(argv[1], "other_globals_ignored") == 0)
    {
        execute("$SHGLB,DAY,*0E\n");
        execute("$SHGLB,NIGHT,*56\n");
        execute("$SHGLB,ON,*47\n");
        assert(!override && alarm_transitions == 0);
    }
    else if (strcmp(argv[1], "exact_header") == 0)
    {
        execute("$SHGLBX,ALARM,*59\n");
        execute("$SHGL,ALARM,*43\n");
        assert(!override && alarm_transitions == 0);
    }
    else if (strcmp(argv[1], "exact_command") == 0)
    {
        execute("$SHGLB,ALARMED,*00\n");
        execute("$SHGLB,AL,*00\n");
        assert(!override && alarm_transitions == 0);
    }
    else if (strcmp(argv[1], "pending_command") == 0)
    {
        feed("$SHGLB,ALARM,*01\n");
        feed("$SHFTL,OFF,1,*17\n");
        feed("$OTHER,DATA,*00\n");
        assert(NMEA_cmd_received);
        process_pending_rf_command();
        assert(alarm_transitions == ALARM_CYCLES * 2u + 1u);
    }
    else if (strcmp(argv[1], "mixed_headers") == 0)
    {
        execute("$SHFTL,ON,1,*59\n");
        execute("$SHGLB,ALARM,*01\n");
        execute("$SHFTL,OFF,1,*17\n");
        assert(alarm_transitions == ALARM_CYCLES * 2u + 1u);
        assert(power_target == 0 && timer_period == POWER_UPDATE_SLOW);
    }
    else if (strcmp(argv[1], "gps_header") == 0)
    {
        const char *wire = "$GPRMC,120000,A,*00\n";
        while (*wire) { rx_byte = (unsigned char)*wire++; gps_signal(); }
        assert(strncmp(NMEA_GPRMC, "GPRMC,120000,A,", 15) == 0);
        assert(!NMEA_cmd_received);
    }
    else if (strcmp(argv[1], "snapshot_interleaving") == 0)
    {
        feed("$SHGLB,ALARM,*01\n");
        inject_on_enable = "$SHFTL,ON,1,*59\n";
        process_pending_rf_command();
        assert(alarm_transitions == ALARM_CYCLES * 2u + 1u);
        assert(power_target == 0 && NMEA_cmd_received);
        process_pending_rf_command();
        assert(power_target == POWER_MAX && !NMEA_cmd_received);
    }
    else return 2;
    return 0;
}
