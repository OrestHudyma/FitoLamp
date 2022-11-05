Project:      output/FitoLamp_slave
Build Number: 0
Date:         Sat Nov 05 20:10:07 2022
Compiler:     ICCM8C
Version:      7.05.00


Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   0190   028D =    653. bytes (rel,con,rom,lit)

       Addr  Global Symbol
      -----  --------------------------------
       0190  _hw_id
       0192  _cmd_on
       0195  _cmd_off
       0199  _cmd_fon
       019D  _cmd_foff
       01A2  _schedule
       01AC  LoadConfigTBL_fitolamp_slave_Bank0
       0245  LoadConfigTBL_fitolamp_slave_Bank1
       031C  __ctype
       041D  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                        func_lit   041D   0002 =      2. bytes (rel,con,rom,proclab)

       Addr  Global Symbol
      -----  --------------------------------
       041D  __func_lit_start
       041F  __func_lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     psoc_config   041F   0133 =    307. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       041F  LoadConfigTBL_fitolamp_slave_Ordered
       04E4  _LoadConfigInit
       04E4  LoadConfigInit
       04F7  _LoadConfig_fitolamp_slave
       04F7  LoadConfig_fitolamp_slave
       0552  __psoc_config_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     UserModules   0552   0698 =   1688. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       0552  _RX8_RF_ISR
       0654  _RX8_RF_EnableInt
       0654  RX8_RF_EnableInt
       0658  RX8_RF_DisableInt
       0658  _RX8_RF_DisableInt
       065C  RX8_RF_Start
       065C  _RX8_RF_Start
       0661  RX8_RF_Stop
       0661  _RX8_RF_Stop
       0665  _RX8_RF_bReadRxData
       0665  bRX8_RF_ReadRxData
       0665  RX8_RF_bReadRxData
       0665  _bRX8_RF_ReadRxData
       0668  _bRX8_RF_ReadRxStatus
       0668  _RX8_RF_bReadRxStatus
       0668  bRX8_RF_ReadRxStatus
       0668  RX8_RF_bReadRxStatus
       066B  RX8_RF_cGetChar
       066B  _RX8_RF_cGetChar
       0673  _RX8_RF_cReadChar
       0673  RX8_RF_cReadChar
       068A  _RX8_RF_iReadChar
       068A  RX8_RF_iReadChar
       06A0  _RX8_RF_CmdReset
       06A0  RX8_RF_CmdReset
       06B6  RX8_RF_bCmdCheck
       06B6  _RX8_RF_bCmdCheck
       06BE  RX8_RF_bErrCheck
       06BE  _RX8_RF_bErrCheck
       06C9  RX8_RF_bCmdLength
       06C9  _RX8_RF_bCmdLength
       06CF  _RX8_RF_szGetParam
       06CF  RX8_RF_szGetParam
       072D  _RX8_RF_szGetRestOfParams
       072D  RX8_RF_szGetRestOfParams
       0738  _RX8_GPS_ISR
       083A  RX8_GPS_EnableInt
       083A  _RX8_GPS_EnableInt
       083E  _RX8_GPS_DisableInt
       083E  RX8_GPS_DisableInt
       0842  RX8_GPS_Start
       0842  _RX8_GPS_Start
       0847  RX8_GPS_Stop
       0847  _RX8_GPS_Stop
       084B  bRX8_GPS_ReadRxData
       084B  _RX8_GPS_bReadRxData
       084B  RX8_GPS_bReadRxData
       084B  _bRX8_GPS_ReadRxData
       084E  _bRX8_GPS_ReadRxStatus
       084E  _RX8_GPS_bReadRxStatus
       084E  RX8_GPS_bReadRxStatus
       084E  bRX8_GPS_ReadRxStatus
       0851  _RX8_GPS_cGetChar
       0851  RX8_GPS_cGetChar
       0859  RX8_GPS_cReadChar
       0859  _RX8_GPS_cReadChar
       0870  _RX8_GPS_iReadChar
       0870  RX8_GPS_iReadChar
       0886  RX8_GPS_CmdReset
       0886  _RX8_GPS_CmdReset
       089C  _RX8_GPS_bCmdCheck
       089C  RX8_GPS_bCmdCheck
       08A4  _RX8_GPS_bErrCheck
       08A4  RX8_GPS_bErrCheck
       08AF  _RX8_GPS_bCmdLength
       08AF  RX8_GPS_bCmdLength
       08B5  _RX8_GPS_szGetParam
       08B5  RX8_GPS_szGetParam
       0913  _RX8_GPS_szGetRestOfParams
       0913  RX8_GPS_szGetRestOfParams
       091E  _RTC_ISR
       091F  _RTC_EnableInt
       091F  RTC_EnableInt
       092A  RTC_DisableInt
       092A  _RTC_DisableInt
       0935  _RTC_ClearInt
       0935  RTC_ClearInt
       0939  _RTC_Start
       0939  RTC_Start
       0941  _RTC_Stop
       0941  RTC_Stop
       0949  RTC_SetIntPeriod
       0949  _RTC_SetIntPeriod
       095F  RTC_bReadSecond
       095F  _RTC_bReadSecond
       0966  RTC_bReadMinute
       0966  _RTC_bReadMinute
       096D  _RTC_bReadHour
       096D  RTC_bReadHour
       0974  _RTC_SetSecond
       0974  RTC_SetSecond
       098C  _RTC_SetMinute
       098C  RTC_SetMinute
       09A4  _RTC_SetHour
       09A4  RTC_SetHour
       09BC  _PWM16_CH1_ISR
       09BD  _PWM16_CH1_EnableInt
       09BD  PWM16_CH1_EnableInt
       09C1  PWM16_CH1_DisableInt
       09C1  _PWM16_CH1_DisableInt
       09C5  _PWM16_CH1_Start
       09C5  PWM16_CH1_Start
       09C9  PWM16_CH1_Stop
       09C9  _PWM16_CH1_Stop
       09CD  _PWM16_CH1_WritePeriod
       09CD  PWM16_CH1_WritePeriod
       09D3  _PWM16_CH1_WritePulseWidth
       09D3  PWM16_CH1_WritePulseWidth
       09D9  _wPWM16_CH1_ReadPulseWidth
       09D9  _PWM16_CH1_wReadPulseWidth
       09D9  wPWM16_CH1_ReadPulseWidth
       09D9  PWM16_CH1_wReadPulseWidth
       09DF  _PWM16_CH1_wReadCounter
       09DF  _wPWM16_CH1_ReadCounter
       09DF  wPWM16_CH1_ReadCounter
       09DF  PWM16_CH1_wReadCounter
       0A19  _PWM16_CH0_ISR
       0A1A  PWM16_CH0_EnableInt
       0A1A  _PWM16_CH0_EnableInt
       0A1E  _PWM16_CH0_DisableInt
       0A1E  PWM16_CH0_DisableInt
       0A22  _PWM16_CH0_Start
       0A22  PWM16_CH0_Start
       0A26  _PWM16_CH0_Stop
       0A26  PWM16_CH0_Stop
       0A2A  _PWM16_CH0_WritePeriod
       0A2A  PWM16_CH0_WritePeriod
       0A30  _PWM16_CH0_WritePulseWidth
       0A30  PWM16_CH0_WritePulseWidth
       0A36  wPWM16_CH0_ReadPulseWidth
       0A36  PWM16_CH0_wReadPulseWidth
       0A36  _PWM16_CH0_wReadPulseWidth
       0A36  _wPWM16_CH0_ReadPulseWidth
       0A3C  _wPWM16_CH0_ReadCounter
       0A3C  wPWM16_CH0_ReadCounter
       0A3C  PWM16_CH0_wReadCounter
       0A3C  _PWM16_CH0_wReadCounter
       0A76  _LED_Blue_On
       0A76  LED_Blue_On
       0A7A  LED_Blue_Start
       0A7A  LED_Blue_Off
       0A7A  _LED_Blue_Start
       0A7A  _LED_Blue_Stop
       0A7A  _LED_Blue_Off
       0A7A  LED_Blue_Stop
       0A7C  LED_Blue_Switch
       0A7C  _LED_Blue_Switch
       0A94  _LED_Blue_Invert
       0A94  LED_Blue_Invert
       0AA3  _LED_Blue_GetState
       0AA3  LED_Blue_GetState
       0AB4  _Counter8_RF_clk_ISR
       0AB5  Counter8_RF_clk_EnableInt
       0AB5  _Counter8_RF_clk_EnableInt
       0AB9  _Counter8_RF_clk_DisableInt
       0AB9  Counter8_RF_clk_DisableInt
       0ABD  _Counter8_RF_clk_Start
       0ABD  Counter8_RF_clk_Start
       0AC1  _Counter8_RF_clk_Stop
       0AC1  Counter8_RF_clk_Stop
       0AC5  Counter8_RF_clk_WritePeriod
       0AC5  _Counter8_RF_clk_WritePeriod
       0AC8  Counter8_RF_clk_WriteCompareValue
       0AC8  _Counter8_RF_clk_WriteCompareValue
       0ACB  _Counter8_RF_clk_bReadCompareValue
       0ACB  Counter8_RF_clk_bReadCompareValue
       0ACB  bCounter8_RF_clk_ReadCompareValue
       0ACB  _bCounter8_RF_clk_ReadCompareValue
       0ACE  _bCounter8_RF_clk_ReadCounter
       0ACE  _Counter8_RF_clk_bReadCounter
       0ACE  bCounter8_RF_clk_ReadCounter
       0ACE  Counter8_RF_clk_bReadCounter
       0AFF  _Counter16_PwrUpd_ISR
       0B8A  Counter16_PwrUpd_EnableInt
       0B8A  _Counter16_PwrUpd_EnableInt
       0B8E  _Counter16_PwrUpd_DisableInt
       0B8E  Counter16_PwrUpd_DisableInt
       0B92  Counter16_PwrUpd_Start
       0B92  _Counter16_PwrUpd_Start
       0B96  Counter16_PwrUpd_Stop
       0B96  _Counter16_PwrUpd_Stop
       0B9A  _Counter16_PwrUpd_WritePeriod
       0B9A  Counter16_PwrUpd_WritePeriod
       0BA0  Counter16_PwrUpd_WriteCompareValue
       0BA0  _Counter16_PwrUpd_WriteCompareValue
       0BA6  _Counter16_PwrUpd_wReadCompareValue
       0BA6  _wCounter16_PwrUpd_ReadCompareValue
       0BA6  Counter16_PwrUpd_wReadCompareValue
       0BA6  wCounter16_PwrUpd_ReadCompareValue
       0BAC  _Counter16_PwrUpd_wReadCounter
       0BAC  Counter16_PwrUpd_wReadCounter
       0BAC  wCounter16_PwrUpd_ReadCounter
       0BAC  _wCounter16_PwrUpd_ReadCounter
       0BEA  __UserModules_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   0BEA   0F4F =   3919. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       0BEA  Delay10msTimes
       0BEA  _Delay10msTimes
       0BEA  __text_start
       0BF7  _Delay50uTimes
       0BF7  Delay50uTimes
       0BFD  Delay50u_Table
       0C05  Delay50u
       0C05  _Delay50u
       0C19  _gps_signal
       0CCA  _rf_signal
       0D91  _main
       0FB8  _set_power
       0FD7  _override_enable
       0FF0  _update_power
       1074  _schedule_processing
       10F4  _schedule_init
       1132  _rtc_update
       1194  _check_fld
       11CF  _NMEA_GetField
       130A  _NMEA_handle_packet
       1435  _NMEA_GetTimeUTC
       1649  _str_cmp
       169A  _str_cmp_const
       16EC  _byte_to_bcd
       1723  _bcd_to_byte
       175A  _utc_to_local
       184D  _atoi
       198E  _isdigit
       19AF  _isspace
       19D0  _strncat
       1A7D  _strncpy
       1B39  __text_end
       1B39  __xidata_start
       1B91  __xidata_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             TOP   0000   0151 =    337. bytes (abs,ovr,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       0080  __Start
       014E  _bGetPowerSetting
       014E  bGetPowerSetting

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                    InterruptRAM   0000   0008 =      8. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0000  _RX8_RF_fStatus
       0000  RX8_RF_fStatus
       0001  RX8_RF_bRxCnt
       0001  _RX8_RF_bRxCnt
       0002  RX8_GPS_fStatus
       0002  _RX8_GPS_fStatus
       0003  RX8_GPS_bRxCnt
       0003  _RX8_GPS_bRxCnt
       0004  Port_1_Data_SHADE
       0004  NO_SHADOW
       0004  _NO_SHADOW
       0004  _Port_1_Data_SHADE
       0005  _Port_2_Data_SHADE
       0005  Port_2_Data_SHADE
       0006  Port_2_DriveMode_0_SHADE
       0006  _Port_2_DriveMode_0_SHADE
       0007  _Port_2_DriveMode_1_SHADE
       0007  Port_2_DriveMode_1_SHADE

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     RX8_GPS_RAM   0000   0011 =     17. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       00BC  _RX8_GPS_aRxBuffer
       00BC  RX8_GPS_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                      RX8_RF_RAM   0000   0011 =     17. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       00AC  RX8_RF_aRxBuffer
       00AC  _RX8_RF_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
               virtual_registers   00CC   000F =     15. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       00CC  __rZ
       00CD  __rY
       00CE  __rX
       00CF  __r11
       00D0  __r10
       00D1  __r9
       00D2  __r8
       00D3  __r7
       00D4  __r6
       00D5  __r5
       00D6  __r4
       00D7  __r3
       00D8  __r2
       00D9  __r1
       00DA  __r0

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            data   0000   01C2 =    450. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0000  __data_start
       0008  _NMEA_buffer_gps
       005A  _NMEA_SHFTL
       00DB  _local_datetime
       00E3  _gps_datetime
       00EB  _nmea_shftl_empty
       00F1  _nmea_gprmc_empty
       0100  _NMEA_GPRMC
       0152  _NMEA_buffer_rf
       01A4  _fld_buf
       01F6  _external_override_counter
       01FA  _NMEA_cmd_received
       01FC  _power_target
       01FE  _external_override
       0200  _NMEA_pointer_rf
       0201  _NMEA_pointer_gps

Files Linked      [ module(s) ]

./obj/boot.o	[ boot.asm ]
obj/delay.o	[ delay.asm ]
obj/main.o	[ main.c ]
<library>	[ rx8_rfint.asm, rx8_rf.asm, rx8_gpsint.asm, rx8_gps.asm, rtcint.asm, rtc.asm, pwm16_ch1int.asm ]
[ pwm16_ch1.asm, pwm16_ch0int.asm, pwm16_ch0.asm, psocconfigtbl.asm, psocconfig.asm, led_blue.asm ]
[ counter8_rf_clkint.asm, counter8_rf_clk.asm, counter16_pwrupdint.asm, counter16_pwrupd.asm ]
[ rZ.asm, rY.asm, rX.asm, r11.asm, r10.asm, r9.asm, r8.asm, r7.asm, r6.asm, r5.asm, r4.asm, r3.asm, r2.asm ]
[ r1.asm, r0.asm, divmodu_8x8_8_util.asm, i_div8_block_util.asm, atoi.s, isdigit.s, isspace.s, ctype.s ]
[ strncat.s, strncpy.s ]

Area Definitions

-blit:0x190.0x3fff
-bdata:0.0x2ff
-BInterruptRAM:0	allocated at: 0
-Bvirtual_registers	allocated at: 0xcc
-Bfitolamp_slave_RAM	allocated at: 0
-rwh:SSCParmBlk:0xf7.0xff

