Project:      output/FitoLamp_slave
Build Number: 0
Date:         Sun Nov 06 13:29:26 2022
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
                     psoc_config   041F   012A =    298. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       041F  LoadConfigTBL_fitolamp_slave_Ordered
       04E4  _LoadConfigInit
       04E4  LoadConfigInit
       04EE  _LoadConfig_fitolamp_slave
       04EE  LoadConfig_fitolamp_slave
       0549  __psoc_config_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     UserModules   0549   0698 =   1688. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       0549  _RX8_RF_ISR
       064B  _RX8_RF_EnableInt
       064B  RX8_RF_EnableInt
       064F  RX8_RF_DisableInt
       064F  _RX8_RF_DisableInt
       0653  RX8_RF_Start
       0653  _RX8_RF_Start
       0658  RX8_RF_Stop
       0658  _RX8_RF_Stop
       065C  _RX8_RF_bReadRxData
       065C  bRX8_RF_ReadRxData
       065C  RX8_RF_bReadRxData
       065C  _bRX8_RF_ReadRxData
       065F  _bRX8_RF_ReadRxStatus
       065F  _RX8_RF_bReadRxStatus
       065F  bRX8_RF_ReadRxStatus
       065F  RX8_RF_bReadRxStatus
       0662  RX8_RF_cGetChar
       0662  _RX8_RF_cGetChar
       066A  _RX8_RF_cReadChar
       066A  RX8_RF_cReadChar
       0681  _RX8_RF_iReadChar
       0681  RX8_RF_iReadChar
       0697  _RX8_RF_CmdReset
       0697  RX8_RF_CmdReset
       06AD  RX8_RF_bCmdCheck
       06AD  _RX8_RF_bCmdCheck
       06B5  RX8_RF_bErrCheck
       06B5  _RX8_RF_bErrCheck
       06C0  RX8_RF_bCmdLength
       06C0  _RX8_RF_bCmdLength
       06C6  _RX8_RF_szGetParam
       06C6  RX8_RF_szGetParam
       0724  _RX8_RF_szGetRestOfParams
       0724  RX8_RF_szGetRestOfParams
       072F  _RX8_GPS_ISR
       0831  RX8_GPS_EnableInt
       0831  _RX8_GPS_EnableInt
       0835  _RX8_GPS_DisableInt
       0835  RX8_GPS_DisableInt
       0839  RX8_GPS_Start
       0839  _RX8_GPS_Start
       083E  RX8_GPS_Stop
       083E  _RX8_GPS_Stop
       0842  bRX8_GPS_ReadRxData
       0842  _RX8_GPS_bReadRxData
       0842  RX8_GPS_bReadRxData
       0842  _bRX8_GPS_ReadRxData
       0845  _bRX8_GPS_ReadRxStatus
       0845  _RX8_GPS_bReadRxStatus
       0845  RX8_GPS_bReadRxStatus
       0845  bRX8_GPS_ReadRxStatus
       0848  _RX8_GPS_cGetChar
       0848  RX8_GPS_cGetChar
       0850  RX8_GPS_cReadChar
       0850  _RX8_GPS_cReadChar
       0867  _RX8_GPS_iReadChar
       0867  RX8_GPS_iReadChar
       087D  RX8_GPS_CmdReset
       087D  _RX8_GPS_CmdReset
       0893  _RX8_GPS_bCmdCheck
       0893  RX8_GPS_bCmdCheck
       089B  _RX8_GPS_bErrCheck
       089B  RX8_GPS_bErrCheck
       08A6  _RX8_GPS_bCmdLength
       08A6  RX8_GPS_bCmdLength
       08AC  _RX8_GPS_szGetParam
       08AC  RX8_GPS_szGetParam
       090A  _RX8_GPS_szGetRestOfParams
       090A  RX8_GPS_szGetRestOfParams
       0915  _RTC_ISR
       0916  _RTC_EnableInt
       0916  RTC_EnableInt
       0921  RTC_DisableInt
       0921  _RTC_DisableInt
       092C  _RTC_ClearInt
       092C  RTC_ClearInt
       0930  _RTC_Start
       0930  RTC_Start
       0938  _RTC_Stop
       0938  RTC_Stop
       0940  RTC_SetIntPeriod
       0940  _RTC_SetIntPeriod
       0956  RTC_bReadSecond
       0956  _RTC_bReadSecond
       095D  RTC_bReadMinute
       095D  _RTC_bReadMinute
       0964  _RTC_bReadHour
       0964  RTC_bReadHour
       096B  _RTC_SetSecond
       096B  RTC_SetSecond
       0983  _RTC_SetMinute
       0983  RTC_SetMinute
       099B  _RTC_SetHour
       099B  RTC_SetHour
       09B3  _PWM16_CH1_ISR
       09B4  _PWM16_CH1_EnableInt
       09B4  PWM16_CH1_EnableInt
       09B8  PWM16_CH1_DisableInt
       09B8  _PWM16_CH1_DisableInt
       09BC  _PWM16_CH1_Start
       09BC  PWM16_CH1_Start
       09C0  PWM16_CH1_Stop
       09C0  _PWM16_CH1_Stop
       09C4  _PWM16_CH1_WritePeriod
       09C4  PWM16_CH1_WritePeriod
       09CA  _PWM16_CH1_WritePulseWidth
       09CA  PWM16_CH1_WritePulseWidth
       09D0  _wPWM16_CH1_ReadPulseWidth
       09D0  _PWM16_CH1_wReadPulseWidth
       09D0  wPWM16_CH1_ReadPulseWidth
       09D0  PWM16_CH1_wReadPulseWidth
       09D6  _PWM16_CH1_wReadCounter
       09D6  _wPWM16_CH1_ReadCounter
       09D6  wPWM16_CH1_ReadCounter
       09D6  PWM16_CH1_wReadCounter
       0A10  _PWM16_CH0_ISR
       0A11  PWM16_CH0_EnableInt
       0A11  _PWM16_CH0_EnableInt
       0A15  _PWM16_CH0_DisableInt
       0A15  PWM16_CH0_DisableInt
       0A19  _PWM16_CH0_Start
       0A19  PWM16_CH0_Start
       0A1D  _PWM16_CH0_Stop
       0A1D  PWM16_CH0_Stop
       0A21  _PWM16_CH0_WritePeriod
       0A21  PWM16_CH0_WritePeriod
       0A27  _PWM16_CH0_WritePulseWidth
       0A27  PWM16_CH0_WritePulseWidth
       0A2D  wPWM16_CH0_ReadPulseWidth
       0A2D  PWM16_CH0_wReadPulseWidth
       0A2D  _PWM16_CH0_wReadPulseWidth
       0A2D  _wPWM16_CH0_ReadPulseWidth
       0A33  _wPWM16_CH0_ReadCounter
       0A33  wPWM16_CH0_ReadCounter
       0A33  PWM16_CH0_wReadCounter
       0A33  _PWM16_CH0_wReadCounter
       0A6D  _LED_Blue_On
       0A6D  LED_Blue_On
       0A71  LED_Blue_Start
       0A71  LED_Blue_Off
       0A71  _LED_Blue_Start
       0A71  _LED_Blue_Stop
       0A71  _LED_Blue_Off
       0A71  LED_Blue_Stop
       0A73  LED_Blue_Switch
       0A73  _LED_Blue_Switch
       0A8B  _LED_Blue_Invert
       0A8B  LED_Blue_Invert
       0A9A  _LED_Blue_GetState
       0A9A  LED_Blue_GetState
       0AAB  _Counter8_RF_clk_ISR
       0AAC  Counter8_RF_clk_EnableInt
       0AAC  _Counter8_RF_clk_EnableInt
       0AB0  _Counter8_RF_clk_DisableInt
       0AB0  Counter8_RF_clk_DisableInt
       0AB4  _Counter8_RF_clk_Start
       0AB4  Counter8_RF_clk_Start
       0AB8  _Counter8_RF_clk_Stop
       0AB8  Counter8_RF_clk_Stop
       0ABC  Counter8_RF_clk_WritePeriod
       0ABC  _Counter8_RF_clk_WritePeriod
       0ABF  Counter8_RF_clk_WriteCompareValue
       0ABF  _Counter8_RF_clk_WriteCompareValue
       0AC2  _Counter8_RF_clk_bReadCompareValue
       0AC2  Counter8_RF_clk_bReadCompareValue
       0AC2  bCounter8_RF_clk_ReadCompareValue
       0AC2  _bCounter8_RF_clk_ReadCompareValue
       0AC5  _bCounter8_RF_clk_ReadCounter
       0AC5  _Counter8_RF_clk_bReadCounter
       0AC5  bCounter8_RF_clk_ReadCounter
       0AC5  Counter8_RF_clk_bReadCounter
       0AF6  _Counter16_PwrUpd_ISR
       0B81  Counter16_PwrUpd_EnableInt
       0B81  _Counter16_PwrUpd_EnableInt
       0B85  _Counter16_PwrUpd_DisableInt
       0B85  Counter16_PwrUpd_DisableInt
       0B89  Counter16_PwrUpd_Start
       0B89  _Counter16_PwrUpd_Start
       0B8D  Counter16_PwrUpd_Stop
       0B8D  _Counter16_PwrUpd_Stop
       0B91  _Counter16_PwrUpd_WritePeriod
       0B91  Counter16_PwrUpd_WritePeriod
       0B97  Counter16_PwrUpd_WriteCompareValue
       0B97  _Counter16_PwrUpd_WriteCompareValue
       0B9D  _Counter16_PwrUpd_wReadCompareValue
       0B9D  _wCounter16_PwrUpd_ReadCompareValue
       0B9D  Counter16_PwrUpd_wReadCompareValue
       0B9D  wCounter16_PwrUpd_ReadCompareValue
       0BA3  _Counter16_PwrUpd_wReadCounter
       0BA3  Counter16_PwrUpd_wReadCounter
       0BA3  wCounter16_PwrUpd_ReadCounter
       0BA3  _wCounter16_PwrUpd_ReadCounter
       0BE1  __UserModules_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   0BE1   0F92 =   3986. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       0BE1  Delay10msTimes
       0BE1  _Delay10msTimes
       0BE1  __text_start
       0BEE  _Delay50uTimes
       0BEE  Delay50uTimes
       0BF4  Delay50u_Table
       0BFC  Delay50u
       0BFC  _Delay50u
       0C10  _gps_signal
       0CC5  _rf_signal
       0D90  _main
       0FBD  _set_power
       0FDC  _override_enable
       0FF5  _update_power
       1079  _schedule_processing
       10F9  _schedule_init
       1137  _rtc_update
       1199  _check_fld
       11E6  _NMEA_GetField
       1321  _NMEA_handle_packet
       1449  _NMEA_GetTimeUTC
       165D  _str_cmp
       16AE  _str_cmp_const
       1700  _byte_to_bcd
       1737  _bcd_to_byte
       176E  _utc_to_local
       1861  _strlenLMM
       186C  _strlen
       1887  _atoi
       19C8  _isdigit
       19E9  _isspace
       1A0A  _strncat
       1AB7  _strncpy
       1B73  __text_end
       1B73  __xidata_start
       1BCE  __xidata_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             TOP   0000   0154 =    340. bytes (abs,ovr,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       0080  __Start
       0151  _bGetPowerSetting
       0151  bGetPowerSetting

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                    InterruptRAM   0000   0005 =      5. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0000  _RX8_RF_fStatus
       0000  RX8_RF_fStatus
       0001  _RX8_RF_bRxCnt
       0001  RX8_RF_bRxCnt
       0002  RX8_GPS_fStatus
       0002  _RX8_GPS_fStatus
       0003  _RX8_GPS_bRxCnt
       0003  RX8_GPS_bRxCnt
       0004  NO_SHADOW
       0004  Port_1_Data_SHADE
       0004  _Port_1_Data_SHADE
       0004  _NO_SHADOW

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     RX8_GPS_RAM   0000   0011 =     17. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       00B9  _RX8_GPS_aRxBuffer
       00B9  RX8_GPS_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                      RX8_RF_RAM   0000   0011 =     17. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       00A9  RX8_RF_aRxBuffer
       00A9  _RX8_RF_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
               virtual_registers   00C9   000F =     15. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       00C9  __rZ
       00CA  __rY
       00CB  __rX
       00CC  __r11
       00CD  __r10
       00CE  __r9
       00CF  __r8
       00D0  __r7
       00D1  __r6
       00D2  __r5
       00D3  __r4
       00D4  __r3
       00D5  __r2
       00D6  __r1
       00D7  __r0

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            data   0000   01C2 =    450. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0000  __data_start
       0005  _NMEA_buffer_gps
       0057  _NMEA_SHFTL
       00D8  _local_datetime
       00E0  _gps_datetime
       00E8  _nmea_shftl_empty
       00EE  _nmea_gprmc_empty
       00F4  _NMEA_cmd_received
       00F6  _NMEA_pointer_rf
       0100  _NMEA_GPRMC
       0152  _NMEA_buffer_rf
       01A4  _fld_buf
       01F6  _override_counter
       01FA  _power_target
       01FC  _override
       01FE  _NMEA_pointer_gps

Files Linked      [ module(s) ]

./obj/boot.o	[ boot.asm ]
obj/delay.o	[ delay.asm ]
obj/main.o	[ main.c ]
<library>	[ rx8_rfint.asm, rx8_rf.asm, rx8_gpsint.asm, rx8_gps.asm, rtcint.asm, rtc.asm, pwm16_ch1int.asm ]
[ pwm16_ch1.asm, pwm16_ch0int.asm, pwm16_ch0.asm, psocconfigtbl.asm, psocconfig.asm, led_blue.asm ]
[ counter8_rf_clkint.asm, counter8_rf_clk.asm, counter16_pwrupdint.asm, counter16_pwrupd.asm ]
[ rZ.asm, rY.asm, rX.asm, r11.asm, r10.asm, r9.asm, r8.asm, r7.asm, r6.asm, r5.asm, r4.asm, r3.asm, r2.asm ]
[ r1.asm, r0.asm, divmodu_8x8_8_util.asm, i_div8_block_util.asm, strlen.s, atoi.s, isdigit.s, isspace.s ]
[ ctype.s, strncat.s, strncpy.s ]

Area Definitions

-blit:0x190.0x3fff
-bdata:0.0x2ff
-BInterruptRAM:0	allocated at: 0
-Bvirtual_registers	allocated at: 0xc9
-Bfitolamp_slave_RAM	allocated at: 0
-rwh:SSCParmBlk:0xf7.0xff

