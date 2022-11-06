PROJNAME=FitoLamp_slave
DEVICE=CY8C28445
BASEDEVICE=CY8C28045
PROJPATH=C:/Work/FitoLamp/FW/Slave/FITOLA~1/FITOLA~1/FITOLA~1
PSOCDIR=C:/PROGRA~2/Cypress/PSOCDE~1/5.4/Common/CYPRES~1
INCLUDE_PATH=C:/PROGRA~2/Cypress/PSOCDE~1/5.4/Common/CYPRES~1/tools/include/CY8C28~1
CSRCS= main.c
LIBCSRCS=
STDCSRCS=
ASMSRCS= delay.asm
LIBASMSRCS= counter16_pwrupd.asm counter16_pwrupdint.asm counter8_rf_clk.asm counter8_rf_clkint.asm lcd.asm led_blue.asm psocconfig.asm psocconfigtbl.asm pwm16_ch0.asm pwm16_ch0int.asm pwm16_ch1.asm pwm16_ch1int.asm rtc.asm rtcint.asm rx8_gps.asm rx8_gpsint.asm rx8_gpsplus.asm rx8_rf.asm rx8_rfint.asm rx8_rfplus.asm tx8_debug.asm tx8_debugint.asm tx8_debugplus.asm
STDASMSRCS=
OBJECT_SOURCES= delay.asm main.c
FILLVALUE=0x30
LASTROM=0x3fff
LASTRAM=0x3ff
LAST_DATARAM=0x2ff
CODECOMPRESSOR=
MORE_CFLAGS=-Wf-Osize -Wf-LMM4 -D_LMM
RELSTART=0x190
INC_PATH=
CDEFINES=
LIBS=
UMLIBS=
LIB_PATH=
ENABLE_ALIGN_SHIFT=0
LMM=1
SYS_INC_CONTENTS:=SYSTEM_STACK_PAGE:_equ_3 SYSTEM_STACK_BASE_ADDR:_equ_0h SYSTEM_LARGE_MEMORY_MODEL:_equ_1 SYSTEM_SMALL_MEMORY_MODEL:_equ_0 IMAGECRAFT:_equ_1 HITECH:_equ_2 TOOLCHAIN:_equ_IMAGECRAFT 
SYSTEM_TOOLS=1
CSFLOW=
CONFIG_NAMES=fitolamp_slave 
