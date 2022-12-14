//*****************************************************************************
//*****************************************************************************
//  FILENAME: PWM16_CH2.h
//   Version: 2.5, Updated on 2015/3/4 at 22:26:51
//  Generated by PSoC Designer 5.4.3191
//
//  DESCRIPTION: PWM16 User Module C Language interface file
//-----------------------------------------------------------------------------
//  Copyright (c) Cypress Semiconductor 2015. All Rights Reserved.
//*****************************************************************************
//*****************************************************************************
#ifndef PWM16_CH2_INCLUDE
#define PWM16_CH2_INCLUDE

#include <m8c.h>

#pragma fastcall16 PWM16_CH2_EnableInt
#pragma fastcall16 PWM16_CH2_DisableInt
#pragma fastcall16 PWM16_CH2_Start
#pragma fastcall16 PWM16_CH2_Stop
#pragma fastcall16 PWM16_CH2_wReadCounter              // Read  DR0
#pragma fastcall16 PWM16_CH2_WritePeriod               // Write DR1
#pragma fastcall16 PWM16_CH2_wReadPulseWidth           // Read  DR2
#pragma fastcall16 PWM16_CH2_WritePulseWidth           // Write DR2

// The following symbols are deprecated.
// They may be omitted in future releases
//
#pragma fastcall16 wPWM16_CH2_ReadCounter              // Read  DR0 (Deprecated)
#pragma fastcall16 wPWM16_CH2_ReadPulseWidth           // Read  DR2 (Deprecated)


//-------------------------------------------------
// Prototypes of the PWM16_CH2 API.
//-------------------------------------------------

extern void PWM16_CH2_EnableInt(void);                  // Proxy Class 1
extern void PWM16_CH2_DisableInt(void);                 // Proxy Class 1
extern void PWM16_CH2_Start(void);                      // Proxy Class 1
extern void PWM16_CH2_Stop(void);                       // Proxy Class 1
extern WORD PWM16_CH2_wReadCounter(void);               // Proxy Class 2
extern void PWM16_CH2_WritePeriod(WORD wPeriod);        // Proxy Class 1
extern WORD PWM16_CH2_wReadPulseWidth(void);            // Proxy Class 1
extern void PWM16_CH2_WritePulseWidth(WORD wPulseWidth);// Proxy Class 1

// The following functions are deprecated.
// They may be omitted in future releases
//
extern WORD wPWM16_CH2_ReadCounter(void);            // Deprecated
extern WORD wPWM16_CH2_ReadPulseWidth(void);         // Deprecated


//-------------------------------------------------
// Constants for PWM16_CH2 API's.
//-------------------------------------------------

#define PWM16_CH2_CONTROL_REG_START_BIT        ( 0x01 )
#define PWM16_CH2_INT_REG_ADDR                 ( 0x0e1 )
#define PWM16_CH2_INT_MASK                     ( 0x08 )


//--------------------------------------------------
// Constants for PWM16_CH2 user defined values
//--------------------------------------------------

#define PWM16_CH2_PERIOD                       ( 0xfde8 )
#define PWM16_CH2_PULSE_WIDTH                  ( 0x7530 )


//-------------------------------------------------
// Register Addresses for PWM16_CH2
//-------------------------------------------------

#pragma ioport  PWM16_CH2_COUNTER_LSB_REG:  0x028          //DR0 Count register LSB
BYTE            PWM16_CH2_COUNTER_LSB_REG;
#pragma ioport  PWM16_CH2_COUNTER_MSB_REG:  0x02c          //DR0 Count register MSB
BYTE            PWM16_CH2_COUNTER_MSB_REG;
#pragma ioport  PWM16_CH2_PERIOD_LSB_REG:   0x029          //DR1 Period register LSB
BYTE            PWM16_CH2_PERIOD_LSB_REG;
#pragma ioport  PWM16_CH2_PERIOD_MSB_REG:   0x02d          //DR1 Period register MSB
BYTE            PWM16_CH2_PERIOD_MSB_REG;
#pragma ioport  PWM16_CH2_COMPARE_LSB_REG:  0x02a          //DR2 Compare register LSB
BYTE            PWM16_CH2_COMPARE_LSB_REG;
#pragma ioport  PWM16_CH2_COMPARE_MSB_REG:  0x02e          //DR2 Compare register MSB
BYTE            PWM16_CH2_COMPARE_MSB_REG;
#pragma ioport  PWM16_CH2_CONTROL_LSB_REG:  0x02b          //Control register LSB
BYTE            PWM16_CH2_CONTROL_LSB_REG;
#pragma ioport  PWM16_CH2_CONTROL_MSB_REG:  0x02f          //Control register MSB
BYTE            PWM16_CH2_CONTROL_MSB_REG;
#pragma ioport  PWM16_CH2_FUNC_LSB_REG: 0x128              //Function register LSB
BYTE            PWM16_CH2_FUNC_LSB_REG;
#pragma ioport  PWM16_CH2_FUNC_MSB_REG: 0x12c              //Function register MSB
BYTE            PWM16_CH2_FUNC_MSB_REG;
#pragma ioport  PWM16_CH2_INPUT_LSB_REG:    0x129          //Input register LSB
BYTE            PWM16_CH2_INPUT_LSB_REG;
#pragma ioport  PWM16_CH2_INPUT_MSB_REG:    0x12d          //Input register MSB
BYTE            PWM16_CH2_INPUT_MSB_REG;
#pragma ioport  PWM16_CH2_OUTPUT_LSB_REG:   0x12a          //Output register LSB
BYTE            PWM16_CH2_OUTPUT_LSB_REG;
#pragma ioport  PWM16_CH2_OUTPUT_MSB_REG:   0x12e          //Output register MSB
BYTE            PWM16_CH2_OUTPUT_MSB_REG;
#pragma ioport  PWM16_CH2_INT_REG:       0x0e1             //Interrupt Mask Register
BYTE            PWM16_CH2_INT_REG;


//-------------------------------------------------
// PWM16_CH2 Macro 'Functions'
//-------------------------------------------------

#define PWM16_CH2_Start_M \
   ( PWM16_CH2_CONTROL_LSB_REG |=  PWM16_CH2_CONTROL_REG_START_BIT )

#define PWM16_CH2_Stop_M  \
   ( PWM16_CH2_CONTROL_LSB_REG &= ~PWM16_CH2_CONTROL_REG_START_BIT )

#define PWM16_CH2_EnableInt_M   \
   M8C_EnableIntMask(  PWM16_CH2_INT_REG, PWM16_CH2_INT_MASK )

#define PWM16_CH2_DisableInt_M  \
   M8C_DisableIntMask( PWM16_CH2_INT_REG, PWM16_CH2_INT_MASK )

#endif
// end of file PWM16_CH2.h
