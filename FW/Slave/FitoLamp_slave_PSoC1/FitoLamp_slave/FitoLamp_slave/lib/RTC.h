//*****************************************************************************
//*****************************************************************************
//  FILENAME: RTC.h
//   Version: 1.10, Updated on 2015/3/4 at 22:19:15
//  Generated by PSoC Designer 5.4.3191
//
//  DESCRIPTION: RTC User Module C Language interface file
//-----------------------------------------------------------------------------
//  Copyright (c) Cypress Semiconductor 2015. All Rights Reserved.
//*****************************************************************************
//*****************************************************************************
#ifndef RTC_INCLUDE
#define RTC_INCLUDE

#include <m8c.h>

#pragma fastcall16 RTC_EnableInt
#pragma fastcall16 RTC_DisableInt
#pragma fastcall16 RTC_ClearInt
#pragma fastcall16 RTC_Start
#pragma fastcall16 RTC_Stop
#pragma fastcall16 RTC_SetIntPeriod              // Write RTCCR
#pragma fastcall16 RTC_bReadSecond               // Read  RTCS 
#pragma fastcall16 RTC_bReadMinute               // Read  RTCM 
#pragma fastcall16 RTC_bReadHour                 // Read  RTCH
#pragma fastcall16 RTC_SetSecond                 // Write RTCS
#pragma fastcall16 RTC_SetMinute                 // Write RTCM
#pragma fastcall16 RTC_SetHour                   // Write RTCH

//-------------------------------------------------
// Prototypes of the RTC API.
//-------------------------------------------------

extern void RTC_EnableInt(void);
extern void RTC_DisableInt(void);
extern void RTC_ClearInt(void);
extern void RTC_Start(void);
extern void RTC_Stop(void);
extern void RTC_SetIntPeriod(BYTE bConfiguration);
extern BYTE RTC_bReadSecond(void);
extern BYTE RTC_bReadMinute(void);
extern BYTE RTC_bReadHour(void);
extern void RTC_SetSecond(BYTE bSecond);
extern void RTC_SetMinute(BYTE bMinute);
extern void RTC_SetHour(BYTE bHour);

//--------------------------------------------------
// RTC interrupt selection definitions
//--------------------------------------------------
#define RTC_INT_SEC                                        (0x00)
#define RTC_INT_MIN                                        (0x04)
#define RTC_INT_HOUR                                       (0x08)
#define RTC_INT_DAY                                        (0x0C)

//--------------------------------------------------
// Constants for RTC API's.
//--------------------------------------------------

#define RTC_RTCCR_REG_START_BIT                            ( 0x01 )
#define RTC_INT_MASK                                       ( 0x20 )
#define RTC_INT_SEL_MASK                                   ( 0x0C )
#define RTC_INT_REG_ADDR                                   ( 0x0de )
#define RTC_INT_RTC_MASK                                   ( 0x08 )

//-------------------------------------------------
// Register Addresses for RTC
//-------------------------------------------------

#pragma ioport  RTC_RTCH_REG:   0x1a4                           //RTC Hour register
BYTE            RTC_RTCH_REG;
#pragma ioport  RTC_RTCM_REG:   0x1a5                           //RTC Minute register
BYTE            RTC_RTCM_REG;
#pragma ioport  RTC_RTCS_REG:   0x1a6                           //RTC Second register
BYTE            RTC_RTCS_REG;
#pragma ioport  RTC_RTCCR_REG:  0x1a7                           //Control register
BYTE            RTC_RTCCR_REG;

//-------------------------------------------------
// RTC Macro 'Functions'
//-------------------------------------------------

#define RTC_Start_M \
   ( RTC_RTCCR_REG |=  RTC_RTCCR_REG_START_BIT )

#define RTC_Stop_M  \
   ( RTC_RTCCR_REG &= ~RTC_RTCCR_REG_START_BIT )

#define RTC_EnableInt_M   \
   M8C_EnableIntMask(RTC_RTCCR_REG, RTC_INT_MASK) ;\
   M8C_EnableIntMask(RTC_INT_REG_ADDR, RTC_INT_RTC_MASK) ;\

#define RTC_DisableInt_M  \
   M8C_DisableIntMask(RTC_RTCCR_REG, RTC_INT_MASK) ;\
   M8C_DisableIntMask(RTC_INT_REG_ADDR, RTC_INT_RTC_MASK) ;\

#endif
// end of file RTC.h