;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME: RTC.asm
;;   Version: 1.10, Updated on 2015/3/4 at 22:19:15
;;  Generated by PSoC Designer 5.4.3191
;;
;;  DESCRIPTION: RTC User Module software implementation file
;;
;;  NOTE: User Module APIs conform to the fastcall16 convention for marshalling
;;        arguments and observe the associated "Registers are volatile" policy.
;;        This means it is the caller's responsibility to preserve any values
;;        in the X and A registers that are still needed after the API functions
;;        returns. For Large Memory Model devices it is also the caller's 
;;        responsibility to perserve any value in the CUR_PP, IDX_PP, MVR_PP and 
;;        MVW_PP registers. Even though some of these registers may not be modified
;;        now, there is no guarantee that will remain the case in future releases.
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress Semiconductor 2015. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************

include "m8c.inc"
include "memory.inc"
include "RTC.inc"

;-----------------------------------------------
;  Global Symbols
;-----------------------------------------------
export  RTC_EnableInt
export _RTC_EnableInt
export  RTC_DisableInt
export _RTC_DisableInt
export  RTC_ClearInt
export _RTC_ClearInt
export  RTC_Start
export _RTC_Start
export  RTC_Stop
export _RTC_Stop
export  RTC_SetIntPeriod
export _RTC_SetIntPeriod
export  RTC_bReadSecond
export _RTC_bReadSecond
export  RTC_bReadMinute
export _RTC_bReadMinute
export  RTC_bReadHour
export _RTC_bReadHour
export  RTC_SetSecond
export _RTC_SetSecond
export  RTC_SetMinute
export _RTC_SetMinute
export  RTC_SetHour
export _RTC_SetHour


;-----------------------------------------------
;  EQUATES
;-----------------------------------------------

;  Time validation constants
SEC_MAX_IN_BSD:           equ 60h
MIN_MAX_IN_BSD:           equ 60h
HOUR_MAX_IN_BSD:          equ 24h

;  Valid parameter range flag constants
VALID_RANGE_INPUT:        equ 01h
INVALID_RANGE_INPUT:      equ 00h

LOW_TETRAD_MASK:          equ 0Fh
MIN_INVALID_BSD_VALUE:    equ 0Ah

AREA InterruptRAM (RAM,REL,CON)

;-----------------------------------------------
;  Constant Definitions
;-----------------------------------------------


;-----------------------------------------------
; Variable Allocation
;-----------------------------------------------

AREA UserModules (ROM, REL)

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: RTC_EnableInt
;
;  DESCRIPTION:
;     Enables this RTC's interrupt by setting the interrupt enable mask bit
;     associated with this User Module. This function has no effect until and
;     unless the global interrupts are enabled (for example by using the
;     macro M8C_EnableGInt).
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None.
;  RETURNS:      None.
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 RTC_EnableInt:
_RTC_EnableInt:
   RAM_PROLOGUE RAM_USE_CLASS_1
   RTC_EnableInt_M
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: RTC_DisableInt
;
;  DESCRIPTION:
;     Disables this RTC's interrupt by clearing the interrupt enable
;     mask bit associated with this User Module.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      None
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 RTC_DisableInt:
_RTC_DisableInt:
   RAM_PROLOGUE RAM_USE_CLASS_1
   RTC_DisableInt_M
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: RTC_ClearInt
;
;  DESCRIPTION:
;
;     NOTE:  Remember to enable the global interrupt by calling the
;           M8C global macro: M8C_EnableGInt
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:  none
;
;  RETURNS:  none
;
;  SIDE EFFECTS:
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16
;    functions.
;
;  THEORY of OPERATION or PROCEDURE:
;     Sets the specific user module interrupt enable mask bit.
;
 RTC_ClearInt:
_RTC_ClearInt:
   RAM_PROLOGUE RAM_USE_CLASS_1
   M8C_ClearIntFlag RTC_CLR_INT_REG, RTC_INT_RTC_MASK
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret
.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: RTC_Start
;
;  DESCRIPTION:
;     Enable the RTC UM to start the real timer clock. 
;     Set the start bit in RTC control register
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      None
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 RTC_Start:
_RTC_Start:
   RAM_PROLOGUE RAM_USE_CLASS_1
   M8C_SetBank1
   RTC_Start_M
   M8C_SetBank0
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: RTC_Stop
;
;  DESCRIPTION:
;     Disable the RTC UM to stop the real timer clock 
;      by clearing the start bit in the Control
;     register.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      None
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 RTC_Stop:
_RTC_Stop:
   RAM_PROLOGUE RAM_USE_CLASS_1
   M8C_SetBank1
   RTC_Stop_M
   M8C_SetBank0
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: RTC_SetIntPeriod
;
;  DESCRIPTION:
;    Configure the interrupt period of RTC module. 
;    The period can be 1 second, 1 minute, 1 hour or 1 day.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS: fastcall16 void SetIntPeriod (passed in A)
;  RETURNS:   None
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 RTC_SetIntPeriod:
_RTC_SetIntPeriod:
   RAM_PROLOGUE RAM_USE_CLASS_2
   and   A, RTC_INT_SEL_MASK   
   mov   X, SP
   push  A                                  ; store value in stack
   M8C_SetBank1
   mov   A, reg[RTC_RTCCR_REG]  ; get register value
   and   A, ~RTC_INT_SEL_MASK   ; clear INT_SEL bits
   or    A, [X]                              ; set INT_SEL bits
   mov   reg[RTC_RTCCR_REG], A  ; set new register value
   M8C_SetBank0
   pop   A
   RAM_EPILOGUE RAM_USE_CLASS_2
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: RTC_bReadSecond
;
;  DESCRIPTION:
;     This function will access the data in seconds register at address 1A6h.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:   None
;  RETURNS:     fastcall16 BYTE bReadSecond (void)
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 RTC_bReadSecond:
_RTC_bReadSecond:
   RAM_PROLOGUE RAM_USE_CLASS_1
   M8C_SetBank1
   mov   A, reg[RTC_RTCS_REG] ; Read second value in BCD format
   M8C_SetBank0
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: RTC_bReadMinute
;
;  DESCRIPTION:
;     This function will access the data in minutes register at address 1A5h.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:   None
;  RETURNS:     fastcall16 BYTE bReadMinute (void)
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 RTC_bReadMinute:
_RTC_bReadMinute:
   RAM_PROLOGUE RAM_USE_CLASS_1
   M8C_SetBank1
   mov   A, reg[RTC_RTCM_REG]   ; Read minute value in BCD format
   M8C_SetBank0
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: RTC_bReadHour
;
;  DESCRIPTION:
;     This function will access the data in hours register at address 1A4h.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:   None
;  RETURNS:     fastcall16 BYTE bReadHour(void)
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 RTC_bReadHour:
_RTC_bReadHour:
   RAM_PROLOGUE RAM_USE_CLASS_1
   M8C_SetBank1
   mov  A,reg[RTC_RTCH_REG]  ; Read hour value in BCD format
   M8C_SetBank0
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: RTC_SetSecond
;
;  DESCRIPTION:
;    This function will set the second valuein seconds register at address 1A6h.
;    The legal range for writing the second value is 0 to 59. This data must be in BCD format.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    fastcall16 void SetSecond(passed in A)
;  RETURNS:    None
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 RTC_SetSecond:
_RTC_SetSecond:
   RAM_PROLOGUE RAM_USE_CLASS_1
   cmp   A, SEC_MAX_IN_BSD                   ; the sec value (in BCD) validation
   jnc   .SetSec_skip                        ; skip sec Reg Update if BCD sec is invalid (>0x59)
   push  A
   and   A, LOW_TETRAD_MASK
   cmp   A, MIN_INVALID_BSD_VALUE
   pop   A
   jnc   .SetSec_skip                        ; skip sec Reg Update if sec is not BCD 
   M8C_SetBank1
   mov   reg[RTC_RTCS_REG], A   ; Set new second value (in BCD)
   M8C_SetBank0
   mov   A, VALID_RANGE_INPUT                ; Set Flag to Valid
   RAM_EPILOGUE RAM_USE_CLASS_1   
   ret
.SetSec_skip:
   mov   A, INVALID_RANGE_INPUT              ; Set Flag to Invalid
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret
.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: RTC_SetMinute
;
;  DESCRIPTION:
;    This function will set the minute value in minute register at address 1A5h.
;    The legal range for writing the minute value is 0 to 59. This data must be in BCD format.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    fastcall16 void SetMinute(passed in A)
;  RETURNS:    None
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 RTC_SetMinute:
_RTC_SetMinute:
   RAM_PROLOGUE RAM_USE_CLASS_1
   cmp   A, MIN_MAX_IN_BSD                   ; the minute value (in BCD) validation
   jnc   .SetMin_skip                        ; skip minute Reg Update if BCD minute is invalid (>0x59)
   push  A
   and   A, LOW_TETRAD_MASK
   cmp   A, MIN_INVALID_BSD_VALUE
   pop   A
   jnc   .SetMin_skip                        ; skip sec Reg Update if minute is not BCD 
   M8C_SetBank1
   mov   reg[RTC_RTCM_REG], A   ; Set new minute value (in BCD)
   M8C_SetBank0
   mov   A, VALID_RANGE_INPUT                ; Set Flag to Valid
   RAM_EPILOGUE RAM_USE_CLASS_1   
   ret
.SetMin_skip:
   mov   A, INVALID_RANGE_INPUT              ; Set Flag to Invalid
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: RTC_SetHour
;
;  DESCRIPTION:
;    This function will set the hour value in hour register at address 1A4h.
;    The legal range for writing the hour value is 0 to 23. This data must be in BCD format.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    fastcall16 void SetHour(passed in A)
;  RETURNS:    None
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 RTC_SetHour:
_RTC_SetHour:
   RAM_PROLOGUE RAM_USE_CLASS_1
   cmp   A, HOUR_MAX_IN_BSD                  ; the hour value (in BCD) validation
   jnc   .SetHour_skip                       ; skip hour Reg Update if BCD hour is invalid (>0x59)
   push  A
   and   A, LOW_TETRAD_MASK
   cmp   A, MIN_INVALID_BSD_VALUE
   pop   A
   jnc   .SetHour_skip                        ; skip sec Reg Update if hour is not BCD 
   M8C_SetBank1
   mov   reg[RTC_RTCH_REG], A   ; Set new hour value (in BCD)
   M8C_SetBank0
   mov   A, VALID_RANGE_INPUT                ; Set Flag to Valid
   RAM_EPILOGUE RAM_USE_CLASS_1   
   ret
.SetHour_skip:
   mov   A, INVALID_RANGE_INPUT              ; Set Flag to Invalid
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION
; End of File RTC.asm
