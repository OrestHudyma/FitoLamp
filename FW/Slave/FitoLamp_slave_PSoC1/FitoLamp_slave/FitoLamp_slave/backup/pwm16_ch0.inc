;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME: PWM16_CH0.inc
;;   Version: 2.5, Updated on 2015/3/4 at 22:26:51
;;  Generated by PSoC Designer 5.4.3191
;;
;;  DESCRIPTION: Assembler declarations for the PWM16 user module interface
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress Semiconductor 2015. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************

include "m8c.inc"


;--------------------------------------------------
; Constants for PWM16_CH0 API's.
;--------------------------------------------------

PWM16_CH0_CONTROL_REG_START_BIT:           equ 0x01    ; Control register start bit
PWM16_CH0_INT_REG:                         equ 0x0e1
PWM16_CH0_INT_MASK:                        equ 0x02


;--------------------------------------------------
; Constants for PWM16_CH0 user defined values
;--------------------------------------------------

PWM16_CH0_PERIOD:                          equ 0x4e20
PWM16_CH0_PULSE_WIDTH:                     equ 0x0


;--------------------------------------------------
; Register Address Constants for PWM16_CH0
;--------------------------------------------------

PWM16_CH0_COUNTER_LSB_REG:             equ 0x20   ; DR0 Count register
PWM16_CH0_COUNTER_MSB_REG:             equ 0x24
PWM16_CH0_PERIOD_LSB_REG:              equ 0x21   ; DR1 Period register
PWM16_CH0_PERIOD_MSB_REG:              equ 0x25
PWM16_CH0_COMPARE_LSB_REG:             equ 0x22   ; DR2 Compare register
PWM16_CH0_COMPARE_MSB_REG:             equ 0x26
PWM16_CH0_CONTROL_LSB_REG:             equ 0x23   ; Control register
PWM16_CH0_CONTROL_MSB_REG:             equ 0x27
PWM16_CH0_FUNC_LSB_REG:                equ 0x20   ; Function register
PWM16_CH0_FUNC_MSB_REG:                equ 0x24
PWM16_CH0_INPUT_LSB_REG:               equ 0x21   ; Input register
PWM16_CH0_INPUT_MSB_REG:               equ 0x25
PWM16_CH0_OUTPUT_LSB_REG:              equ 0x22   ; Output register
PWM16_CH0_OUTPUT_MSB_REG:              equ 0x26


;--------------------------------------------------
; PWM16_CH0 Macro 'Functions'
;--------------------------------------------------

   macro PWM16_CH0_Start_M
   or    reg[PWM16_CH0_CONTROL_LSB_REG],  PWM16_CH0_CONTROL_REG_START_BIT
   endm

   macro PWM16_CH0_Stop_M
   and   reg[PWM16_CH0_CONTROL_LSB_REG], ~PWM16_CH0_CONTROL_REG_START_BIT
   endm

   macro PWM16_CH0_EnableInt_M
   M8C_EnableIntMask PWM16_CH0_INT_REG, PWM16_CH0_INT_MASK
   endm

   macro PWM16_CH0_DisableInt_M
   M8C_DisableIntMask PWM16_CH0_INT_REG, PWM16_CH0_INT_MASK
   endm


; end of file PWM16_CH0.inc

