;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME: PWM16_CH1.inc
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
; Constants for PWM16_CH1 API's.
;--------------------------------------------------

PWM16_CH1_CONTROL_REG_START_BIT:           equ 0x01    ; Control register start bit
PWM16_CH1_INT_REG:                         equ 0x0e1
PWM16_CH1_INT_MASK:                        equ 0x08


;--------------------------------------------------
; Constants for PWM16_CH1 user defined values
;--------------------------------------------------

PWM16_CH1_PERIOD:                          equ 0x4e20
PWM16_CH1_PULSE_WIDTH:                     equ 0x0


;--------------------------------------------------
; Register Address Constants for PWM16_CH1
;--------------------------------------------------

PWM16_CH1_COUNTER_LSB_REG:             equ 0x28   ; DR0 Count register
PWM16_CH1_COUNTER_MSB_REG:             equ 0x2c
PWM16_CH1_PERIOD_LSB_REG:              equ 0x29   ; DR1 Period register
PWM16_CH1_PERIOD_MSB_REG:              equ 0x2d
PWM16_CH1_COMPARE_LSB_REG:             equ 0x2a   ; DR2 Compare register
PWM16_CH1_COMPARE_MSB_REG:             equ 0x2e
PWM16_CH1_CONTROL_LSB_REG:             equ 0x2b   ; Control register
PWM16_CH1_CONTROL_MSB_REG:             equ 0x2f
PWM16_CH1_FUNC_LSB_REG:                equ 0x28   ; Function register
PWM16_CH1_FUNC_MSB_REG:                equ 0x2c
PWM16_CH1_INPUT_LSB_REG:               equ 0x29   ; Input register
PWM16_CH1_INPUT_MSB_REG:               equ 0x2d
PWM16_CH1_OUTPUT_LSB_REG:              equ 0x2a   ; Output register
PWM16_CH1_OUTPUT_MSB_REG:              equ 0x2e


;--------------------------------------------------
; PWM16_CH1 Macro 'Functions'
;--------------------------------------------------

   macro PWM16_CH1_Start_M
   or    reg[PWM16_CH1_CONTROL_LSB_REG],  PWM16_CH1_CONTROL_REG_START_BIT
   endm

   macro PWM16_CH1_Stop_M
   and   reg[PWM16_CH1_CONTROL_LSB_REG], ~PWM16_CH1_CONTROL_REG_START_BIT
   endm

   macro PWM16_CH1_EnableInt_M
   M8C_EnableIntMask PWM16_CH1_INT_REG, PWM16_CH1_INT_MASK
   endm

   macro PWM16_CH1_DisableInt_M
   M8C_DisableIntMask PWM16_CH1_INT_REG, PWM16_CH1_INT_MASK
   endm


; end of file PWM16_CH1.inc

