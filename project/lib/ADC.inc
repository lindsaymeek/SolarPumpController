;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME: ADC.inc
;;   Version: 3.0, Updated on 2003/08/28 at 17:08:06
;;
;;  DESCRIPTION: Assembler declarations for the 8-bit Delta-Sigma A/D Converter
;;               User Module with a 2nd-order modulator.
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress MicroSystems 2000-2003. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************

include "m8c.inc"


;--------------------------------------------------
; Constants for ADC API's.
;--------------------------------------------------

ADC_OFF:                       equ 00h
ADC_LOWPOWER:                  equ 01h
ADC_MEDPOWER:                  equ 02h
ADC_HIGHPOWER:                 equ 03h

ADC_POLL_ENABLE:               equ 1
ADC_DATA_READY_BIT:            equ 10h


;--------------------------------------------------
; Register Address Constants for ADC
;--------------------------------------------------

ADC_TimerDR0:                  equ 0x24    ; Period reg
ADC_TimerDR1:                  equ 0x25    ; Down count reg
ADC_TimerDR2:                  equ 0x26    ; Compare reg
ADC_TimerCR0:                  equ 0x27    ; Control reg
ADC_TimerFN:                   equ 0x24    ; Function reg
ADC_TimerSL:                   equ 0x25    ; Input  select reg
ADC_TimerOS:                   equ 0x26    ; Output select reg

ADC_AtoD1cr0:                  equ 0x88    ; SC Block 1 Control Reg 0
ADC_AtoD1cr1:                  equ 0x89    ; SC Block 1 Control Reg 1
ADC_AtoD1cr2:                  equ 0x8a    ; SC Block 1 Control Reg 2
ADC_AtoD1cr3:                  equ 0x8b    ; SC Block 1 Control Reg 3
ADC_AtoD2cr0:                  equ 0x98    ; SC Block 2 Control Reg 0
ADC_AtoD2cr1:                  equ 0x99    ; SC Block 2 Control Reg 1
ADC_AtoD2cr2:                  equ 0x9a    ; SC Block 2 Control Reg 2
ADC_AtoD2cr3:                  equ 0x9b    ; SC Block 2 Control Reg 3


; end of file ADC.inc
