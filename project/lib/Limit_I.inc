;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME:   Limit_I.inc
;;  Version: 3.0, Updated on 2003/08/27 at 14:02:42
;;  Generated by PSoC Designer ver 4.0 b865 : 27 August, 2003
;;
;;  DESCRIPTION:  Assembler declarations for the CMPPRG user module interface
;;                for the 22/24/27xxx PSoC family of devices.
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress MicroSystems 2002-2003. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************


;--------------------------------------------------
; Constants for Limit_I API's
;--------------------------------------------------
Limit_I_OFF:        equ 0
Limit_I_LOWPOWER:   equ 1
Limit_I_MEDPOWER:   equ 2
Limit_I_HIGHPOWER:  equ 3

Limit_I_REF1_000:   equ f0h
Limit_I_REF0_937:   equ e0h 
Limit_I_REF0_875:   equ d0h
Limit_I_REF0_812:   equ c0h
Limit_I_REF0_750:   equ b0h
Limit_I_REF0_688:   equ a0h
Limit_I_REF0_625:   equ 90h
Limit_I_REF0_562:   equ 80h
Limit_I_REF0_500:   equ 70h
Limit_I_REF0_437:   equ 60h
Limit_I_REF0_375:   equ 50h
Limit_I_REF0_312:   equ 40h
Limit_I_REF0_250:   equ 30h
Limit_I_REF0_188:   equ 20h
Limit_I_REF0_125:   equ 10h
Limit_I_REF0_062:   equ 00h
Limit_I_REF0_042:   equ 14h
Limit_I_REF0_021:   equ 04h

;--------------------------------------------------
; Register Address Constants for Limit_I
;--------------------------------------------------
Limit_I_COMP_CR0:   equ 7dh
Limit_I_COMP_CR1:   equ 7eh
Limit_I_COMP_CR2:   equ 7fh
Limit_I_COMP_CR3:   equ 7ch

; end of file Limit_I.inc
