;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME:   RefAGND.inc
;;  Version: 1.1, Updated on 2003/08/19 at 20:48:24
;;  Generated by PSoC Designer ver 4.0 b865 : 27 August, 2003
;;
;;  DESCRIPTION: Assembler declarations for the RefMux user module interface
;;               for the 25/26xxx PSoC family of devices.
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress MicroSystems 2001-2003. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************

include "m8c.inc"

;--------------------------------------------------
; Constants for RefAGND API's.
;--------------------------------------------------
; Power settings

RefAGND_OFF:                 equ 0
RefAGND_LOWPOWER:            equ 1
RefAGND_MEDPOWER:            equ 2
RefAGND_HIGHPOWER:           equ 3


RefAGND_MUXOFF:              equ 00h
RefAGND_PMUXOUT:             equ 10h
RefAGND_AGND:                equ 14h
RefAGND_REFLO:               equ 18h
RefAGND_REFHI:               equ 1Ch

;--------------------------------------------------
; Register Address Constants for RefAGND
;--------------------------------------------------
RefAGND_REFMUX_CR0: equ 79h
RefAGND_REFMUX_CR1: equ 7ah
RefAGND_REFMUX_CR2: equ 7bh
RefAGND_REFMUX_CR3: equ 78h

; end of file RefAGND.inc