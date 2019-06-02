;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME: ADCINT.asm
;;   Version: 3.0, Updated on 2003/08/28 at 17:08:06
;;
;;  DESCRIPTION: Assembler interrupt service routine for the 8-bit Delta-Sigma
;;               A/D Converter User Module. This code works for both the
;;               first and second-order modulator topologies.
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress MicroSystems 2000-2003. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************

include "m8c.inc"
include "ADC.inc"


;-----------------------------------------------
;  Global Symbols
;-----------------------------------------------

export _ADC_ADConversion_ISR

IF (ADC_POLL_ENABLE)
export _ADC_cResult
export  ADC_cResult
export _ADC_bfStatus
export  ADC_bfStatus
ENDIF


;-----------------------------------------------
; Variable Allocation
;-----------------------------------------------
area bss(RAM,REL)

iOut:                                      BLK  2  ; Converted output value
iTmp2:                                     BLK  2  ; z^-2
iTmp1:                                     BLK  2  ; z^-1

IF (ADC_POLL_ENABLE)
_ADC_cResult:
 ADC_cResult:                              BLK  1  ; A/D value
_ADC_bfStatus:
 ADC_bfStatus:                             BLK  1  ; Data Valid Flag
ENDIF


;-----------------------------------------------
;  EQUATES
;-----------------------------------------------
HIGHBYTE:                   equ  0
LOWBYTE:                    equ  1


;@PSoC_UserCode_INIT@ (Do not change this line.)
;---------------------------------------------------
; Insert your custom declarations below this banner
;---------------------------------------------------

;---------------------------------------------------
; Insert your custom declarations above this banner
;---------------------------------------------------
;@PSoC_UserCode_END@ (Do not change this line.)


area UserModules (ROM, REL)

;-----------------------------------------------------------------------------
;  FUNCTION NAME: _ADC_ADConversion_ISR
;
;  DESCRIPTION: Perform final filter operations to produce output samples.
;
;-----------------------------------------------------------------------------
;
;  THEORY of OPERATION or PROCEDURE: In the Z-domain, the DelSig transfer
;     function is given by
;
;        H(Z) = [ (1 - Z^(-n)) / (1 - Z^(-1)) ]^2
;
;    The denominator is implemened by the hardware decimation unit operating
;    at the modulator (single-bit) sample rate. The following code, operating
;    at the decimation rate (n=64), completes the calculation:
;
;        (1 - Z^-n)^2  =  1 - 2z^-1 + Z^-2n  =  (1 - Z^-n) - (Z^-n - Z^-2n)
;    or
;        (Z^-2n - Z^-n) - (Z^-n - 1)  (inverting twice).
;
;    In time domain notation, for samples x[0],...,x[n],...,x[2n],...
;    (where x[0] is the most recent) this is becomes simply 
;
;        (x[2n]-x[n])-(x[n]-x[0]) or, for ease of notation, (x2-x1)-(x1-x0)
;
;    The decimation rate is established by the timer interrupt. Four timer
;    clocks elapse for each modulator output (decimator input) since the 
;    phi1/phi2 generator divides by 4. This means the timer period and thus
;    it's interrupt must equal 4 times the actual decimation rate, in this 
;    case, 4*64 = 256.
;
_ADC_ADConversion_ISR:
   push A                        ;  Variables:     Out              Tmp2     Tmp1   Deci
                                 ;  Initial state: (x3-x2)-(x2-x1)  (x2-x1)  x1     x0
                                                 ; --> Tmp2 moved to Out:
   mov  [iOut + LOWBYTE],   [iTmp2 + LOWBYTE]    ; Out              Tmp2     Tmp1   Deci
   mov  [iOut + HIGHBYTE],  [iTmp2 + HIGHBYTE]   ; (x2-x1)          (x2-x1)  x1     x0

                                                 ; --> Tmp1 moved to Tmp2:
   mov  [iTmp2 + LOWBYTE],  [iTmp1 + LOWBYTE]    ; Out              Tmp2     Tmp1   Deci
   mov  [iTmp2 + HIGHBYTE], [iTmp1 + HIGHBYTE]   ; (x2-x1)          x1       x1     x0

   mov  A, reg[DEC_DL]                           ; --> Deci to Tmp1 & ...
   mov  [iTmp1 + LOWBYTE],  A                    ;
   sub  [iTmp2 + LOWBYTE],  A                    ;   deci subtracted from Tmp2:
   mov  A, reg[DEC_DH]                           ;
   mov  [iTmp1 + HIGHBYTE], A                    ; Out              Tmp2     Tmp1   Deci
   sbb  [iTmp2 + HIGHBYTE], A                    ; (x2-x1)          x1-x0    x0     x0

   mov  A, [iTmp2 + LOWBYTE]                     ; --> Subtract Tmp2 from Out:
   sub  [iOut + LOWBYTE],   A                    ;
   mov  A, [iTmp2 + HIGHBYTE]                    ; Out              Tmp2     Tmp1   Deci
   sbb  [iOut + HIGHBYTE],  A                    ; (x2-x1)-(x1-x0)  x1-x0    x0     x0

   cmp  [iOut + HIGHBYTE], 10h                   ; Is the value less than full scale?
   jnz  LessThanFullScale                        ;   Yes, go normalize it.
   mov  A, 7fh                                   ;    No, limit value to plus full-scale
   jmp  ConversionReady                          ;         range (already normalized).
LessThanFullScale:
    mov  A, [iOut + HIGHBYTE]                    ; Normalize data (multiply by 8 and...
    rlc  [iOut + LOWBYTE]                        ;   use only what lies in the upper
    rlc  A                                       ;   byte)
    rlc  [iOut + LOWBYTE]
    rlc  A
    rlc  [iOut + LOWBYTE]
    rlc  A
ConversionReady:
	
   ;@PSoC_UserCode_BODY@ (Do not change this line.)
   ;---------------------------------------------------
   ; Insert your custom code below this banner
   ;---------------------------------------------------
   ; Sample data is now in the A register.
   ; Be sure to preserve the X register if you modify it!

IF (ADC_POLL_ENABLE)
   mov  [ADC_cResult],  A                                            ; Save result in cResult
   mov  [ADC_bfStatus], ADC_DATA_READY_BIT                           ; Set valid data flag
ENDIF

   ;---------------------------------------------------
   ; Insert your custom code above this banner
   ;---------------------------------------------------
   ;@PSoC_UserCode_END@ (Do not change this line.)

   pop A
   reti

; end of file ADCINT.asm
