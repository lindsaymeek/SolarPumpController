;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME: ADC.asm
;;   Version: 3.0, Updated on 2003/08/28 at 17:08:06
;;  Generated by PSoC Designer ver 4.0 b865 : 27 August, 2003
;;
;;  DESCRIPTION: Assembler source for the 8-bit Delta-Sigma A/D Converter
;;               User Module with 2nd-order modulator.
;;
;;  NOTE: User Module APIs conform to the fastcall convention for marshalling
;;        arguments and observe the associated "Registers are volatile" policy.
;;        This means it is the caller's responsibility to preserve any values
;;        in the X and A registers that are still needed after the API
;;        function returns. Even though these registers may be preserved now,
;;        there is no guarantee they will be preserved in future releases.
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress MicroSystems 2000-2003. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************

include "m8c.inc"
include "ADC.inc"


;-----------------------------------------------
;  Global Symbols
;-----------------------------------------------
export  ADC_Start
export _ADC_Start
export  ADC_SetPower
export _ADC_SetPower
export  ADC_Stop
export _ADC_Stop
export  ADC_StartAD
export _ADC_StartAD
export  ADC_StopAD
export _ADC_StopAD

IF (ADC_POLL_ENABLE)
export  ADC_fIsDataAvailable:
export _ADC_fIsDataAvailable:
export  ADC_cGetDataClearFlag:
export _ADC_cGetDataClearFlag:
export  ADC_cGetData:
export _ADC_cGetData:
export  ADC_ClearFlag:
export _ADC_ClearFlag:
ENDIF


AREA bss (RAM,REL)

;-----------------------------------------------
;  Constant Definitions
;-----------------------------------------------

CONTROL_REG_ENABLE_BIT:                    equ  01h     ; Control register start bit
POWERMASK:                                 equ  03h     ; Analog PSoC Block Power bits
TIMER_INT_MASK:                            equ  02h     ; Interrupt mask
NOAZ:                                      equ  1       ;
SC_AZ_MASK:                                equ  20h     ; Switched Cap Auto Zero bit
SC_FSW0_MASK:                              equ  10h     ; Switched Cap Feedback '0' switch


;-----------------------------------------------
; Variable Allocation
;-----------------------------------------------


AREA UserModules (ROM, REL)

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: ADC_Start
;  FUNCTION NAME: ADC_SetPower
;
;  DESCRIPTION: Applies power setting to the module's analog PSoc blocks.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    The A register contains the power setting.
;  RETURNS:      Nothing.
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 ADC_Start:
_ADC_Start:
 ADC_SetPower:
_ADC_SetPower:
   mov  X,SP                                     ; Set up Stack frame
   and  A,POWERMASK                              ; Ensure value is legal
   push A
   mov  A,reg[ADC_AtoD1cr3]                      ; First SC block:
   and  A,~POWERMASK                             ;   clear power bits to zero
   or   A,[ X ]                                  ;   establish new value
   mov  reg[ADC_AtoD1cr3],A                      ;   change the actual setting
   mov  A,reg[ADC_AtoD2cr3]                      ; Second SC block: as previous
   and  A,~POWERMASK
   or   A,[ X ]
   mov  reg[ADC_AtoD2cr3],A
   pop  A
   ret

.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: ADC_Stop
;
;  DESCRIPTION:   Removes power from the module's analog PSoc blocks.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:     None.
;  RETURNS:       Nothing.
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 ADC_Stop:
_ADC_Stop:
   and  reg[ADC_AtoD1cr3], ~POWERMASK
   and  reg[ADC_AtoD2cr3], ~POWERMASK
   ret


.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: ADC_StartAD
;
;  DESCRIPTION: Activates interrupts for this user module and begins sampling.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None.
;  RETURNS:      Nothing.
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 ADC_StartAD:
_ADC_StartAD:
   M8C_EnableIntMask INT_MSK1, TIMER_INT_MASK        ;Enable timer interrupt
   and  reg[ADC_AtoD1cr3], ~SC_FSW0_MASK             ;Enable the Integrator ...
   and  reg[ADC_AtoD2cr3], ~SC_FSW0_MASK             ;  (both blocks)
IF NOAZ
   and  reg[ADC_AtoD1cr2], ~SC_AZ_MASK               ;Turn offAutoZero
   and  reg[ADC_AtoD2cr2], ~SC_AZ_MASK               ;  (both blocks)
ENDIF
                                                     ;Initialize Timer ...
   mov  reg[ADC_TimerDR1], FFh                       ;  set period = 256
   mov  reg[ADC_TimerCR0], CONTROL_REG_ENABLE_BIT
   ret


.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: ADC_StopAD
;
;  DESCRIPTION: Shuts down the A/D is an orderly manner.  The Timer stops
;               operating and it's interrupt is disabled. Analog power is
;               still supplied to the analog blocks, however.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None.
;  RETURNS:      Nothing.
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 ADC_StopAD:
_ADC_StopAD:
   mov  reg[ADC_TimerCR0],00h                        ; disable the Timer
   M8C_DisableIntMask INT_MSK1, TIMER_INT_MASK       ; Disable its interrupt
IF NOAZ
   or   reg[ADC_AtoD1cr2],SC_AZ_MASK                 ; force the Integrator into reset
   or   reg[ADC_AtoD2cr2],SC_AZ_MASK                 ;
ENDIF
   or   reg[ADC_AtoD1cr3],SC_FSW0_MASK               ; reset Integrator
   or   reg[ADC_AtoD2cr3],SC_FSW0_MASK               ;
   ret

.ENDSECTION

IF (ADC_POLL_ENABLE)

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: ADC_fIsDataAvailable
;
;  DESCRIPTION: Returns the status of the A/D Data
;-----------------------------------------------------------------------------
;  ARGUMENTS:    None.
;  RETURNS:      fastcall BOOL DataAvailable returned in the A register
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 ADC_fIsDataAvailable:
_ADC_fIsDataAvailable:
   mov  A, [ADC_bfStatus]
   ret

.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME:  ADC_cGetDataClearFlag
;
;  DESCRIPTION:    Returns the data from the A/D.  Does not check if data is
;                  available. Also clears the DATA_READY flag.
;-----------------------------------------------------------------------------
;  ARGUMENTS:    None.
;  RETURNS:      fastcall CHAR cData returned in the A register
;  SIDE EFFECTS: The DATA_READY flag is cleared.
;                REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 ADC_cGetDataClearFlag:
_ADC_cGetDataClearFlag:
   and  [ADC_bfStatus], ~ADC_DATA_READY_BIT

   ;; Deliberately falls into cGetData, below:

;-----------------------------------------------------------------------------
;  FUNCTION NAME:  ADC_cGetData:
;
;  DESCRIPTION:     Returns the data from the A/D.  Does not check if data is
;                   available.
;-----------------------------------------------------------------------------
;  ARGUMENTS:    None.
;  RETURNS:      fastcall CHAR cData returned in the A register
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 ADC_cGetData:
_ADC_cGetData:
   mov  A, [ADC_cResult]
   ret


.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: ADC_ClearFlag
;
;  DESCRIPTION: Clears the data ready flag.
;-----------------------------------------------------------------------------
;  ARGUMENTS:    None.
;  RETURNS:      Nothing.
;  SIDE EFFECTS: The DATA_READY flag is cleared.
;                REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 ADC_ClearFlag:
_ADC_ClearFlag:
   and  [ADC_bfStatus], ~ADC_DATA_READY_BIT
   ret

.ENDSECTION

ENDIF


; End of File ADC.asm
