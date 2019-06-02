;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME: Count_I.asm
;;   Version: 2.1, Updated on 2003/08/26 at 16:04:39
;;  Generated by PSoC Designer ver 4.0 b865 : 27 August, 2003
;;
;;  DESCRIPTION: Counter8 User Module software implementation file
;;               for the 22/24/27/28xxx PSoC family of devices
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
include "Count_I.inc"

;-----------------------------------------------
;  Global Symbols
;-----------------------------------------------
export  Count_I_EnableInt
export _Count_I_EnableInt
export  Count_I_DisableInt
export _Count_I_DisableInt
export  Count_I_Start
export _Count_I_Start
export  Count_I_Stop
export _Count_I_Stop
export  Count_I_WritePeriod
export _Count_I_WritePeriod
export  Count_I_WriteCompareValue
export _Count_I_WriteCompareValue
export  Count_I_bReadCompareValue
export _Count_I_bReadCompareValue
export  Count_I_bReadCounter
export _Count_I_bReadCounter

; The following functions are deprecated and subject to omission in future releases
;
export  bCount_I_ReadCompareValue  ; deprecated
export _bCount_I_ReadCompareValue  ; deprecated
export  bCount_I_ReadCounter       ; deprecated
export _bCount_I_ReadCounter       ; deprecated


AREA bss (RAM,REL)

;-----------------------------------------------
;  Constant Definitions
;-----------------------------------------------

INPUT_REG_NULL:                equ 0x00    ; Clear the input register


;-----------------------------------------------
; Variable Allocation
;-----------------------------------------------


AREA UserModules (ROM, REL)

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: Count_I_EnableInt
;
;  DESCRIPTION:
;     Enables this counter's interrupt by setting the interrupt enable mask bit
;     associated with this User Module. This function has no effect until and
;     unless the global interrupts are enabled (for example by using the
;     macro M8C_EnableGInt).
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None.
;  RETURNS:      Nothing.
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 Count_I_EnableInt:
_Count_I_EnableInt:
   Count_I_EnableInt_M
   ret

.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: Count_I_DisableInt
;
;  DESCRIPTION:
;     Disables this counter's interrupt by clearing the interrupt enable
;     mask bit associated with this User Module.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      Nothing
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 Count_I_DisableInt:
_Count_I_DisableInt:
   Count_I_DisableInt_M
   ret


.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: Count_I_Start
;
;  DESCRIPTION:
;     Sets the start bit in the Control register of this user module.  The
;     counter will begin counting on the next input clock as soon as the
;     enable input is asserted high.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      Nothing
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 Count_I_Start:
_Count_I_Start:
   Count_I_Start_M
   ret


.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: Count_I_Stop
;
;  DESCRIPTION:
;     Disables counter operation by clearing the start bit in the Control
;     register.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      Nothing
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 Count_I_Stop:
_Count_I_Stop:
   Count_I_Stop_M
   ret


.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: Count_I_WritePeriod
;
;  DESCRIPTION:
;     Write the 8-bit period value into the Period register (DR1).
;-----------------------------------------------------------------------------
;
;  ARGUMENTS: fastcall BYTE bPeriodValue (passed in A)
;  RETURNS:   Nothing
;  SIDE EFFECTS:
;     If the counter user module is stopped, then this value will also be
;     latched into the Count register (DR0).
;     REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 Count_I_WritePeriod:
_Count_I_WritePeriod:
   mov   reg[Count_I_PERIOD_REG], A
   ret


.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: Count_I_WriteCompareValue
;
;  DESCRIPTION:
;     Writes compare value into the Compare register (DR2).
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    fastcall BYTE bCompareValue (passed in A)
;  RETURNS:      Nothing
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 Count_I_WriteCompareValue:
_Count_I_WriteCompareValue:
   mov   reg[Count_I_COMPARE_REG], A
   ret


.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: Count_I_bReadCompareValue
;
;  DESCRIPTION:
;     Reads the Compare register.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      fastcall BYTE bCompareValue (value of DR2 in the A register)
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 Count_I_bReadCompareValue:
_Count_I_bReadCompareValue:
 bCount_I_ReadCompareValue:                      ; this name deprecated
_bCount_I_ReadCompareValue:                      ; this name deprecated
   mov   A, reg[Count_I_COMPARE_REG]
   ret


.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: Count_I_bReadCounter
;
;  DESCRIPTION:
;     Returns the value in the Count register (DR0), preserving the value in
;     the compare register (DR2). Interrupts are prevented during the transfer
;     from the Count to the Compare registers by holding the clock low in
;     the PSoC block.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS: None
;  RETURNS:   fastcall BYTE bCount (value of DR0 in the A register)
;  SIDE EFFECTS:
;     1) If running, the user module is stopped momentarily and one or more
;        counts may be missed.
;     2) REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 Count_I_bReadCounter:
_Count_I_bReadCounter:
 bCount_I_ReadCounter:                           ; this name deprecated
_bCount_I_ReadCounter:                           ; this name deprecated

   bOrigCompareValue:      EQU   0               ; Frame offset to temp Compare store
   bOrigControlReg:        EQU   1               ; Frame offset to temp CR0     store
   bOrigClockSetting:      EQU   2               ; Frame offset to temp Input   store
   wCounter:               EQU   3               ; Frame offset to temp Count   store
   STACK_FRAME_SIZE:       EQU   4               ; max stack frame size is 4 bytes

   mov   X, SP                                   ; X <- stack frame pointer
   mov   A, reg[Count_I_COMPARE_REG]             ; Save the Compare register on the stack
   push  A                                       ;
   mov   A, reg[Count_I_CONTROL_REG]             ; Save CR0 (running or stopped state)
   push  A                                       ;
   Count_I_Stop_M                                ; Disable (stop) the Counter if running
   M8C_SetBank1                                  ;
   mov   A, reg[Count_I_INPUT_REG]               ; save the clock input setting
   push  A                                       ;   on the stack (now 2 bytes) and ...
                                                 ;   hold the clock low:
   mov   reg[Count_I_INPUT_REG], INPUT_REG_NULL
   M8C_SetBank0
                                                 ; Extract the Count via DR2 register
   mov   A, reg[Count_I_COUNTER_REG]             ; DR2 <- DR0
   mov   A, reg[Count_I_COMPARE_REG]             ; Stash the Count on the stack
   push  A                                       ;  -stack frame is now 3 bytes
   mov   A, [X+bOrigCompareValue]                ; Restore the Compare register
   mov   reg[Count_I_COMPARE_REG], A
   M8C_SetBank1                                  ; Restore the counter operation:
   mov   A, [X+bOrigClockSetting]                ;   First, the clock setting...
   mov   reg[Count_I_INPUT_REG], A               ;
   M8C_SetBank0                                  ;   then re-enable (start) the counter
   mov   A, [X+bOrigControlReg]                  ;     if it was running when
   mov   reg[Count_I_CONTROL_REG], A             ;     this function was first called
   pop   A                                       ; Setup the return value
   ADD   SP, -(STACK_FRAME_SIZE-1)               ; Zap remainder of stack frame
   ret

.ENDSECTION

; End of File Count_I.asm
