;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME:   AMUX4_1.asm
;;  Version: 1.1, Updated on 2003/08/20 at 20:37:14
;;  Generated by PSoC Designer ver 4.0 b865 : 27 August, 2003
;;
;;  DESCRIPTION: AMux4 User Module software implementation file
;;               for 22/24/25/26/27xxx PSoC family devices.
;;
;;  NOTE: User Module APIs conform to the fastcall convention for marshalling
;;        arguments and observe the associated "Registers are volatile" policy.
;;        This means it is the caller's responsibility to preserve any values
;;        in the X and A registers that are still needed after the API 
;;        function returns. Even though these registers may be preserved now,
;;        there is no guarantee they will be preserved in future releases.
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress MicroSystems 2002-2003. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************


include "AMUX4_1.inc"
include "m8c.inc"

;-----------------------------------------------
;  Global Symbols
;-----------------------------------------------
export  AMUX4_1_InputSelect
export _AMUX4_1_InputSelect

export  AMUX4_1_Start
export _AMUX4_1_Start

export  AMUX4_1_Stop
export _AMUX4_1_Stop

;-----------------------------------------------
;  EQUATES
;-----------------------------------------------

MUXMASK:                     equ 03h

AREA UserModules (ROM, REL)
.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: AMUX4_1_InputSelect
;
;  DESCRIPTION:
;    Place the signal from one of four port0 pins on the Analog Column bus.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:
;    A contains the mux input control value
;   
;      Input           Code
;   Mux input 0,1       00h
;   Mux input 2,3       01h
;   Mux input 4,4       02h
;   Mux input 6,7       03h
;
;  RETURNS: none
;
;  SIDE EFFECTS:
;     REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
;-----------------------------------------------------------------------------
 AMUX4_1_InputSelect:
_AMUX4_1_InputSelect:
    and  A, MUXMASK                    ; Mask off only valid port0 input bits
    mov  X,AMUX4_1_MUX_COL
.AMUX4_ShiftIt:                         ; Shift value to correct bit position for Mux
    dec  X
    jc   .AMUX4_SetIt
    asl  A
    asl  A
    jmp  .AMUX4_ShiftIt                ; Loop until bits shifted to 2 LSBs

.AMUX4_SetIt:
    mov  X,SP                          ; Get current stack location
    push A                             ; Save a to OR it to AMX_IN later
    mov  A,reg[AMX_IN]                 ; Get current value of mux control register
    and  A,~AMUX4_1_MUX_MASK           ; Mask off the bits for this Mux
    or   A,[X]                         ; OR in the mux control bits
    mov  reg[AMX_IN],A                 ; Write out mux value to mux control register
    pop  A                             ; Restore the stack
    ret
.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: AMUX4_1_Start
;
;  DESCRIPTION:
;   This function does nothing at this time.  It is only here for
;   future compatibility.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS: none
;
;  RETURNS: none
;
;  SIDE EFFECTS:
;     REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;-----------------------------------------------------------------------------
 AMUX4_1_Start:
_AMUX4_1_Start:
    ret                             
.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: AMUX4_1_Stop
;
;  DESCRIPTION:
;   This function does nothing at this time.  It is only here for
;   future compatibility.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:  none
;
;  RETURNS: none
;
;  SIDE EFFECTS:
;     REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;-----------------------------------------------------------------------------
 AMUX4_1_Stop:
_AMUX4_1_Stop:
    ret                             
.ENDSECTION
; End of File AMUX4_1.asm
