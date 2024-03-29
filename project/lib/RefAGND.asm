;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME:   RefAGND.asm
;;  Version: 1.1, Updated on 2003/08/19 at 20:48:24
;;  Generated by PSoC Designer ver 4.0 b865 : 27 August, 2003
;;
;;  DESCRIPTION: RefMux User Module software implementation file for the
;;               22/24/25/26/27xxx PSoc family of devices.
;;
;;  NOTE: User Module APIs conform to the fastcall convention for marshalling
;;        arguments and observe the associated "Registers are volatile" policy.
;;        This means it is the caller's responsibility to preserve any values
;;        in the X and A registers that are still needed after the API
;;        function returns. Even though these registers may be preserved now,
;;        there is no guarantee they will be preserved in future releases.
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress MicroSystems 2001 - 2003. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************

include "RefAGND.inc"
include "m8c.inc"

;-----------------------------------------------
;  Global Symbols
;-----------------------------------------------
export  RefAGND_Start
export _RefAGND_Start

export  RefAGND_SetPower
export _RefAGND_SetPower

export  RefAGND_Stop
export _RefAGND_Stop

export  RefAGND_RefSelect
export _RefAGND_RefSelect

;-----------------------------------------------
;  EQUATES
;-----------------------------------------------
TMUX_MASK:        equ 1Ch              ; Mask for Test Mux control
POWERMASK:        equ 03h

AREA UserModules (ROM, REL)
.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: RefAGND_Start
;  FUNCTION NAME: RefAGND_SetPower
;
;  DESCRIPTION:
;   Applies power setting to the module's PSoC block
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:
;  A  Contains the power setting; 0 = Off
;                                 1 = Low
;                                 2 = Med
;                                 3 = High
;
;  RETURNS:  NA
;
;  SIDE EFFECTS:
;    REGISTERS ARE VOLATILE:  THE A AND X REGISTERS MAY BE MODIFIED!
;
;  THEORY of OPERATION or PROCEDURE:
;
;-----------------------------------------------------------------------------

 RefAGND_Start:
_RefAGND_Start:
 RefAGND_SetPower:
_RefAGND_SetPower:

    and  A, POWERMASK                  ; mask A to protect unchanged bits
    mov  X, SP                         ; define temp store location

    push A                             ; put power value in temp store
    mov  A, reg[RefAGND_REFMUX_CR2]    ; read power value
    and  A, ~POWERMASK                 ; clear power bits in A
    or   A, [X]                        ; combine power value with balance of reg.
    mov  reg[RefAGND_REFMUX_CR2], A    ; move complete value back to register
    pop  A
    ret
.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: RefAGND_Stop
;
;  DESCRIPTION:
;   Turn off power to user module
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS: None
;
;  RETURNS:  NA
;
;  SIDE EFFECTS:
;    REGISTERS ARE VOLATILE:  THE A AND X REGISTERS MAY BE MODIFIED!
;
;  THEORY of OPERATION or PROCEDURE:
;
;-----------------------------------------------------------------------------
 RefAGND_Stop:
_RefAGND_Stop:

    and REG[RefAGND_REFMUX_CR2], ~POWERMASK
    ret
.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: RefAGND_InputSelect
;
;  DESCRIPTION:
;   Select one of the input of the TestMux to be connected to the Analog
;   Column bus.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:
;   A  contains value to select desired RefMux output.
;         Input              Code
;         Off                 00h
;         PMuxOut             10h
;         AGND                14h  (Power must be applied)
;         REFLO               18h
;         REFHI               1Ch
;
;  RETURNS:  NA
;
;  SIDE EFFECTS:
;    REGISTERS ARE VOLATILE:  THE A AND X REGISTERS MAY BE MODIFIED!
;
;  THEORY of OPERATION or PROCEDURE:
;
;-----------------------------------------------------------------------------
 RefAGND_RefSelect:
_RefAGND_RefSelect:
    and  A,TMUX_MASK                   ; Mask off test mux control
    mov  X,SP                          ; Get stack location
    push A
    mov  A,reg[RefAGND_REFMUX_CR2]     ; Write register Test Mux register to select reference
    and  A, ~TMUX_MASK                 ; Clear Mux control bits
    or   A,[X]                         ; Or in the new setting
    mov  reg[RefAGND_REFMUX_CR2],A     ; Write register Test Mux register to select reference
    pop  A                             ; Restore the stack
    ret
.ENDSECTION


; End of File RefAGND.asm
