;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME: LPF2_I.asm
;;   Version: 2.0, Updated on 2003/08/28 at 13:31:00
;;  Generated by PSoC Designer ver 4.0 b865 : 27 August, 2003
;;
;;  DESCRIPTION: Asm source for 2 Pole Switched Capacitor Low Pass Filter
;;
;;  NOTE: User Module APIs conform to the fastcall convention for marshalling
;;        arguments and observe the associated "Registers are volatile" policy.
;;        This means it is the caller's responsibility to preserve any values
;;        in the X and A registers that are still needed after the API
;;        function returns. Even though these registers may be preserved now,
;;        there is no guarantee they will be preserved in future releases.
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress MicroSystems 2001-2003. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************

include "m8c.inc"
include "LPF2_I.inc"


;-----------------------------------------------
;  Global Symbols
;-----------------------------------------------
export  LPF2_I_Start
export _LPF2_I_Start
export  LPF2_I_SetPower
export _LPF2_I_SetPower

export  LPF2_I_SetC1
export _LPF2_I_SetC1
export  LPF2_I_SetC2
export _LPF2_I_SetC2
export  LPF2_I_SetC3
export _LPF2_I_SetC3
export  LPF2_I_SetC4
export _LPF2_I_SetC4
export  LPF2_I_SetCA
export _LPF2_I_SetCA
export  LPF2_I_SetCB
export _LPF2_I_SetCB

export  LPF2_I_Stop
export _LPF2_I_Stop


;-----------------------------------------------
; Variable Allocation
;-----------------------------------------------
AREA bss (RAM,REL)



;-----------------------------------------------
;  EQUATES
;-----------------------------------------------
bPOWERMASK:          equ   03h         ; Power field mask for CR3
LPF_CNUMMASK:        equ   1fh         ; Mask for A, B & C Caps
LPF_CFBMASK:         equ   80h         ; Mask for Feedback Caps



AREA UserModules (ROM,REL)
;-----------------------------------------------------------------------------
;  FUNCTION NAME: LPF2_I_Start
;
;  DESCRIPTION: Applies power setting to the module's PSoC blocks
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    A register contains the power setting (constant)
;  RETURNS:      Nothing
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;

 LPF2_I_Start:
_LPF2_I_Start:
 LPF2_I_SetPower:
_LPF2_I_SetPower:

    and A, bPOWERMASK                        ; mask A to protect unchanged bits
    mov X, SP                                ; define temp store location

    push A                                   ; put power value in temp store
    mov A, reg[LPF2_I_FLIN_CR3]    ; read power value
    and A, ~bPOWERMASK                       ; clear power bits in A
    or  A, [X]                               ; combine power value with balance of reg.
    mov reg[LPF2_I_FLIN_CR3], A    ; move complete value back to register

    mov A, reg[LPF2_I_FLOUT_CR3]   ; read power value
    and A, ~bPOWERMASK                       ; clear power bits in A
    or  A, [X]                               ; combine power value with balance of reg.
    mov reg[LPF2_I_FLOUT_CR3], A   ; move complete value back to register
    pop A
    ret

;-----------------------------------------------------------------------------
;  FUNCTION NAME: LPF2_I_SetCn, n=1..4
;
;  DESCRIPTION:   Alters the filter transfer function by modifying the value
;                 of the capacitors (defined in LPF2_I.inc)
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    The A register contains a non-negative integer less than 32
;  RETURNS:
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;

 LPF2_I_SetC1:
_LPF2_I_SetC1:

    and A, LPF_CNUMMASK                      ; mask A to protect unchanged bits
    mov X, SP                                ; define temp store location
    push A                                   ; put C1 value in temp store
    mov A, reg[LPF2_I_FILT_C1_REG] ; read current C1
    and A, ~LPF_CNUMMASK                     ; clear C1 bits in A
    or  A, [X]                               ; combine C1 value with balance of reg.
    mov reg[LPF2_I_FILT_C1_REG], A ; move complete value back to register
    pop  A
    ret

 LPF2_I_SetC2:
_LPF2_I_SetC2:

    and A, LPF_CNUMMASK                      ; mask A to protect unchanged bits
    mov X, SP                                ; define temp store location
    push A                                   ; put C2 value in temp store
    mov A, reg[LPF2_I_FILT_C2_REG] ; read current C2
    and A, ~LPF_CNUMMASK                     ; clear C2 bits in A
    or  A, [X]                               ; combine C2 value with balance of reg.
    mov reg[LPF2_I_FILT_C2_REG], A ; move complete value back to register
    pop A
    ret

 LPF2_I_SetC3:
_LPF2_I_SetC3:

    and A, LPF_CNUMMASK                      ; mask A to protect unchanged bits
    mov X, SP                                ; define temp store location
    push A                                   ; put C3 value in temp store
    mov A, reg[LPF2_I_FILT_C3_REG] ; read current C3
    and A, ~LPF_CNUMMASK                     ; clear C3 bits in A
    or  A, [X]                               ; combine C3 value with balance of reg.
    mov reg[LPF2_I_FILT_C3_REG], A ; move complete value back to register
    pop A
    ret

 LPF2_I_SetC4:
_LPF2_I_SetC4:

    and A, LPF_CNUMMASK                      ; mask A to protect unchanged bits
    mov X, SP                                ; define temp store location
    push A                                   ; put C4 value in temp store
    mov A, reg[LPF2_I_FILT_C4_REG] ; read current C4
    and A, ~LPF_CNUMMASK                     ; clear C4 bits in A
    or  A, [X]                               ; combine C4 value with balance of reg.
    mov reg[LPF2_I_FILT_C4_REG], A ; move complete value back to register
    pop A
    ret


;-----------------------------------------------------------------------------
;  FUNCTION NAME: LPF2_I_SetCA
;            and  LPF2_I_SetCB
;
;  DESCRIPTION:   Alters the filter transfer function by modifying the value
;                 of the op-amp feedback capacitors (see LPF2_I.inc)
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    The A register contains one of the LPF2_I_FEEDBACK
;                  constants defined in LPF2_I.inc
;  RETURNS:      Nothing.
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;

 LPF2_I_SetCA:
_LPF2_I_SetCA:
    cmp   A, LPF2_I_FEEDBACK_32    ; Change Feedback to 32 units?
    jz    .SetCAto32                         ;    Yes, go make it so
                                             ;     No, clear it to 16 units...
    and   reg[LPF2_I_FILT_CA_REG], ~LPF_CFBMASK
    ret
.SetCAto32:
    or    reg[LPF2_I_FILT_CA_REG],  LPF_CFBMASK
    ret


 LPF2_I_SetCB:
_LPF2_I_SetCB:
    cmp   A, LPF2_I_FEEDBACK_32    ; Change Feedback to 32 units?
    jz    .SetCAto32                         ;    Yes, go make it so
                                             ;     No, clear it to 16 units...
    and   reg[LPF2_I_FILT_CB_REG], ~LPF_CFBMASK
    ret
.SetCAto32:
    or    reg[LPF2_I_FILT_CB_REG],  LPF_CFBMASK
    ret


;-----------------------------------------------------------------------------
;  FUNCTION NAME: LPF2_I_Stop
;
;  DESCRIPTION: Cuts power to the user module.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:     None
;  RETURNS:       Nothing
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 LPF2_I_Stop:
_LPF2_I_Stop:


    and reg[LPF2_I_FLIN_CR3],  ~bPOWERMASK
    and reg[LPF2_I_FLOUT_CR3], ~bPOWERMASK
    ret


; End of File LPF2_I.asm