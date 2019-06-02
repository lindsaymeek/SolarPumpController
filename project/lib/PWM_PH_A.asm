;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME: PWM_PH_A.asm
;;   Version: 2.2, Updated on 2003/08/26 at 16:06:22
;;  Generated by PSoC Designer ver 4.0 b865 : 27 August, 2003
;;
;;  DESCRIPTION: PWMDB8 User Module software implementation file
;;               for the 22/24/27xxx PSoC family of devices
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
include "PWM_PH_A.inc"


;-----------------------------------------------
;  Global Symbols
;-----------------------------------------------

export   PWM_PH_A_EnableInt
export  _PWM_PH_A_EnableInt
export   PWM_PH_A_DisableInt
export  _PWM_PH_A_DisableInt
export   PWM_PH_A_Start
export  _PWM_PH_A_Start
export   PWM_PH_A_Stop
export  _PWM_PH_A_Stop
export   PWM_PH_A_WritePeriod
export  _PWM_PH_A_WritePeriod
export   PWM_PH_A_WritePulseWidth  
export  _PWM_PH_A_WritePulseWidth  
export   PWM_PH_A_WriteDeadTime  
export  _PWM_PH_A_WriteDeadTime  
export   PWM_PH_A_bReadPulseWidth
export  _PWM_PH_A_bReadPulseWidth

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;       WARNING WARNING WARNING
; The following exports are for backwards
; compatibility only and should not be used
; for new designs.
export   bPWM_PH_A_ReadPulseWidth
export  _bPWM_PH_A_ReadPulseWidth
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;-----------------------------------------------
;  Constant Definitions
;-----------------------------------------------
bfCONTROL_REG_START_BIT:   equ   1  ; Control register start bit 

;-----------------------------------------------
; Variable Allocation
;-----------------------------------------------


area UserModules (ROM, REL)

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: PWM_PH_A_EnableInt
;
;  DESCRIPTION:
;     Enables this PWM's interrupt by setting the interrupt enable mask bit
;     associated with this User Module. Remember to call the global interrupt
;     enable function by using the macro: M8C_EnableGInt.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:  None
;
;  RETURNS: None
;
;  SIDE EFFECTS: 
;     REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
;  THEORY of OPERATION or PROCEDURE:
;     Sets the specific user module interrupt enable mask bit.
;
;-----------------------------------------------------------------------------
 PWM_PH_A_EnableInt:
_PWM_PH_A_EnableInt:
   M8C_EnableIntMask PWM_PH_A_INT_REG, PWM_PH_A_bINT_MASK
   ret	
.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: PWM_PH_A_DisableInt
;
;  DESCRIPTION:
;     Disables this PWM's interrupt by clearing the interrupt enable mask bit
;     associated with this User Module. 
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS: None
;
;  RETURNS: None
;
;  SIDE EFFECTS: 
;     REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
;  THEORY of OPERATION or PROCEDURE:
;     Clears the specific user module interrupt enable mask bit.
;
;-----------------------------------------------------------------------------
 PWM_PH_A_DisableInt:
_PWM_PH_A_DisableInt:
   M8C_DisableIntMask PWM_PH_A_INT_REG, PWM_PH_A_bINT_MASK
   ret
.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: PWM_PH_A_Start
;
;  DESCRIPTION:
;     Sets the start bit in the Control registers of this user module.  The
;     PWM will begin counting on the next input clock as soon as the 
;     enable input is asserted high.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS: None
;
;  RETURNS:  None
;
;  SIDE EFFECTS: 
;     REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
;  THEORY of OPERATION or PROCEDURE:
;     Set the start bit in the PWM and DB Control register.
;
;-----------------------------------------------------------------------------
 PWM_PH_A_Start:
_PWM_PH_A_Start:
   or    REG[PWM_PH_A_PWM_CONTROL_REG], bfCONTROL_REG_START_BIT
   or    REG[PWM_PH_A_DB_CONTROL_REG], bfCONTROL_REG_START_BIT
   ret	
.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: PWM_PH_A_Stop
;
;  DESCRIPTION:
;     Disables PWMDB8 operation.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS: None
;
;  RETURNS: None
;
;  SIDE EFFECTS: 
;     REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;     After this function completes:
;        1) The PWM Counter register will latch any data written to the Period 
;           register
;        2) The DB Counter register will latch any data written to the DeadTime 
;           Counter register.  
;
;  THEORY of OPERATION or PROCEDURE:
;     Clear the start bit in the Control registers.
;
;-----------------------------------------------------------------------------
 PWM_PH_A_Stop:
_PWM_PH_A_Stop:
   and   REG[PWM_PH_A_PWM_CONTROL_REG], ~bfCONTROL_REG_START_BIT
   and   REG[PWM_PH_A_DB_CONTROL_REG], ~bfCONTROL_REG_START_BIT
   ret	
.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: PWM_PH_A_WritePeriod
;
;  DESCRIPTION:
;     Write the period value into the Period register.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:
;     BYTE  bPeriodValue - period count - passed in the Accumulator.
;
;  RETURNS:  None
;
;  SIDE EFFECTS: 
;     REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;     If the PWMDB8 user module is stopped, then this value will also be
;     latched into the Counter register.
;
;  THEORY of OPERATION or PROCEDURE:
;     Write data into the Period register.
;
;-----------------------------------------------------------------------------
 PWM_PH_A_WritePeriod:
_PWM_PH_A_WritePeriod:
   mov   REG[PWM_PH_A_PWM_PERIOD_REG], A
   ret
.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: PWM_PH_A_WritePulseWidth
;
;  DESCRIPTION:
;     Writes compare value into the PulseWidth register.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:
;     BYTE  bPWidthValue - Pulse Width value count - passed in Accumulator.
;
;  RETURNS: None
;
;  SIDE EFFECTS: 
;     REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
;  THEORY of OPERATION or PROCEDURE:
;     Write data into the PulseWidth register.
;
;-----------------------------------------------------------------------------
 PWM_PH_A_WritePulseWidth:
_PWM_PH_A_WritePulseWidth:
   mov   REG[PWM_PH_A_PULSE_WIDTH_REG], A
   ret
.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: PWM_PH_A_WriteDeadTime
;
;  DESCRIPTION:
;     Writes Dead Time counter value into the DeadTimeCounter register.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:
;     BYTE  bDeadTime - dead time counter value count - passed in Accumulator.
;
;  RETURNS: None
;
;  SIDE EFFECTS: 
;     REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
;  THEORY of OPERATION or PROCEDURE:
;     Write data into the DeadTimeCounter register.
;
;-----------------------------------------------------------------------------
 PWM_PH_A_WriteDeadTime:
_PWM_PH_A_WriteDeadTime:
   mov   REG[PWM_PH_A_DEAD_TIME_REG], A
   ret
.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: PWM_PH_A_bReadPulseWidth
;
;  DESCRIPTION:
;     Reads the PulseWidth register.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS: None
;
;  RETURNS:
;     BYTE  bPulseWidth - value read from PulseWidth register - returned
;                           in the Accumulator.
;
;  SIDE EFFECTS: 
;     REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
;  THEORY of OPERATION or PROCEDURE:
;     Read the PulseWidth register and return value in A.
;
;-----------------------------------------------------------------------------
 PWM_PH_A_bReadPulseWidth:
_PWM_PH_A_bReadPulseWidth:
 bPWM_PH_A_ReadPulseWidth:  ; For backwards compatibility only.
_bPWM_PH_A_ReadPulseWidth:  ; For backwards compatibility only.
   mov   A, REG[PWM_PH_A_PULSE_WIDTH_REG]
   ret

.ENDSECTION

; End of File PWM_PH_A.asm
