//*****************************************************************************
//*****************************************************************************
//  FILENAME: ADC.h
//   Version: 3.0, Updated on 2003/08/28 at 17:08:06
//
//  DESCRIPTION:  C declarations for the DELSIG8 User Module with
//                a 2nd-order modulator.
//-----------------------------------------------------------------------------
//      Copyright (c) Cypress MicroSystems YYYY. All Rights Reserved.
//*****************************************************************************
//*****************************************************************************

#include <m8c.h>

#define ADC_POLL_ENABLE 1

#pragma fastcall ADC_Start
#pragma fastcall ADC_SetPower
#pragma fastcall ADC_StartAD
#pragma fastcall ADC_StopAD
#pragma fastcall ADC_Stop

#if ( ADC_POLL_ENABLE )
#pragma fastcall ADC_fIsDataAvailable
#pragma fastcall ADC_cGetData
#pragma fastcall ADC_cGetDataClearFlag
#pragma fastcall ADC_ClearFlag
#endif

//-------------------------------------------------
// Prototypes of the ADC API.
//-------------------------------------------------
extern void ADC_Start(BYTE bPower);
extern void ADC_SetPower(BYTE bPower);
extern void ADC_StartAD(void);
extern void ADC_StopAD(void);
extern void ADC_Stop(void);

#if ( ADC_POLL_ENABLE )
extern BYTE ADC_fIsDataAvailable(void);
extern CHAR ADC_cGetData(void);
extern CHAR ADC_cGetDataClearFlag(void);
extern void ADC_ClearFlag(void);
#endif

//-------------------------------------------------
// Defines for ADC API's.
//-------------------------------------------------
#define ADC_OFF         0
#define ADC_LOWPOWER    1
#define ADC_MEDPOWER    2
#define ADC_HIGHPOWER   3


//-------------------------------------------------
// Hardware Register Definitions
//-------------------------------------------------
#pragma ioport  ADC_TimerDR0:   0x024              //Time base Counter register
BYTE            ADC_TimerDR0;
#pragma ioport  ADC_TimerDR1:   0x025              //Time base Period value register
BYTE            ADC_TimerDR1;
#pragma ioport  ADC_TimerDR2:   0x026              //Time base CompareValue register
BYTE            ADC_TimerDR2;
#pragma ioport  ADC_TimerCR0:   0x027              //Time base Control register
BYTE            ADC_TimerCR0;
#pragma ioport  ADC_TimerFN:    0x124               //Time base Function register
BYTE            ADC_TimerFN;
#pragma ioport  ADC_TimerSL:    0x125               //Time base Input register
BYTE            ADC_TimerSL;
#pragma ioport  ADC_TimerOS:    0x126               //Time base Output register
BYTE            ADC_TimerOS;

#pragma ioport  ADC_AtoD1cr0:   0x088              //Analog block 1 control register 0
BYTE            ADC_AtoD1cr0;
#pragma ioport  ADC_AtoD1cr1:   0x089              //Analog block 1 control register 1
BYTE            ADC_AtoD1cr1;
#pragma ioport  ADC_AtoD1cr2:   0x08a              //Analog block 1 control register 2
BYTE            ADC_AtoD1cr2;
#pragma ioport  ADC_AtoD1cr3:   0x08b              //Analog block 1 control register 3
BYTE            ADC_AtoD1cr3;
#pragma ioport  ADC_AtoD2cr0:   0x098              //Analog block 2 control register 0
BYTE            ADC_AtoD2cr0;
#pragma ioport  ADC_AtoD2cr1:   0x099              //Analog block 2 control register 1
BYTE            ADC_AtoD2cr1;
#pragma ioport  ADC_AtoD2cr2:   0x09a              //Analog block 2 control register 2
BYTE            ADC_AtoD2cr2;
#pragma ioport  ADC_AtoD2cr3:   0x09b              //Analog block 2 control register 3
BYTE            ADC_AtoD2cr3;


// end of file ADC.h
