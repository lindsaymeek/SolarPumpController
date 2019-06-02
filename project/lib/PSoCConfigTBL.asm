; Generated by PSoC Designer ver 4.0 b865 : 27 August, 2003
;
include "m8c.inc"
;  Personalization tables 
export LoadConfigTBL_pump_Bank1
export LoadConfigTBL_pump_Bank0
export LoadConfigTBL_pump_Ordered
export UnloadConfigTBL_pump_Bank1
export UnloadConfigTBL_pump_Bank0
export ReloadConfigTBL_pump_Bank1
export ReloadConfigTBL_pump_Bank0
export LoadConfigTBL_AGND_Bank1
export LoadConfigTBL_AGND_Bank0
export UnloadConfigTBL_AGND_Bank1
export UnloadConfigTBL_AGND_Bank0
export UnloadConfigTBL_Total_Bank1
export UnloadConfigTBL_Total_Bank0
AREA lit(rom, rel)
LoadConfigTBL_AGND_Bank0:
;  Instance name RefAGND, User Module RefMux
;       Instance name RefAGND, Block Name REFMUX(ACB02)
	db		79h, 01h		;RefAGND_REFMUX_CR0(ACB02CR0)
	db		7ah, 09h		;RefAGND_REFMUX_CR1(ACB02CR1)
	db		7bh, 14h		;RefAGND_REFMUX_CR2(ACB02CR2)
	db		78h, 00h		;RefAGND_REFMUX_CR3(ACB02CR3)
	db		ffh
LoadConfigTBL_AGND_Bank1:
;  Instance name RefAGND, User Module RefMux
;       Instance name RefAGND, Block Name REFMUX(ACB02)
	db		ffh
UnloadConfigTBL_AGND_Bank0:
;  Instance name RefAGND, User Module RefMux
;       Instance name RefAGND, Block Name REFMUX(ACB02)
	db		7bh, 00h		;RefAGND_CR2 (ACB02CR2)
	db		ffh
UnloadConfigTBL_AGND_Bank1:
;  Instance name RefAGND, User Module RefMux
;       Instance name RefAGND, Block Name REFMUX(ACB02)
	db		ffh

;  Instance name RefAGND, User Module RefMux
;       Instance name RefAGND, Block Name REFMUX(ACB02)
	db		ffh
LoadConfigTBL_pump_Ordered:
;  Ordered Global Register values
	M8C_SetBank1
	mov	reg[00h], 00h		; Port_0_DriveMode_0 register (PRT0DM0)
	mov	reg[01h], ffh		; Port_0_DriveMode_1 register (PRT0DM1)
	M8C_SetBank0
	mov	reg[03h], ffh		; Port_0_DriveMode_2 register (PRT0DM2)
	mov	reg[02h], 00h		; Port_0_GlobalSelect register (PRT0GS)
	M8C_SetBank1
	mov	reg[02h], 00h		; Port_0_IntCtrl_0 register (PRT0IC0)
	mov	reg[03h], 00h		; Port_0_IntCtrl_1 register (PRT0IC1)
	M8C_SetBank0
	mov	reg[01h], 00h		; Port_0_IntEn register (PRT0IE)
	M8C_SetBank1
	mov	reg[04h], cch		; Port_1_DriveMode_0 register (PRT1DM0)
	mov	reg[05h], 33h		; Port_1_DriveMode_1 register (PRT1DM1)
	M8C_SetBank0
	mov	reg[07h], 00h		; Port_1_DriveMode_2 register (PRT1DM2)
	mov	reg[06h], cch		; Port_1_GlobalSelect register (PRT1GS)
	M8C_SetBank1
	mov	reg[06h], 00h		; Port_1_IntCtrl_0 register (PRT1IC0)
	mov	reg[07h], 00h		; Port_1_IntCtrl_1 register (PRT1IC1)
	M8C_SetBank0
	mov	reg[05h], 00h		; Port_1_IntEn register (PRT1IE)
	M8C_SetBank1
	mov	reg[08h], fah		; Port_2_DriveMode_0 register (PRT2DM0)
	mov	reg[09h], 05h		; Port_2_DriveMode_1 register (PRT2DM1)
	M8C_SetBank0
	mov	reg[0bh], 04h		; Port_2_DriveMode_2 register (PRT2DM2)
	mov	reg[0ah], 30h		; Port_2_GlobalSelect register (PRT2GS)
	M8C_SetBank1
	mov	reg[0ah], 00h		; Port_2_IntCtrl_0 register (PRT2IC0)
	mov	reg[0bh], 00h		; Port_2_IntCtrl_1 register (PRT2IC1)
	M8C_SetBank0
	mov	reg[09h], 00h		; Port_2_IntEn register (PRT2IE)
	M8C_SetBank1
	mov	reg[0ch], 00h		; Port_3_DriveMode_0 register (PRT3DM0)
	mov	reg[0dh], 00h		; Port_3_DriveMode_1 register (PRT3DM1)
	M8C_SetBank0
	mov	reg[0fh], 00h		; Port_3_DriveMode_2 register (PRT3DM2)
	mov	reg[0eh], 00h		; Port_3_GlobalSelect register (PRT3GS)
	M8C_SetBank1
	mov	reg[0eh], 00h		; Port_3_IntCtrl_0 register (PRT3IC0)
	mov	reg[0fh], 00h		; Port_3_IntCtrl_1 register (PRT3IC1)
	M8C_SetBank0
	mov	reg[0dh], 00h		; Port_3_IntEn register (PRT3IE)
	M8C_SetBank1
	mov	reg[10h], 00h		; Port_4_DriveMode_0 register (PRT4DM0)
	mov	reg[11h], 00h		; Port_4_DriveMode_1 register (PRT4DM1)
	M8C_SetBank0
	mov	reg[13h], 00h		; Port_4_DriveMode_2 register (PRT4DM2)
	mov	reg[12h], 00h		; Port_4_GlobalSelect register (PRT4GS)
	M8C_SetBank1
	mov	reg[12h], 00h		; Port_4_IntCtrl_0 register (PRT4IC0)
	mov	reg[13h], 00h		; Port_4_IntCtrl_1 register (PRT4IC1)
	M8C_SetBank0
	mov	reg[11h], 00h		; Port_4_IntEn register (PRT4IE)
	M8C_SetBank1
	mov	reg[14h], 00h		; Port_5_DriveMode_0 register (PRT5DM0)
	mov	reg[15h], 00h		; Port_5_DriveMode_1 register (PRT5DM1)
	M8C_SetBank0
	mov	reg[17h], 00h		; Port_5_DriveMode_2 register (PRT5DM2)
	mov	reg[16h], 00h		; Port_5_GlobalSelect register (PRT5GS)
	M8C_SetBank1
	mov	reg[16h], 00h		; Port_5_IntCtrl_0 register (PRT5IC0)
	mov	reg[17h], 00h		; Port_5_IntCtrl_1 register (PRT5IC1)
	M8C_SetBank0
	mov	reg[15h], 00h		; Port_5_IntEn register (PRT5IE)
	ret
LoadConfigTBL_pump_Bank0:
;  Global Register values
	db		60h, e0h		; AnalogColumnInputSelect register (AMX_IN)
	db		66h, 00h		; AnalogComparatorControl1 register (CMP_CR1)
	db		63h, 37h		; AnalogReferenceControl register (ARF_CR)
	db		65h, 00h		; AnalogSyncControl register (ASY_CR)
	db		e6h, 04h		; DecimatorControl_0 register (DEC_CR0)
	db		e7h, 01h		; DecimatorControl_1 register (DEC_CR1)
	db		d6h, 00h		; I2CConfig register (I2CCFG)
	db		b0h, 00h		; Row_0_InputMux register (RDI0RI)
	db		b1h, 0fh		; Row_0_InputSync register (RDI0SYN)
	db		b2h, 00h		; Row_0_LogicInputAMux register (RDI0IS)
	db		b3h, 33h		; Row_0_LogicSelect_0 register (RDI0LT0)
	db		b4h, 33h		; Row_0_LogicSelect_1 register (RDI0LT1)
	db		b5h, 00h		; Row_0_OutputDrive_0 register (RDI0SRO0)
	db		b6h, 84h		; Row_0_OutputDrive_1 register (RDI0SRO1)
	db		b8h, 55h		; Row_1_InputMux register (RDI1RI)
	db		b9h, 0fh		; Row_1_InputSync register (RDI1SYN)
	db		bah, 10h		; Row_1_LogicInputAMux register (RDI1IS)
	db		bbh, 33h		; Row_1_LogicSelect_0 register (RDI1LT0)
	db		bch, 33h		; Row_1_LogicSelect_1 register (RDI1LT1)
	db		bdh, 22h		; Row_1_OutputDrive_0 register (RDI1SRO0)
	db		beh, 48h		; Row_1_OutputDrive_1 register (RDI1SRO1)
;  Instance name ADC, User Module DELSIG8
;       Instance name ADC, Block Name ADC1(ASC12)
	db		88h, c8h		;ADC_AtoD1cr0(ASC12CR0)
	db		89h, 00h		;ADC_AtoD1cr1(ASC12CR1)
	db		8ah, 20h		;ADC_AtoD1cr2(ASC12CR2)
	db		8bh, f0h		;ADC_AtoD1cr3(ASC12CR3)
;       Instance name ADC, Block Name ADC2(ASD22)
	db		98h, d0h		;ADC_AtoD2cr0(ASD22CR0)
	db		99h, 00h		;ADC_AtoD2cr1(ASD22CR1)
	db		9ah, 60h		;ADC_AtoD2cr2(ASD22CR2)
	db		9bh, f0h		;ADC_AtoD2cr3(ASD22CR3)
;       Instance name ADC, Block Name TMR(DBB01)
	db		27h, 00h		;ADC_TimerCR0(DBB01CR0)
	db		25h, 00h		;ADC_TimerDR1(DBB01DR1)
	db		26h, 00h		;ADC_TimerDR2(DBB01DR2)
;  Instance name AMUX4_1, User Module AMUX4
;  Instance name Amp_Vac, User Module INSAMP
;       Instance name Amp_Vac, Block Name INV(ACB01)
	db		75h, 7dh		;Amp_Vac_INV_CR0(ACB01CR0)
	db		76h, 21h		;Amp_Vac_INV_CR1(ACB01CR1)
	db		77h, 20h		;Amp_Vac_INV_CR2(ACB01CR2)
	db		74h, 00h		;Amp_Vac_INV_CR3(ACB01CR3)
;       Instance name Amp_Vac, Block Name NON_INV(ACB00)
	db		71h, 7ch		;Amp_Vac_NON_INV_CR0(ACB00CR0)
	db		72h, a1h		;Amp_Vac_NON_INV_CR1(ACB00CR1)
	db		73h, 20h		;Amp_Vac_NON_INV_CR2(ACB00CR2)
	db		70h, 00h		;Amp_Vac_NON_INV_CR3(ACB00CR3)
;  Instance name Count_I, User Module Counter8
;       Instance name Count_I, Block Name CNTR8(DBB00)
	db		23h, 00h		;Count_I_CONTROL_REG(DBB00CR0)
	db		21h, ffh		;Count_I_PERIOD_REG(DBB00DR1)
	db		22h, 00h		;Count_I_COMPARE_REG(DBB00DR2)
;  Instance name FlashTemp_1, User Module FlashTemp
;       Instance name FlashTemp_1, Block Name TEMPERATURE(ASC21)
	db		94h, 90h		;FlashTemp_1_FlashTempCR0(ASC21CR0)
	db		95h, 70h		;FlashTemp_1_FlashTempCR1(ASC21CR1)
	db		96h, 60h		;FlashTemp_1_FlashTempCR2(ASC21CR2)
	db		97h, fch		;FlashTemp_1_FlashTempCR3(ASC21CR3)
;  Instance name LPF2_I, User Module LPF2
;       Instance name LPF2_I, Block Name FLIN(ASC23)
	db		9ch, a1h		;LPF2_I_FLIN_CR0(ASC23CR0)
	db		9dh, e1h		;LPF2_I_FLIN_CR1(ASC23CR1)
	db		9eh, 00h		;LPF2_I_FLIN_CR2(ASC23CR2)
	db		9fh, 20h		;LPF2_I_FLIN_CR3(ASC23CR3)
;       Instance name LPF2_I, Block Name FLOUT(ASD13)
	db		8ch, 82h		;LPF2_I_FLOUT_CR0(ASD13CR0)
	db		8dh, 60h		;LPF2_I_FLOUT_CR1(ASD13CR1)
	db		8eh, 9fh		;LPF2_I_FLOUT_CR2(ASD13CR2)
	db		8fh, 20h		;LPF2_I_FLOUT_CR3(ASD13CR3)
;  Instance name Limit_I, User Module CMPPRG
;       Instance name Limit_I, Block Name COMP(ACB03)
	db		7dh, fah		;Limit_I_COMP_CR0(ACB03CR0)
	db		7eh, 61h		;Limit_I_COMP_CR1(ACB03CR1)
	db		7fh, 40h		;Limit_I_COMP_CR2(ACB03CR2)
	db		7ch, 00h		;Limit_I_COMP_CR3(ACB03CR3)
;  Instance name PGA_1, User Module PGA
;       Instance name PGA_1, Block Name GAIN(ACB02)
	db		79h, fdh		;PGA_1_GAIN_CR0(ACB02CR0)
	db		7ah, 21h		;PGA_1_GAIN_CR1(ACB02CR1)
	db		7bh, 20h		;PGA_1_GAIN_CR2(ACB02CR2)
	db		78h, 00h		;PGA_1_GAIN_CR3(ACB02CR3)
;  Instance name PWM_PH_A, User Module PWMDB8
;       Instance name PWM_PH_A, Block Name DB8(DCB13)
	db		3fh, 00h		;PWM_PH_A_DB_CONTROL_REG (DCB13CR0)
	db		3dh, 02h		;PWM_PH_A_DEAD_TIME_REG  (DCB13DR1)
	db		3eh, 00h		;PWM_PH_A_(DCB13DR2)
;       Instance name PWM_PH_A, Block Name PWM8(DCB12)
	db		3bh, 00h		;PWM_PH_A_PWM_CONTROL_REG(DCB12CR0)
	db		39h, ffh		;PWM_PH_A_PWM_PERIOD_REG (DCB12DR1)
	db		3ah, 00h		;PWM_PH_A_PULSE_WIDTH_REG(DCB12DR2)
;  Instance name PWM_PH_B, User Module PWMDB8
;       Instance name PWM_PH_B, Block Name DB8(DCB03)
	db		2fh, 00h		;PWM_PH_B_DB_CONTROL_REG (DCB03CR0)
	db		2dh, 02h		;PWM_PH_B_DEAD_TIME_REG  (DCB03DR1)
	db		2eh, 00h		;PWM_PH_B_(DCB03DR2)
;       Instance name PWM_PH_B, Block Name PWM8(DCB02)
	db		2bh, 00h		;PWM_PH_B_PWM_CONTROL_REG(DCB02CR0)
	db		29h, ffh		;PWM_PH_B_PWM_PERIOD_REG (DCB02DR1)
	db		2ah, 00h		;PWM_PH_B_PULSE_WIDTH_REG(DCB02DR2)
;  Instance name PWM_PH_C, User Module PWMDB8
;       Instance name PWM_PH_C, Block Name DB8(DBB11)
	db		37h, 00h		;PWM_PH_C_DB_CONTROL_REG (DBB11CR0)
	db		35h, 02h		;PWM_PH_C_DEAD_TIME_REG  (DBB11DR1)
	db		36h, 00h		;PWM_PH_C_(DBB11DR2)
;       Instance name PWM_PH_C, Block Name PWM8(DBB10)
	db		33h, 00h		;PWM_PH_C_PWM_CONTROL_REG(DBB10CR0)
	db		31h, ffh		;PWM_PH_C_PWM_PERIOD_REG (DBB10DR1)
	db		32h, 00h		;PWM_PH_C_PULSE_WIDTH_REG(DBB10DR2)
	db		ffh
LoadConfigTBL_pump_Bank1:
;  Global Register values
	db		61h, 00h		; AnalogClockSelect1 register (CLK_CR1)
	db		69h, 00h		; AnalogClockSelect2 register (CLK_CR2)
	db		60h, 04h		; AnalogColumnClockSelect register (CLK_CR0)
	db		62h, 1ch		; AnalogIOControl_0 register (ABF_CR0)
	db		67h, 33h		; AnalogLUTControl0 register (ALT_CR0)
	db		68h, 33h		; AnalogLUTControl1 register (ALT_CR1)
	db		63h, 00h		; AnalogModulatorControl_0 register (AMD_CR0)
	db		66h, 00h		; AnalogModulatorControl_1 register (AMD_CR1)
	db		d1h, 00h		; GlobalDigitalInterconnect_Drive_Even_Input register (GDI_E_IN)
	db		d3h, 00h		; GlobalDigitalInterconnect_Drive_Even_Output register (GDI_E_OU)
	db		d0h, 00h		; GlobalDigitalInterconnect_Drive_Odd_Input register (GDI_O_IN)
	db		d2h, 00h		; GlobalDigitalInterconnect_Drive_Odd_Output register (GDI_O_OU)
	db		e1h, 8fh		; OscillatorControl_1 register (OSC_CR1)
	db		e2h, 01h		; OscillatorControl_2 register (OSC_CR2)
	db		dfh, 0fh		; OscillatorControl_3 register (OSC_CR3)
	db		deh, 00h		; OscillatorControl_4 register (OSC_CR4)
	db		e3h, 84h		; VoltageMonitorControl register (VLT_CR)
;  Instance name ADC, User Module DELSIG8
;       Instance name ADC, Block Name ADC1(ASC12)
;       Instance name ADC, Block Name ADC2(ASD22)
;       Instance name ADC, Block Name TMR(DBB01)
	db		24h, 20h		;ADC_TimerFN(DBB01FN)
	db		25h, 15h		;ADC_TimerSL(DBB01IN)
	db		26h, 00h		;ADC_TimerOS(DBB01OU)
;  Instance name AMUX4_1, User Module AMUX4
;  Instance name Amp_Vac, User Module INSAMP
;       Instance name Amp_Vac, Block Name INV(ACB01)
;       Instance name Amp_Vac, Block Name NON_INV(ACB00)
;  Instance name Count_I, User Module Counter8
;       Instance name Count_I, Block Name CNTR8(DBB00)
	db		20h, 21h		;Count_I_FUNC_REG(DBB00FN)
	db		21h, 75h		;Count_I_INPUT_REG(DBB00IN)
	db		22h, 40h		;Count_I_OUTPUT_REG(DBB00OU)
;  Instance name FlashTemp_1, User Module FlashTemp
;       Instance name FlashTemp_1, Block Name TEMPERATURE(ASC21)
;  Instance name LPF2_I, User Module LPF2
;       Instance name LPF2_I, Block Name FLIN(ASC23)
;       Instance name LPF2_I, Block Name FLOUT(ASD13)
;  Instance name Limit_I, User Module CMPPRG
;       Instance name Limit_I, Block Name COMP(ACB03)
;  Instance name PGA_1, User Module PGA
;       Instance name PGA_1, Block Name GAIN(ACB02)
;  Instance name PWM_PH_A, User Module PWMDB8
;       Instance name PWM_PH_A, Block Name DB8(DCB13)
	db		3ch, 24h		;PWM_PH_A_DB_FUNC_REG    (DCB13FN)
	db		3dh, 01h		;PWM_PH_A_DB_INPUT_REG   (DCB13IN)
	db		3eh, 6eh		;PWM_PH_A_DB_OUTPUT_REG  (DCB13OU)
;       Instance name PWM_PH_A, Block Name PWM8(DCB12)
	db		38h, 21h		;PWM_PH_A_PWM_FUNC_REG   (DCB12FN)
	db		39h, 11h		;PWM_PH_A_PWM_INPUT_REG  (DCB12IN)
	db		3ah, 40h		;PWM_PH_A_PWM_OUTPUT_REG (DCB12OU)
;  Instance name PWM_PH_B, User Module PWMDB8
;       Instance name PWM_PH_B, Block Name DB8(DCB03)
	db		2ch, 24h		;PWM_PH_B_DB_FUNC_REG    (DCB03FN)
	db		2dh, 01h		;PWM_PH_B_DB_INPUT_REG   (DCB03IN)
	db		2eh, 7eh		;PWM_PH_B_DB_OUTPUT_REG  (DCB03OU)
;       Instance name PWM_PH_B, Block Name PWM8(DCB02)
	db		28h, 21h		;PWM_PH_B_PWM_FUNC_REG   (DCB02FN)
	db		29h, 11h		;PWM_PH_B_PWM_INPUT_REG  (DCB02IN)
	db		2ah, 40h		;PWM_PH_B_PWM_OUTPUT_REG (DCB02OU)
;  Instance name PWM_PH_C, User Module PWMDB8
;       Instance name PWM_PH_C, Block Name DB8(DBB11)
	db		34h, 24h		;PWM_PH_C_DB_FUNC_REG    (DBB11FN)
	db		35h, 01h		;PWM_PH_C_DB_INPUT_REG   (DBB11IN)
	db		36h, 7ch		;PWM_PH_C_DB_OUTPUT_REG  (DBB11OU)
;       Instance name PWM_PH_C, Block Name PWM8(DBB10)
	db		30h, 21h		;PWM_PH_C_PWM_FUNC_REG   (DBB10FN)
	db		31h, 11h		;PWM_PH_C_PWM_INPUT_REG  (DBB10IN)
	db		32h, 40h		;PWM_PH_C_PWM_OUTPUT_REG (DBB10OU)
	db		ffh
ReloadConfigTBL_pump_Bank0:
;  Instance name ADC, User Module DELSIG8
;       Instance name ADC, Block Name ADC1(ASC12)
	db		88h, c8h		;ADC_AtoD1cr0(ASC12CR0)
	db		89h, 00h		;ADC_AtoD1cr1(ASC12CR1)
	db		8ah, 20h		;ADC_AtoD1cr2(ASC12CR2)
	db		8bh, f0h		;ADC_AtoD1cr3(ASC12CR3)
;       Instance name ADC, Block Name ADC2(ASD22)
	db		98h, d0h		;ADC_AtoD2cr0(ASD22CR0)
	db		99h, 00h		;ADC_AtoD2cr1(ASD22CR1)
	db		9ah, 60h		;ADC_AtoD2cr2(ASD22CR2)
	db		9bh, f0h		;ADC_AtoD2cr3(ASD22CR3)
;       Instance name ADC, Block Name TMR(DBB01)
	db		27h, 00h		;ADC_TimerCR0(DBB01CR0)
	db		25h, 00h		;ADC_TimerDR1(DBB01DR1)
	db		26h, 00h		;ADC_TimerDR2(DBB01DR2)
;  Instance name AMUX4_1, User Module AMUX4
;  Instance name Amp_Vac, User Module INSAMP
;       Instance name Amp_Vac, Block Name INV(ACB01)
	db		75h, 7dh		;Amp_Vac_INV_CR0(ACB01CR0)
	db		76h, 21h		;Amp_Vac_INV_CR1(ACB01CR1)
	db		77h, 20h		;Amp_Vac_INV_CR2(ACB01CR2)
	db		74h, 00h		;Amp_Vac_INV_CR3(ACB01CR3)
;       Instance name Amp_Vac, Block Name NON_INV(ACB00)
	db		71h, 7ch		;Amp_Vac_NON_INV_CR0(ACB00CR0)
	db		72h, a1h		;Amp_Vac_NON_INV_CR1(ACB00CR1)
	db		73h, 20h		;Amp_Vac_NON_INV_CR2(ACB00CR2)
	db		70h, 00h		;Amp_Vac_NON_INV_CR3(ACB00CR3)
;  Instance name Count_I, User Module Counter8
;       Instance name Count_I, Block Name CNTR8(DBB00)
	db		23h, 00h		;Count_I_CONTROL_REG(DBB00CR0)
	db		21h, ffh		;Count_I_PERIOD_REG(DBB00DR1)
	db		22h, 00h		;Count_I_COMPARE_REG(DBB00DR2)
;  Instance name FlashTemp_1, User Module FlashTemp
;       Instance name FlashTemp_1, Block Name TEMPERATURE(ASC21)
	db		94h, 90h		;FlashTemp_1_FlashTempCR0(ASC21CR0)
	db		95h, 70h		;FlashTemp_1_FlashTempCR1(ASC21CR1)
	db		96h, 60h		;FlashTemp_1_FlashTempCR2(ASC21CR2)
	db		97h, fch		;FlashTemp_1_FlashTempCR3(ASC21CR3)
;  Instance name LPF2_I, User Module LPF2
;       Instance name LPF2_I, Block Name FLIN(ASC23)
	db		9ch, a1h		;LPF2_I_FLIN_CR0(ASC23CR0)
	db		9dh, e1h		;LPF2_I_FLIN_CR1(ASC23CR1)
	db		9eh, 00h		;LPF2_I_FLIN_CR2(ASC23CR2)
	db		9fh, 20h		;LPF2_I_FLIN_CR3(ASC23CR3)
;       Instance name LPF2_I, Block Name FLOUT(ASD13)
	db		8ch, 82h		;LPF2_I_FLOUT_CR0(ASD13CR0)
	db		8dh, 60h		;LPF2_I_FLOUT_CR1(ASD13CR1)
	db		8eh, 9fh		;LPF2_I_FLOUT_CR2(ASD13CR2)
	db		8fh, 20h		;LPF2_I_FLOUT_CR3(ASD13CR3)
;  Instance name Limit_I, User Module CMPPRG
;       Instance name Limit_I, Block Name COMP(ACB03)
	db		7dh, fah		;Limit_I_COMP_CR0(ACB03CR0)
	db		7eh, 61h		;Limit_I_COMP_CR1(ACB03CR1)
	db		7fh, 40h		;Limit_I_COMP_CR2(ACB03CR2)
	db		7ch, 00h		;Limit_I_COMP_CR3(ACB03CR3)
;  Instance name PGA_1, User Module PGA
;       Instance name PGA_1, Block Name GAIN(ACB02)
	db		79h, fdh		;PGA_1_GAIN_CR0(ACB02CR0)
	db		7ah, 21h		;PGA_1_GAIN_CR1(ACB02CR1)
	db		7bh, 20h		;PGA_1_GAIN_CR2(ACB02CR2)
	db		78h, 00h		;PGA_1_GAIN_CR3(ACB02CR3)
;  Instance name PWM_PH_A, User Module PWMDB8
;       Instance name PWM_PH_A, Block Name DB8(DCB13)
	db		3fh, 00h		;PWM_PH_A_DB_CONTROL_REG (DCB13CR0)
	db		3dh, 02h		;PWM_PH_A_DEAD_TIME_REG  (DCB13DR1)
	db		3eh, 00h		;PWM_PH_A_(DCB13DR2)
;       Instance name PWM_PH_A, Block Name PWM8(DCB12)
	db		3bh, 00h		;PWM_PH_A_PWM_CONTROL_REG(DCB12CR0)
	db		39h, ffh		;PWM_PH_A_PWM_PERIOD_REG (DCB12DR1)
	db		3ah, 00h		;PWM_PH_A_PULSE_WIDTH_REG(DCB12DR2)
;  Instance name PWM_PH_B, User Module PWMDB8
;       Instance name PWM_PH_B, Block Name DB8(DCB03)
	db		2fh, 00h		;PWM_PH_B_DB_CONTROL_REG (DCB03CR0)
	db		2dh, 02h		;PWM_PH_B_DEAD_TIME_REG  (DCB03DR1)
	db		2eh, 00h		;PWM_PH_B_(DCB03DR2)
;       Instance name PWM_PH_B, Block Name PWM8(DCB02)
	db		2bh, 00h		;PWM_PH_B_PWM_CONTROL_REG(DCB02CR0)
	db		29h, ffh		;PWM_PH_B_PWM_PERIOD_REG (DCB02DR1)
	db		2ah, 00h		;PWM_PH_B_PULSE_WIDTH_REG(DCB02DR2)
;  Instance name PWM_PH_C, User Module PWMDB8
;       Instance name PWM_PH_C, Block Name DB8(DBB11)
	db		37h, 00h		;PWM_PH_C_DB_CONTROL_REG (DBB11CR0)
	db		35h, 02h		;PWM_PH_C_DEAD_TIME_REG  (DBB11DR1)
	db		36h, 00h		;PWM_PH_C_(DBB11DR2)
;       Instance name PWM_PH_C, Block Name PWM8(DBB10)
	db		33h, 00h		;PWM_PH_C_PWM_CONTROL_REG(DBB10CR0)
	db		31h, ffh		;PWM_PH_C_PWM_PERIOD_REG (DBB10DR1)
	db		32h, 00h		;PWM_PH_C_PULSE_WIDTH_REG(DBB10DR2)
	db		ffh
ReloadConfigTBL_pump_Bank1:
;  Instance name ADC, User Module DELSIG8
;       Instance name ADC, Block Name ADC1(ASC12)
;       Instance name ADC, Block Name ADC2(ASD22)
;       Instance name ADC, Block Name TMR(DBB01)
	db		24h, 20h		;ADC_TimerFN(DBB01FN)
	db		25h, 15h		;ADC_TimerSL(DBB01IN)
	db		26h, 00h		;ADC_TimerOS(DBB01OU)
;  Instance name AMUX4_1, User Module AMUX4
;  Instance name Amp_Vac, User Module INSAMP
;       Instance name Amp_Vac, Block Name INV(ACB01)
;       Instance name Amp_Vac, Block Name NON_INV(ACB00)
;  Instance name Count_I, User Module Counter8
;       Instance name Count_I, Block Name CNTR8(DBB00)
	db		20h, 21h		;Count_I_FUNC_REG(DBB00FN)
	db		21h, 75h		;Count_I_INPUT_REG(DBB00IN)
	db		22h, 40h		;Count_I_OUTPUT_REG(DBB00OU)
;  Instance name FlashTemp_1, User Module FlashTemp
;       Instance name FlashTemp_1, Block Name TEMPERATURE(ASC21)
;  Instance name LPF2_I, User Module LPF2
;       Instance name LPF2_I, Block Name FLIN(ASC23)
;       Instance name LPF2_I, Block Name FLOUT(ASD13)
;  Instance name Limit_I, User Module CMPPRG
;       Instance name Limit_I, Block Name COMP(ACB03)
;  Instance name PGA_1, User Module PGA
;       Instance name PGA_1, Block Name GAIN(ACB02)
;  Instance name PWM_PH_A, User Module PWMDB8
;       Instance name PWM_PH_A, Block Name DB8(DCB13)
	db		3ch, 24h		;PWM_PH_A_DB_FUNC_REG    (DCB13FN)
	db		3dh, 01h		;PWM_PH_A_DB_INPUT_REG   (DCB13IN)
	db		3eh, 6eh		;PWM_PH_A_DB_OUTPUT_REG  (DCB13OU)
;       Instance name PWM_PH_A, Block Name PWM8(DCB12)
	db		38h, 21h		;PWM_PH_A_PWM_FUNC_REG   (DCB12FN)
	db		39h, 11h		;PWM_PH_A_PWM_INPUT_REG  (DCB12IN)
	db		3ah, 40h		;PWM_PH_A_PWM_OUTPUT_REG (DCB12OU)
;  Instance name PWM_PH_B, User Module PWMDB8
;       Instance name PWM_PH_B, Block Name DB8(DCB03)
	db		2ch, 24h		;PWM_PH_B_DB_FUNC_REG    (DCB03FN)
	db		2dh, 01h		;PWM_PH_B_DB_INPUT_REG   (DCB03IN)
	db		2eh, 7eh		;PWM_PH_B_DB_OUTPUT_REG  (DCB03OU)
;       Instance name PWM_PH_B, Block Name PWM8(DCB02)
	db		28h, 21h		;PWM_PH_B_PWM_FUNC_REG   (DCB02FN)
	db		29h, 11h		;PWM_PH_B_PWM_INPUT_REG  (DCB02IN)
	db		2ah, 40h		;PWM_PH_B_PWM_OUTPUT_REG (DCB02OU)
;  Instance name PWM_PH_C, User Module PWMDB8
;       Instance name PWM_PH_C, Block Name DB8(DBB11)
	db		34h, 24h		;PWM_PH_C_DB_FUNC_REG    (DBB11FN)
	db		35h, 01h		;PWM_PH_C_DB_INPUT_REG   (DBB11IN)
	db		36h, 7ch		;PWM_PH_C_DB_OUTPUT_REG  (DBB11OU)
;       Instance name PWM_PH_C, Block Name PWM8(DBB10)
	db		30h, 21h		;PWM_PH_C_PWM_FUNC_REG   (DBB10FN)
	db		31h, 11h		;PWM_PH_C_PWM_INPUT_REG  (DBB10IN)
	db		32h, 40h		;PWM_PH_C_PWM_OUTPUT_REG (DBB10OU)
	db		ffh
UnloadConfigTBL_pump_Bank0:
;  Instance name ADC, User Module DELSIG8
;       Instance name ADC, Block Name ADC1(ASC12)
	db		8bh, 00h		;ADC_CR3 (ASC12CR3)
;       Instance name ADC, Block Name ADC2(ASD22)
	db		9bh, 00h		;ADC_CR3 (ASD22CR3)
;       Instance name ADC, Block Name TMR(DBB01)
	db		27h, 00h		;ADC_CONTROL_0 (DBB01CR0)
;  Instance name AMUX4_1, User Module AMUX4
;  Instance name Amp_Vac, User Module INSAMP
;       Instance name Amp_Vac, Block Name INV(ACB01)
	db		77h, 00h		;Amp_Vac_CR2 (ACB01CR2)
;       Instance name Amp_Vac, Block Name NON_INV(ACB00)
	db		73h, 00h		;Amp_Vac_CR2 (ACB00CR2)
;  Instance name Count_I, User Module Counter8
;       Instance name Count_I, Block Name CNTR8(DBB00)
	db		23h, 00h		;Count_I_CONTROL_0 (DBB00CR0)
;  Instance name FlashTemp_1, User Module FlashTemp
;       Instance name FlashTemp_1, Block Name TEMPERATURE(ASC21)
	db		97h, 00h		;FlashTemp_1_CR3 (ASC21CR3)
;  Instance name LPF2_I, User Module LPF2
;       Instance name LPF2_I, Block Name FLIN(ASC23)
	db		9fh, 00h		;LPF2_I_CR3 (ASC23CR3)
;       Instance name LPF2_I, Block Name FLOUT(ASD13)
	db		8fh, 00h		;LPF2_I_CR3 (ASD13CR3)
;  Instance name Limit_I, User Module CMPPRG
;       Instance name Limit_I, Block Name COMP(ACB03)
	db		7fh, 00h		;Limit_I_CR2 (ACB03CR2)
;  Instance name PGA_1, User Module PGA
;       Instance name PGA_1, Block Name GAIN(ACB02)
	db		7bh, 00h		;PGA_1_CR2 (ACB02CR2)
;  Instance name PWM_PH_A, User Module PWMDB8
;       Instance name PWM_PH_A, Block Name DB8(DCB13)
	db		3fh, 00h		;PWM_PH_A_CONTROL_0 (DCB13CR0)
;       Instance name PWM_PH_A, Block Name PWM8(DCB12)
	db		3bh, 00h		;PWM_PH_A_CONTROL_0 (DCB12CR0)
;  Instance name PWM_PH_B, User Module PWMDB8
;       Instance name PWM_PH_B, Block Name DB8(DCB03)
	db		2fh, 00h		;PWM_PH_B_CONTROL_0 (DCB03CR0)
;       Instance name PWM_PH_B, Block Name PWM8(DCB02)
	db		2bh, 00h		;PWM_PH_B_CONTROL_0 (DCB02CR0)
;  Instance name PWM_PH_C, User Module PWMDB8
;       Instance name PWM_PH_C, Block Name DB8(DBB11)
	db		37h, 00h		;PWM_PH_C_CONTROL_0 (DBB11CR0)
;       Instance name PWM_PH_C, Block Name PWM8(DBB10)
	db		33h, 00h		;PWM_PH_C_CONTROL_0 (DBB10CR0)
	db		ffh
UnloadConfigTBL_pump_Bank1:
;  Instance name ADC, User Module DELSIG8
;       Instance name ADC, Block Name ADC1(ASC12)
;       Instance name ADC, Block Name ADC2(ASD22)
;       Instance name ADC, Block Name TMR(DBB01)
	db		24h, 00h		;ADC_DIG_BasicFunction (DBB01FN)
	db		25h, 00h		;ADC_DIG_Input (DBB01IN)
	db		26h, 00h		;ADC_DIG_Output (DBB01OU)
;  Instance name AMUX4_1, User Module AMUX4
;  Instance name Amp_Vac, User Module INSAMP
;       Instance name Amp_Vac, Block Name INV(ACB01)
;       Instance name Amp_Vac, Block Name NON_INV(ACB00)
;  Instance name Count_I, User Module Counter8
;       Instance name Count_I, Block Name CNTR8(DBB00)
	db		20h, 00h		;Count_I_DIG_BasicFunction (DBB00FN)
	db		21h, 00h		;Count_I_DIG_Input (DBB00IN)
	db		22h, 00h		;Count_I_DIG_Output (DBB00OU)
;  Instance name FlashTemp_1, User Module FlashTemp
;       Instance name FlashTemp_1, Block Name TEMPERATURE(ASC21)
;  Instance name LPF2_I, User Module LPF2
;       Instance name LPF2_I, Block Name FLIN(ASC23)
;       Instance name LPF2_I, Block Name FLOUT(ASD13)
;  Instance name Limit_I, User Module CMPPRG
;       Instance name Limit_I, Block Name COMP(ACB03)
;  Instance name PGA_1, User Module PGA
;       Instance name PGA_1, Block Name GAIN(ACB02)
;  Instance name PWM_PH_A, User Module PWMDB8
;       Instance name PWM_PH_A, Block Name DB8(DCB13)
	db		3ch, 00h		;PWM_PH_A_DIG_BasicFunction (DCB13FN)
	db		3dh, 00h		;PWM_PH_A_DIG_Input (DCB13IN)
	db		3eh, 00h		;PWM_PH_A_DIG_Output (DCB13OU)
;       Instance name PWM_PH_A, Block Name PWM8(DCB12)
	db		38h, 00h		;PWM_PH_A_DIG_BasicFunction (DCB12FN)
	db		39h, 00h		;PWM_PH_A_DIG_Input (DCB12IN)
	db		3ah, 00h		;PWM_PH_A_DIG_Output (DCB12OU)
;  Instance name PWM_PH_B, User Module PWMDB8
;       Instance name PWM_PH_B, Block Name DB8(DCB03)
	db		2ch, 00h		;PWM_PH_B_DIG_BasicFunction (DCB03FN)
	db		2dh, 00h		;PWM_PH_B_DIG_Input (DCB03IN)
	db		2eh, 00h		;PWM_PH_B_DIG_Output (DCB03OU)
;       Instance name PWM_PH_B, Block Name PWM8(DCB02)
	db		28h, 00h		;PWM_PH_B_DIG_BasicFunction (DCB02FN)
	db		29h, 00h		;PWM_PH_B_DIG_Input (DCB02IN)
	db		2ah, 00h		;PWM_PH_B_DIG_Output (DCB02OU)
;  Instance name PWM_PH_C, User Module PWMDB8
;       Instance name PWM_PH_C, Block Name DB8(DBB11)
	db		34h, 00h		;PWM_PH_C_DIG_BasicFunction (DBB11FN)
	db		35h, 00h		;PWM_PH_C_DIG_Input (DBB11IN)
	db		36h, 00h		;PWM_PH_C_DIG_Output (DBB11OU)
;       Instance name PWM_PH_C, Block Name PWM8(DBB10)
	db		30h, 00h		;PWM_PH_C_DIG_BasicFunction (DBB10FN)
	db		31h, 00h		;PWM_PH_C_DIG_Input (DBB10IN)
	db		32h, 00h		;PWM_PH_C_DIG_Output (DBB10OU)
	db		ffh

;  Instance name ADC, User Module DELSIG8
;       Instance name ADC, Block Name ADC1(ASC12)
;       Instance name ADC, Block Name ADC2(ASD22)
;       Instance name ADC, Block Name TMR(DBB01)
;  Instance name AMUX4_1, User Module AMUX4
;  Instance name Amp_Vac, User Module INSAMP
;       Instance name Amp_Vac, Block Name INV(ACB01)
;       Instance name Amp_Vac, Block Name NON_INV(ACB00)
;  Instance name Count_I, User Module Counter8
;       Instance name Count_I, Block Name CNTR8(DBB00)
;  Instance name FlashTemp_1, User Module FlashTemp
;       Instance name FlashTemp_1, Block Name TEMPERATURE(ASC21)
;  Instance name LPF2_I, User Module LPF2
;       Instance name LPF2_I, Block Name FLIN(ASC23)
;       Instance name LPF2_I, Block Name FLOUT(ASD13)
;  Instance name Limit_I, User Module CMPPRG
;       Instance name Limit_I, Block Name COMP(ACB03)
;  Instance name PGA_1, User Module PGA
;       Instance name PGA_1, Block Name GAIN(ACB02)
;  Instance name PWM_PH_A, User Module PWMDB8
;       Instance name PWM_PH_A, Block Name DB8(DCB13)
;       Instance name PWM_PH_A, Block Name PWM8(DCB12)
;  Instance name PWM_PH_B, User Module PWMDB8
;       Instance name PWM_PH_B, Block Name DB8(DCB03)
;       Instance name PWM_PH_B, Block Name PWM8(DCB02)
;  Instance name PWM_PH_C, User Module PWMDB8
;       Instance name PWM_PH_C, Block Name DB8(DBB11)
;       Instance name PWM_PH_C, Block Name PWM8(DBB10)
	db		ffh
UnloadConfigTBL_Total_Bank0:
;  Block DBB00
	db		23h, 00h		; CONTROL_0 register (DBB00CR0)
;  Block DBB01
	db		27h, 00h		; CONTROL_0 register (DBB01CR0)
;  Block DCB02
	db		2bh, 00h		; CONTROL_0 register (DCB02CR0)
;  Block DCB03
	db		2fh, 00h		; CONTROL_0 register (DCB03CR0)
;  Block DBB10
	db		33h, 00h		; CONTROL_0 register (DBB10CR0)
;  Block DBB11
	db		37h, 00h		; CONTROL_0 register (DBB11CR0)
;  Block DCB12
	db		3bh, 00h		; CONTROL_0 register (DCB12CR0)
;  Block DCB13
	db		3fh, 00h		; CONTROL_0 register (DCB13CR0)
;  Block ACB00
	db		73h, 00h		; CR2 register (ACB00CR2)
;  Block ASC10
	db		83h, 00h		; CR3 register (ASC10CR3)
;  Block ASD20
	db		93h, 00h		; CR3 register (ASD20CR3)
;  Block ACB01
	db		77h, 00h		; CR2 register (ACB01CR2)
;  Block ASD11
	db		87h, 00h		; CR3 register (ASD11CR3)
;  Block ASC21
	db		97h, 00h		; CR3 register (ASC21CR3)
;  Block ACB02
	db		7bh, 00h		; CR2 register (ACB02CR2)
;  Block ASC12
	db		8bh, 00h		; CR3 register (ASC12CR3)
;  Block ASD22
	db		9bh, 00h		; CR3 register (ASD22CR3)
;  Block ACB03
	db		7fh, 00h		; CR2 register (ACB03CR2)
;  Block ASD13
	db		8fh, 00h		; CR3 register (ASD13CR3)
;  Block ASC23
	db		9fh, 00h		; CR3 register (ASC23CR3)
	db		ffh
UnloadConfigTBL_Total_Bank1:
;  Block DBB00
	db		20h, 00h		; DIG_BasicFunction register (DBB00FN)
	db		21h, 00h		; DIG_Input register (DBB00IN)
	db		22h, 00h		; DIG_Output register (DBB00OU)
;  Block DBB01
	db		24h, 00h		; DIG_BasicFunction register (DBB01FN)
	db		25h, 00h		; DIG_Input register (DBB01IN)
	db		26h, 00h		; DIG_Output register (DBB01OU)
;  Block DCB02
	db		28h, 00h		; DIG_BasicFunction register (DCB02FN)
	db		29h, 00h		; DIG_Input register (DCB02IN)
	db		2ah, 00h		; DIG_Output register (DCB02OU)
;  Block DCB03
	db		2ch, 00h		; DIG_BasicFunction register (DCB03FN)
	db		2dh, 00h		; DIG_Input register (DCB03IN)
	db		2eh, 00h		; DIG_Output register (DCB03OU)
;  Block DBB10
	db		30h, 00h		; DIG_BasicFunction register (DBB10FN)
	db		31h, 00h		; DIG_Input register (DBB10IN)
	db		32h, 00h		; DIG_Output register (DBB10OU)
;  Block DBB11
	db		34h, 00h		; DIG_BasicFunction register (DBB11FN)
	db		35h, 00h		; DIG_Input register (DBB11IN)
	db		36h, 00h		; DIG_Output register (DBB11OU)
;  Block DCB12
	db		38h, 00h		; DIG_BasicFunction register (DCB12FN)
	db		39h, 00h		; DIG_Input register (DCB12IN)
	db		3ah, 00h		; DIG_Output register (DCB12OU)
;  Block DCB13
	db		3ch, 00h		; DIG_BasicFunction register (DCB13FN)
	db		3dh, 00h		; DIG_Input register (DCB13IN)
	db		3eh, 00h		; DIG_Output register (DCB13OU)
;  Block ACB00
;  Block ASC10
;  Block ASD20
;  Block ACB01
;  Block ASD11
;  Block ASC21
;  Block ACB02
;  Block ASC12
;  Block ASD22
;  Block ACB03
;  Block ASD13
;  Block ASC23
	db		ffh


; PSoC Configuration file trailer PsocConfig.asm
