//----------------------------------------------------------------------------
//
// Solar Pump Controller. Entry C2982
//
// C main line
//
// Copyright 2004-2007
//
//----------------------------------------------------------------------------

#include <m8c.h>        // part specific constants and macros
#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
#include "isqrt.h"		// Integer sqrt function

/* CONSTANTS */

#define ROOT2 1.41421356

//
// Define to generate a fixed amplitude open loop 50 Hz waveform
//
//#define OPEN_LOOP_CONTROL

// Define this value for fixed frequency
//#define OPEN_LOOP_FREQ 50

// Define this value for fixed amplitude voltage
//#define OPEN_LOOP_VOLTAGE 4096

//
// main control interrupt rate
//
#define TIMER_FREQ (24000000L / (8*256*2))

// useful timer constants
#define CONST_1000MS (TIMER_FREQ)
#define CONST_500MS  (TIMER_FREQ>>1)

//
// phase shift (in samples) between phases
//
#define WAVESTEP ((120*256)/360)

// Hardware constants
#define ADC_FULL_SCALE 127
#define VAC_FULL_SCALE 400		// pk
#define ARRAY_FULL_SCALE 100
#define EDGE_THRESHOLD 2
#define MAX_AMBIENT 60			// heatsink needs to be coupled to micro temperature
#define TEMP_HYST 5				// hystersis on temp fault
#define ARRAY_HYST ((5*ADC_FULL_SCALE)/ARRAY_FULL_SCALE) 

// Restart time in seconds
// The pump will restart this many seconds after no active faults
#define RESTART_TIME_THRESHOLD  60

//
// Volatile control setpoints structure
//
typedef struct SETPOINTS {
	BYTE VAC_LEVEL_RMS;
	BYTE NOM_FREQ;
	BYTE MIN_FREQ;
} SETPOINTS;
	
// 
// Control levels
//
#define DC_GAIN 2			 // DC voltage regulator control gain * 256
#define VAC_GAIN 8			 // VAC voltage regulator control gain * 256
#define MIN_MPPT_VOLTAGE 48  // minimum sane value for MPPT voltage
#define MAX_MPPT_VOLTAGE 75  // maximum sane value for MPPT voltage
#define MPPT_SCAN_PERIOD 1800 // seconds between rescanning for MPPT voltage
#define PANEL_SETTLE_TIME 5	 // seconds to allow panel to reach open-circuit under no load
#define MPPT_PERCENT_VOC 70	 // percentage of open-circuit voltage that corresponds to MPPT voltage
#define ARRAY_MIN_VOLTAGE 39 // minimum system run voltage

// Scaled values 
#define ARRAY_MIN ((ADC_FULL_SCALE * ARRAY_MIN_VOLTAGE) / ARRAY_FULL_SCALE)
#define	MIN_MPPT_VLEVEL ((ADC_FULL_SCALE * MIN_MPPT_VOLTAGE)/ARRAY_FULL_SCALE)
#define	MAX_MPPT_VLEVEL ((ADC_FULL_SCALE * MAX_MPPT_VOLTAGE)/ARRAY_FULL_SCALE)
#define MPPT_GAIN_VOC ((MPPT_PERCENT_VOC*256)/100)	

// fault mask bits
#define FAULT_OC 1
#define FAULT_LOW_DC 2
#define FAULT_TEMP 4
#define FAULT_DRY 8


//
// PWM dead time in PWM ticks
//
#define DEAD_TIME 2

//
// cycles of over regulation voltage before dryrun fault is raised
//
// ~60 seconds @ 50 Hz
//
#define MAX_DRYRUN 3000

//
// mux channel selections for the DC and AC sample inputs
//
#define CHAN_VAC   		AMUX4_1_PORT0_3
#define CHAN_CURRENT    AMUX4_1_PORT0_7
#define CHAN_ARRAY 		AMUX4_1_PORT0_5     

//
// LED status bits on port 2
//
#define LED_DRY  6
#define LED_ON 1
#define LED_OT 7
#define LED_OC 3

//
// Dip switch inputs
//
#define SW1_PORT PRT2DR
#define SW1_BIT 0
#define SW2_PORT PRT1DR
#define SW2_BIT 5
#define SW3_PORT PRT1DR
#define SW3_BIT 4

//
// Sigma-delta 7-bit+sign ADC control and status registers
//
extern volatile BYTE ADC_bfStatus;  
extern volatile CHAR ADC_cResult;  

//
// internal state variables
//
static volatile WORD  timer;	// Elapsed time counter
static BYTE  second_flag=0;		// Set when a second has elapsed
static WORD  nco_freq;			// NCO frequency << 8
static DWORD index;				// NCO phase << 16
static BYTE indexb;				// NCO sample index (0..255)
static DWORD index_step;		// NCO phase velocity << 16
static DWORD new_index_step;	// latched value of new velocity
static volatile char index_step_flag=0;	// flag indicating that a new velocity has been calculated
static short gain=0;				// output gain 0..32767 corresponds to 0..1.0
static BYTE sector,last_sector;	// NCO waveform phase sector number (0..15)
 
static BYTE channel;			// current sampler channel
static BYTE	sample_count=0;		// sample discard counter
static signed char vac_s;				// instantaneous vac voltage sample
static signed char vac_dc_offset;		// vac dc offset
static signed char array_s;			// averaged array voltage
static signed char iac_s;				// averaged iac voltage sample
static signed char iac_dc_offset;		// iac dc offset
static long iac_accum;			// iac sample integrator
static WORD iac_n;				// # samples taken of iac
static long array_accum;		// array voltage sample integrator
static WORD array_n;			// # samples taken of array voltage
static signed char ambient=0;			// ambient temperature in degrees
static BYTE vac_rms;			// raw vac rms
// vac rms accumulator (instantaneous and latched)
static DWORD vac_accum,latched_vac_accum;

static BYTE control_state;		// background controller state machine state variable
static BYTE control_on;			// control enabled flag
static WORD restart_timer;		// auto restart timer
static WORD second_mark;		// Used to sense a second has elapsed

// gain << 8 to be applied to frequency to give desired voltage regulation level
static WORD v2f_gain;

// desired output voltage 
static BYTE vac_reference;

// 2 seconds at 50 Hz 
#define CNT_SIZE 100

// Switch position integrators
static BYTE sw1_cnt=CNT_SIZE>>1,sw2_cnt=CNT_SIZE>>1,sw3_cnt=CNT_SIZE>>1;
// Switch pressed event flags
static BYTE sw1_flag=0,sw2_flag=0,sw3_flag=0;

// Setpoints structure
static SETPOINTS setpoints;

//
// Timer used to detect a dryrun condition
//
static WORD dryrun_integrator=0;


//
// Active and latched fault registers
//
static char latched_faults=0,active_faults=0;

//
// MPPT control voltage
//
static BYTE MPPT_VLEVEL = (MAX_MPPT_VLEVEL + MIN_MPPT_VLEVEL) >> 1; 

// MPPT scan timer
static signed short mppt_scan_timer=MPPT_SCAN_PERIOD;

//
// Conditionally halt the CPU with a pulsing waveform appearing on the ON led
//
// Primative diagnostic routine
//
void DiagLoop(BYTE x,BYTE forever) 
{
	BYTE y;

 	M8C_DisableGInt;
 		
	do 
	{
		M8C_ClearWDT;
		y=0;
		do
		 {	 
			PRT2DR &= ~(1<<LED_ON);
	
		 	if(y < x)
				PRT2DR |= (1<<LED_ON);
	
			y++;
		} while(y != 0);						    
    } while(forever);
    
 
	M8C_ClearWDT;
	M8C_EnableGInt;					  
}

// Reset a timer reference to the current timer tick
void Mark(WORD *mark)
{
	*mark = timer;
}

// Returned elapsed timer ticks relative to the given timer reference
WORD Elapsed(WORD mark)
{
	return timer-mark;
}

#pragma interrupt_handler 	pump_isr

//
// composite sine inverter 7-bit normalised waveform lookup table
//
static const char wavelut[] = {
0x00,0x03,0x06,0x09,0x0c,0x0f,0x12,0x16,
0x19,0x1c,0x1f,0x22,0x25,0x28,0x2b,0x2e,
0x31,0x33,0x36,0x39,0x3c,0x3f,0x41,0x44,
0x47,0x49,0x4c,0x4e,0x51,0x53,0x55,0x58,
0x5a,0x5c,0x5e,0x60,0x62,0x64,0x66,0x68,
0x6a,0x6b,0x6d,0x6f,0x70,0x72,0x73,0x74,
0x76,0x77,0x78,0x79,0x7a,0x7b,0x7b,0x7c,
0x7d,0x7d,0x7e,0x7e,0x7f,0x7f,0x7f,0x7f,
0x7f,0x7f,0x7f,0x7f,0x7f,0x7e,0x7e,0x7d,
0x7d,0x7c,0x7b,0x7b,0x7a,0x79,0x78,0x77,
0x76,0x74,0x73,0x72,0x70,0x6f,0x6f,0x71,
0x72,0x73,0x75,0x76,0x77,0x78,0x79,0x7a,
0x7b,0x7c,0x7c,0x7d,0x7e,0x7e,0x7e,0x7f,
0x7f,0x7f,0x7f,0x7f,0x7f,0x7f,0x7f,0x7f,
0x7e,0x7e,0x7d,0x7d,0x7c,0x7b,0x7a,0x79,
0x78,0x77,0x76,0x75,0x74,0x73,0x71,0x70,
0x6e,0x6d,0x6b,0x69,0x67,0x66,0x64,0x62,
0x60,0x5e,0x5b,0x59,0x57,0x55,0x52,0x50,
0x4d,0x4b,0x48,0x46,0x43,0x40,0x3e,0x3b,
0x38,0x35,0x32,0x30,0x2d,0x2a,0x27,0x24,
0x21,0x1e,0x1b,0x18,0x15,0x11,0x0e,0x0b,
0x08,0x05,0x02,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

//
// main control interrupt. runs at the switching frequency.
//
void pump_isr()
{
			
	// bump system timer
	timer++;
	
	// load new velocity if request is active
	if(index_step_flag)
	{
		index_step = new_index_step;
		index_step_flag=0;
	}
		
	// advance NCO phase
	index += index_step;
	
	// extract sample index
	indexb = ((BYTE *)&index)[1];

	// set multiplier to output gain
	MUL_Y = ((BYTE *)&gain)[0];
	
	// load 8-bit samples into duty registers, scaled by output gain
	MUL_X = wavelut[indexb];
	
	PWM_PH_A_PULSE_WIDTH_REG = DEAD_TIME + (MUL_DH << 2);

	indexb += WAVESTEP;
	
	MUL_X = wavelut[indexb];

	PWM_PH_B_PULSE_WIDTH_REG = DEAD_TIME + (MUL_DH << 2);

	indexb += WAVESTEP;
	
	MUL_X = wavelut[indexb];

	PWM_PH_C_PULSE_WIDTH_REG = DEAD_TIME + (MUL_DH << 2);
	
	//PRT2DR ^= (1<<LED_DRY);
			
}

//
// Maximum gain that can be selected without causing problems with PWM duty cycle
//
#define MAX_GAIN ((4194304L - 32768L*DEAD_TIME)/127)

//
// Set waveform generator gain 0..32767 == 0..1
//
void vc_set_gain(signed short _gain)
{
	// clip negative gains
	if(_gain < 0)
		_gain = 0;
		
	// clip to prevent duty cycles exceeding switching period - dead time 
	if(_gain > MAX_GAIN) 
		_gain = MAX_GAIN;
		
	// update ISR value	
	gain = _gain;
}

//
// Set waveform generator frequency 0..65 Hz << 8
//
void vc_set_freq(WORD freq)
{
	if(!index_step_flag)
	{
		
		if((freq>>8) < setpoints.MIN_FREQ )
			freq = (WORD)setpoints.MIN_FREQ << 8;
		if((freq>>8) > setpoints.NOM_FREQ)
			freq = (WORD)setpoints.NOM_FREQ << 8;	
						
		// save
		nco_freq = freq;
		
		// convert frequency to phase velocity as 8.16 fixed point value
		new_index_step = ((DWORD)freq << 16)/TIMER_FREQ;
	
		// inform interrupt to update velocity
		index_step_flag = 1;
		
	}
	
}

//
// Initialise and enable the NCO
// to generator the required frequency 
//
// freq = 0..255 Hz
//
void init_NCO(BYTE freq)
{
		
	// reset NCO
	index = 0;
	indexb = 0;
	sector = 0;
	last_sector = 0;
	
	// configure output frequency & gain
	vc_set_freq((WORD)freq << 8);
	vc_set_gain(0);

	PWM_PH_A_WritePulseWidth(DEAD_TIME);
	PWM_PH_B_WritePulseWidth(DEAD_TIME);
	PWM_PH_C_WritePulseWidth(DEAD_TIME);
	
	// start pwm generators
	PWM_PH_A_Start();
	PWM_PH_B_Start();
	PWM_PH_C_Start();
				
	// enable isr on first PWM generator
	PWM_PH_B_DisableInt();
	PWM_PH_C_DisableInt();		
	PWM_PH_A_EnableInt();
	
}
													 
//
// initialise the analog sampler
//
void init_sampler(void)
{
	
	// enable the differential amp for the inverter voltage sensing
	Amp_Vac_Start(Amp_Vac_MEDPOWER);
	
	// enable the front end low-pass filter for the primary current
	LPF2_I_Start(LPF2_I_HIGHPOWER);
	
	// Start single-ended input amplifier
	PGA_1_Start(PGA_1_MEDPOWER );

	// Provide Analog ground on output pin 
	RefAGND_RefSelect(RefAGND_AGND);
														 		
	// select the mux input to the ADC
	AMUX4_1_InputSelect(channel=CHAN_VAC);

	// enable the ADC and start sampling	
    ADC_Start( ADC_HIGHPOWER );
    
}

//
// perform internal calibration
//
void calibrate_sampler(void)
{
	signed short accum;
	WORD samples,measurement,mark;

	//
	// wait 1 second for circuitry to stablise
	//
	Mark(&mark);
	while(Elapsed(mark) < CONST_1000MS)
	{
		if(timer & 0x200)
			PRT2DR |= (1<<LED_ON) | (1<<LED_DRY) | (1<<LED_OC) | (1<<LED_OT);
		else
			PRT2DR &= ~((1<<LED_ON) | (1<<LED_DRY) | (1<<LED_OC) | (1<<LED_OT));
			
		M8C_ClearWDT;
	}
	
	PRT2DR &= ~((1<<LED_ON) | (1<<LED_DRY) | (1<<LED_OC) | (1<<LED_OT));
	    
    //
    // enable flash (ambient) temperature sampling
    // and take initial sample
    //
	FlashTemp_1_Start();

	while(!FlashTemp_1_fIsData()) 
    	M8C_ClearWDT;
			
	ambient=FlashTemp_1_cGetData();
	
	FlashTemp_1_Stop();
				
	AMUX4_1_InputSelect(channel=CHAN_VAC);	
	
    ADC_StartAD();  
    
	//
	// average 256 samples from vac to determine DC offset 
	//
	accum=0;
	ADC_bfStatus=0;
	for(samples=0;samples<256;samples++)
	{
		while (! ADC_bfStatus ) ;
		accum += ADC_cResult;
		ADC_bfStatus=0;
    	// tickle watchdog
    	M8C_ClearWDT;		
	}

	// store averaged DC offset
	vac_dc_offset = (char)(accum >> 8);	

	AMUX4_1_InputSelect(channel=CHAN_CURRENT);	
	
	//
	// average 256 samples from iac to determine DC offset 
	//
	accum=0;
	ADC_bfStatus=0;
	for(samples=0;samples<256;samples++)
	{
		while (! ADC_bfStatus ) ;
		accum += ADC_cResult;
		ADC_bfStatus=0;
    	// tickle watchdog
    	M8C_ClearWDT;		
	}

	// store averaged DC offset
	iac_dc_offset = (char)(accum >> 8);	

	sample_count = 0;

 	vac_accum=0;		      	
 	array_accum=0;
 	array_n=0;
 	iac_accum=0;
 	iac_n=0;

	AMUX4_1_InputSelect(channel=CHAN_ARRAY);	
}

#define abs(x) ((x)<0?-(x):(x))

//
// process incoming ADC samples
//
// sample rate = 1300 Hz per channel
//
void run_sampler(void)
{
  		//
  		// discard first two samples, to allow for the ADC to settle on the new channel value
  		//
    	if(++sample_count == 3)
     	{
  	     	 sample_count=0;

  	     	 // store incoming sample and change mux setting
	         switch(channel) 
	         {
	         	default:
	         	case CHAN_VAC:
	         	
	         		vac_s = ADC_cResult ;
				 	AMUX4_1_InputSelect(channel=CHAN_ARRAY);
					
	         		break;
	         		
	         	case CHAN_ARRAY:
	         	
	         		array_accum += ADC_cResult;
	         		array_n++;
			      	AMUX4_1_InputSelect(channel=CHAN_CURRENT); 
			      				      	
	         		break; 
	         			
	         	case CHAN_CURRENT:        		  	
        		  	
	         		iac_accum += abs(ADC_cResult-iac_dc_offset);
	         		iac_n++;
			      	AMUX4_1_InputSelect(channel=CHAN_VAC);	  
			      		        		
			      	break;
	          }
	   }
	          
       // clear status flag / restart sampler
       ADC_bfStatus = 0;  
	     
		// pick up RMS sample sector number 0..15
		sector = indexb >> 4;
	
		// sense change in sector
		if(sector != last_sector)
		{
			// update change detector
			last_sector=sector;
	
			// zero crossing (sector 0)?
			if(!sector)
			{
				latched_vac_accum = vac_accum;
			
				// start background control thread			
				control_state=1;
				
				// reset accumulator
				vac_accum = 0;	
			}
			
			// update vac accumulator with latest sample value						
			vac_accum +=  (vac_s-vac_dc_offset)*(vac_s-vac_dc_offset) ;
	}		
}

//
// initialise the over current fault detector 
//
void init_fault_monitoring(void)
{
	// Enable the current comparator
	Limit_I_Start(Limit_I_HIGHPOWER);

	// Set the current comparator voltage level	
	Limit_I_SetRef(Limit_I_REF0_937);
	
	//
	// Enable the over current pulse counter
	//
	Count_I_WritePeriod(255);
	Count_I_WriteCompareValue(2);
	Count_I_Start();
}

//
// Measure switch states and update debouncing filters & decision bits
//
void sample_switches(void)
{

				if(SW1_PORT & (1<<SW1_BIT))
				{
					if(sw1_cnt != CNT_SIZE)
						sw1_cnt++;
					else
						sw1_flag=1;		
				}
				else
				{
					if(sw1_cnt != 0)
						sw1_cnt--;
					else
						sw1_flag=0;
					
				}
				if(SW2_PORT & (1<<SW2_BIT))
				{
					if(sw2_cnt != CNT_SIZE)
						sw2_cnt++;
					else
						sw2_flag=1;
				}
				else
				{
					if(sw2_cnt != 0)
						sw2_cnt--;
					else
						sw2_flag=0;
					
				}
				if(SW3_PORT & (1<<SW3_BIT))
				{
					if(sw3_cnt != CNT_SIZE)
						sw3_cnt++;
					else
						sw3_flag=1;
				}
				else
				{
					if(sw3_cnt != 0)
						sw3_cnt--;
					else
						sw3_flag=0;
						
				}
				
}

//
// translate switch settings to setpoint levels
//				
void translate_switches(void)
{				
				if(sw1_flag)
					setpoints.NOM_FREQ = 50;
				else
					setpoints.NOM_FREQ = 60;
				
				setpoints.VAC_LEVEL_RMS = 100;
				
				if(sw2_flag)
					setpoints.MIN_FREQ = 25;
				else
					setpoints.MIN_FREQ = 40;

				if(!sw3_flag)
					setpoints.NOM_FREQ += 5;
				
				// recalculate control gain	
 				v2f_gain = ((DWORD)(setpoints.VAC_LEVEL_RMS * ADC_FULL_SCALE) << 8) / (setpoints.NOM_FREQ * VAC_FULL_SCALE);							
}

//
// initialise control algorithm
//
void init_control(void)
{
 	control_state=0;
 	control_on=0;

	// set up initial waveform generator frequency
    init_NCO(setpoints.NOM_FREQ);   

	// reset voltage output level of bridge
	vc_set_gain(0);

	// setup initial second timer		 	
	Mark(&second_mark);

	active_faults = latched_faults = 0;
	dryrun_integrator=0;
	
	// force a scan for the maximum power point
	mppt_scan_timer=0;
}

//
// set setpoints
//
void init_setpoints(void)
{
	WORD cnt;

	// measure power-on switch settings
	for(cnt=0;cnt<512;cnt++)
	{
		sample_switches();
		M8C_ClearWDT;   	
	}
		
	// convert to system setpoints
	translate_switches();	
}

//
// clear active fault bits 
//
// keeps latched fault bits
//
void clear_fault(BYTE mask)
{
	active_faults &= ~mask;
}

//
// set active and latched fault, and shut down controller
//
void set_fault(BYTE mask)
{
	latched_faults |= mask;
	active_faults |= mask;

	// stop controller
	control_on=0;
	vc_set_gain(0);	
	restart_timer=0;
}

//
// trigger state machine for control action
//
void run_control(void)
{
	WORD temp;
	signed char x;
	static BYTE cnt=0;
	
		// control thread
		switch(control_state) 
		{
			// idle
			default:
			case 0:
				control_state =0;
				break;
				
			// extract normalised vac rms				
			case 1:
			
				vac_rms = isqrt((WORD)(latched_vac_accum >> 4));
										
				control_state++;
				break;			
	
			// AC voltage regulator
			case 2:

				if(control_on && (mppt_scan_timer > 0))
				{
				 vac_reference = (v2f_gain * (nco_freq >> 8)) >> 8;

#ifndef OPEN_LOOP_VOLTAGE
				  
			   	 vc_set_gain( gain + (vac_reference - vac_rms) * VAC_GAIN);
				 
#else
				 vc_set_gain(OPEN_LOOP_VOLTAGE);
#endif						 
				}
				else
					vc_set_gain(0);	
				
				control_state++;
				break;
				
			// check for faults
			case 3:
			
				// read average result from integrator
				array_s = array_accum / array_n;
				if(array_s < 0)
					array_s = 0;
									
				// reset integrator
				array_accum = 0;
				array_n = 0;
				
				iac_s = iac_accum / iac_n;
				
				// reset integrator
				iac_accum = 0;
				iac_n = 0;

				if(array_s < ARRAY_MIN)
					set_fault(FAULT_LOW_DC);
				else if(array_s > ARRAY_MIN + ARRAY_HYST)
					clear_fault(FAULT_LOW_DC);
				
				if(ambient > MAX_AMBIENT)
					set_fault(FAULT_TEMP);
				else if(ambient < MAX_AMBIENT - TEMP_HYST)
					clear_fault(FAULT_TEMP);
				
				if(Count_I_bReadCounter() < 255-EDGE_THRESHOLD)
					set_fault(FAULT_OC);
				else
					clear_fault(FAULT_OC);
					
				// Reset current pulse counter
				Count_I_Stop();
				Count_I_WritePeriod(255);
				Count_I_Start();
							
				control_state++;
				break;
			
			// encode fault LEDs	
			case 4:
			
				if(latched_faults & FAULT_OC)
					PRT2DR |= 1<<LED_OC;
				else
					PRT2DR &= ~(1<<LED_OC);	
					
				if(latched_faults & FAULT_DRY)
					PRT2DR |= 1<<LED_DRY;
				else
					PRT2DR &= ~(1<<LED_DRY);
					
					
				if(latched_faults & FAULT_TEMP)
					PRT2DR |= 1<<LED_OT;
				else
					PRT2DR &= ~(1<<LED_OT);

				//PRT2DR ^= (1<<LED_ON);

#ifndef TEST_VAC

												
				if(control_on)
					PRT2DR |= 1<<LED_ON;
				else
				{ 	if(restart_timer > 0)
					{
						if(second_flag)
							PRT2DR ^= (1<<LED_ON);
					}
					else
						PRT2DR &= ~(1<<LED_ON);
				}	
#else

	
			
		DiagLoop(vac_rms,0);
	
		
					
#endif
						
				control_state++;
				break;
			
			// auto-restart timer.. runs every second
			case 5:
			
				if(second_flag)
				{

#ifndef OPEN_LOOP_CONTROL
	
					if(!control_on) 
					{					
						if(!active_faults)
						{
							if(++restart_timer > RESTART_TIME_THRESHOLD)
							{
								// restart control after a predefined period if no active faults
								latched_faults = 0;
								dryrun_integrator=0;
								vc_set_gain(0);
								control_on=1;						
							}
						}
						else
						{
							// clear active faults that cannot clear themselves
							if(active_faults & FAULT_DRY)
							{
								// this will take ~50 times longer to clear
								// approximately 1 hr
								if(dryrun_integrator != 0)
									dryrun_integrator--;
								else
									clear_fault(FAULT_DRY);
																
							}
							
						}
								
					}
					else
					{
						restart_timer = 0;
					}
#else

				control_on=1;
				
#endif

				}
								
				
				control_state++;
				break;
				
			// switch sampling
			case 6:

				// measure switch inputs
				sample_switches();
							
				// translate switch settings to setpoint levels
				translate_switches();
												
				control_state++;
				break;
		
		//
		// dc voltage regulator (changes frequency to regulate dc voltage)
		//
		case 7:
		
#ifndef OPEN_LOOP_FREQ	

			if(control_on)
			{
				if(mppt_scan_timer > 0)
				{	
					// see if DC voltage regulator is controlling correctly
				 	if(array_s > MPPT_VLEVEL)
				 	{
				 		// no, load on pump is too low - trigger dry run fault
						if(dryrun_integrator < MAX_DRYRUN)
							dryrun_integrator++;
						else
							set_fault(FAULT_DRY);
				 	}
				 	else
				 	{
						if(dryrun_integrator!=0)
							dryrun_integrator--;
				 	}
					vc_set_freq( nco_freq - (MPPT_VLEVEL - array_s) * DC_GAIN); 				 	
				}
				else 
				{
					// see if the panels have reached open-circuit voltage yet
					if(mppt_scan_timer <= -PANEL_SETTLE_TIME)
					{				
						// MPPT target voltage = fixed fraction of open-circuit voltage
						MPPT_VLEVEL = ((BYTE)array_s * MPPT_GAIN_VOC) >> 8;
						
						// check the target MPPT voltage is sane
						if(MPPT_VLEVEL < MIN_MPPT_VLEVEL)
							MPPT_VLEVEL = MIN_MPPT_VLEVEL;
						if(MPPT_VLEVEL > MAX_MPPT_VLEVEL)
							MPPT_VLEVEL = MAX_MPPT_VLEVEL;
							
						// rearm mppt scan timer (deactivate scan)
						mppt_scan_timer = MPPT_SCAN_PERIOD;
					}
															
				}
				
			}	
#else
				vc_set_freq(OPEN_LOOP_FREQ << 8);
#endif
			
			control_state++;
			break;		
		
		// second timer	
		case 8:
		
			if(Elapsed(second_mark) >= CONST_1000MS)
			{		
				Mark(&second_mark);
				
				second_flag=1;
				
				if(control_on)
				{
					// count down seconds until the mppt scan is active
					if(mppt_scan_timer > -PANEL_SETTLE_TIME)
						mppt_scan_timer--;
				}
				
			}
			else
				second_flag=0;
					
			control_state++;
			break;

		// temperature sampling
		case 9:			
			// sample the ambient (chip) temperature and IIR it
			if(FlashTemp_1_fIsData())
			{
				ambient=(char)(((short)ambient+(short)FlashTemp_1_cGetData())>>1);	

				//restart temperature sampling			
				FlashTemp_1_Start();
			}
			control_state++;
			break;			
	}		

}


//
// main loop for pump inverter
//
void main()
{

 	M8C_DisableGInt;

 	M8C_EnableWatchDog;

	init_setpoints();
  	 	
 	init_fault_monitoring();

 	init_control();
 	
	init_sampler();
     			
	M8C_EnableGInt;

	calibrate_sampler();

    for(;;)
    {
    	// tickle watchdog
    	M8C_ClearWDT;
    	  	
    	// process incoming ADC samples
      	if ( ADC_bfStatus ) 
        		run_sampler();
		
		// process control state machine      		
		if(control_state)
			run_control();
		
   }     	  
}