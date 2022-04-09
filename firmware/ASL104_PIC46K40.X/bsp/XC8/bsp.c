//////////////////////////////////////////////////////////////////////////////
//
// Filename: bsp.c
//
// Description: BSP level initialization and control for the MCU.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////


/* **************************   Header Files   *************************** */

// NOTE: This must ALWAYS be the first include in a file.
#include "device.h"

// from local
#include "bsp.h"

/* ******************************   Macros   ****************************** */

#define ISR_LOW_PRIO_SET_VAL 	0
#define ISR_HIGH_PRIO_SET_VAL 	1

/* ***********************   Function Prototypes   ************************ */

static void InterruptsInit(void);
static void SysTickTimerInit(void);

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: bspInit
//
// Description: Initializes basic and core BSP functionality.
//
//-------------------------------
void bspInitCore(void)
{
#ifdef _18F46K40
	SysTickTimerInit();
	
    INTCONbits.IPEN = 1; // Enable priorities on interrupts.
    INTCONbits.PEIE_GIEL = 1; // Enable peripheral interrupts
#else
	UCONbits.USBEN = 0; // Disable USB operations
    INTCON2bits.nRBPU = 0; // Disable pull ups on all Port B pins
	
	SysTickTimerInit();
	
    RCONbits.IPEN = 1; // Enable priorities on interrupts.
    INTCONbits.PEIE = 1; // Enable peripheral interrupts
#endif
}

//-------------------------------
// Function: bspEnableInterrupts
//
// Description: Enables interrupts.
//
//-------------------------------
void bspEnableInterrupts(void)
{
	INTCONbits.GIEL = 1;
	INTCONbits.GIEH = 1;
}

//-------------------------------
// Function: bspDisableInterrupts
//
// Description: Disables interrupts.
//
//-------------------------------
void bspDisableInterrupts(void)
{
	INTCONbits.GIEL = 0;
	INTCONbits.GIEH = 0;
}

//-------------------------------
// Function: bspDelayUs
//
// Description: Delays for some number of microseconds.
//
// NOTE: Takes about 7.6 us to execute a delay of even 1 us. So, minimum delay is 7.6 us.
//
//-------------------------------
void bspDelayUs(uint16_t delay)
{
	if (delay == 0)
	{
		return;
	}
	else if (delay < 256)
	{
		// TODO: Needs calibration
		for (uint8_t i = 0; i < (uint8_t)delay; i++)
		{
			(void)0;
		}
	}
	else
	{
		// TODO: Needs calibration
		for (uint16_t i = 0; i < delay - 32; i++)
		{
			(void)0;
		}
	}
}

//-------------------------------
// Function: bspDelayUs
//
// Description: Delays for some number of milliseconds.
//
//-------------------------------
void bspDelayMs(uint16_t delay)
{
	for (uint16_t i = 0; i < delay; i++)
	{
		for (uint16_t j = 0; j < 224; j++)
		{
			(void)0;
		}
	}
}

/* ********************   Private Function Definitions   ****************** */

//-------------------------------
// Function: SysTickTimerInit
//
// Description: Initializes system tick timer
//
//-------------------------------
static void SysTickTimerInit(void)
{
    // Input frequency to Timer2 module is FOSC/4, which is 2.5 MHz for this application.
#ifdef _18F46K40
    T2CLKCONbits.CS = 0x01; // Select Fosc/4 as clock source for timer 2
    T2HLTbits.MODE = 0x00; // Free running timer mode, where TMR2ON control on/off
    T2CONbits.CKPS = 4; // /16 prescaler
    T2CONbits.OUTPS = 0; // /1 postscaler
    PR2 = 156; // Once the timer reaches this value the interrupt triggers
    IPR4bits.TMR2IP = ISR_LOW_PRIO_SET_VAL;
    PIE4bits.TMR2IE = 1; // Enable timer interrupt
    T2CONbits.TMR2ON = 1; // Enable the timer.
#else
    T2CONbits.T2CKPS = 3; // /16 prescaler
    T2CONbits.TOUTPS = 0; // /1 postscaler
    PR2 = 156; // Once the timer reaches this value the interrupt triggers
    IPR1bits.TMR2IP = ISR_LOW_PRIO_SET_VAL;
    PIE1bits.TMR2IE = 1; // Enable timer interrupt
    T2CONbits.TMR2ON = 1; // Enable the timer.
#endif
}

// end of file.
//-------------------------------------------------------------------------
