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

#include "rtos_app.h"

// from local
#include "bsp.h"

/* ***********************   Function Prototypes   ************************ */

static void InterruptsInit(void);
static void TimersInit(void);

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: bspInit
//
// Description: Initializes basic and core BSP functionality.
//
//-------------------------------
void bspInitCore(void)
{
	// Require pull-ups on IO lines.
	port_b_pullups(TRUE);

	// On-chip USB transceiver disabled; digital (GPIO) transceiver interface enabled.
	UTRDIS = 1;

    TimersInit();
}

//-------------------------------
// Function: bspKickOffInterrupts
//
// Description: Enables all interrupts for each peripherals of care as well as global interrupts.
//
//-------------------------------
void bspKickOffInterrupts(void)
{
	enable_interrupts(INT_TIMER0);
	enable_interrupts(INT_TIMER2);
	enable_interrupts(INT_AD);
	enable_interrupts(INT_LOWVOLT);
	enable_interrupts(INT_OSCF);
	enable_interrupts(GLOBAL);
}

//-------------------------------
// Function: bspEnableInterrupts
//
// Description: Enables interrupts.
//
//-------------------------------
void bspEnableInterrupts(void)
{
	enable_interrupts(GLOBAL);
}

//-------------------------------
// Function: bspDisableInterrupts
//
// Description: Disables interrupts.
//
//-------------------------------
void bspDisableInterrupts(void)
{
	disable_interrupts(GLOBAL);
}

//-------------------------------
// Function: bspDelayUs
//
// Description: Delays for some number of microseconds.
//
//-------------------------------
void bspDelayUs(uint16_t delay)
{
	// TODO: Take into account the time it takes to call and exit this function.
	rtosAppDelayUs(delay);
}

//-------------------------------
// Function: bspDelayUs
//
// Description: Delays for some number of milliseconds.
//
//-------------------------------
void bspDelayMs(uint16_t delay)
{
	// TODO: Take into account the time it takes to call and exit this function.
	rtosAppDelayMs(delay);
}

/* ********************   Private Function Definitions   ****************** */

//-------------------------------
// Function: InterruptsInit
//
// Description: Initializes timers
//
//-------------------------------
static void TimersInit(void)
{
    // 409us overflow in theory.
	// Actual: 384us
	setup_timer_0(T0_INTERNAL | T0_DIV_8 | T0_8_BIT);

    // 499us overflow, 998us interrupt in theory.
	// Actual: 819us
	setup_timer_2(T2_DIV_BY_16, 155, 2);
}

// end of file.
//-------------------------------------------------------------------------
