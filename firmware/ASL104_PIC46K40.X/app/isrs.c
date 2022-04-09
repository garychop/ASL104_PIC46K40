//////////////////////////////////////////////////////////////////////////////
//
// Filename: isrs.c
//
// Description: ISR entry points.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////


/* **************************   Header Files   *************************** */

// NOTE: This must ALWAYS be the first include in a file.
#include "device.h"

// from stdlib
#include <stdint.h>

// from RTOS
#include "cocoos.h"

// from project
#include "test_gpio.h"
#include "stopwatch.h"

static uint32_t num_os_ticks_to_process = 0;
static bool can_process_os_ticks = true;

/* *******************   Public Function Definitions   ******************** */


//------------------------------
// Function: isrsOsTickEnable
//
// Description: Allows OS tick to be processed or not be processed.
//		Needed to be disabled by some processes that are time critical.
//
//-------------------------------
void isrsOsTickEnable(bool enable)
{
	can_process_os_ticks = enable;
}

//------------------------------
// Function: highPrioIsr
//
// Description: Handles high priority interrupts
//
//-------------------------------
__interrupt(high_priority) void highPrioIsr(void)
{
	// ISRs here are assigned to the higher priority vector in bsp.c

	// low voltage
	// TODO: Stop all running processes and shutdown
	
	// oscillator fault
	// TODO: Stop all running processes and shutdown
}

//------------------------------
// Function: lowPrioIsr
//
// Description: Handles low priority interrupts
//
//-------------------------------
__interrupt(low_priority) void lowPrioIsr(void)
{
	// ISRs here are assigned to the lower priority vector in bsp.c
#ifdef _18F46K40
    if (PIR4bits.TMR2IF)
    {
        PIR4bits.TMR2IF = 0;
        
		stopwatchTick();
		num_os_ticks_to_process++;
		
		// This tick takes ~240 us. Which, when doing certain time critical operations may not be acceptable.
		// The system must be able to handle missing any number of ticks that are missed when os_tick() is disabled.
		if (can_process_os_ticks)
		{
			testGpioToggle(TEST_GPIO_0); // TODO: remove this. Only here for test.
			os_task_tick(0, num_os_ticks_to_process);
			num_os_ticks_to_process = 0;
		}
    }
#else
    if (PIR1bits.TMR2IF)
    {
        PIR1bits.TMR2IF = 0;
		stopwatchTick();
		num_os_ticks_to_process++;
		
		// This tick takes ~240 us. Which, when doing certain time critical operations may not be acceptable.
		// The system must be able to handle missing any number of ticks that are missed when os_tick() is disabled.
		if (can_process_os_ticks)
		{
			os_task_tick(0, num_os_ticks_to_process);
			num_os_ticks_to_process = 0;
		}
    }
#endif
}

// end of file.
//-------------------------------------------------------------------------
