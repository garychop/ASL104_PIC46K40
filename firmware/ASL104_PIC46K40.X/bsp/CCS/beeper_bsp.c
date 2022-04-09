//////////////////////////////////////////////////////////////////////////////
//
// Filename: beeper_bsp.c
//
// Description: BSP level definitions for beeper
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
#include <stdbool.h>

// from project
#include "bsp.h"

// from local
#include "beeper_bsp.h"


/* ******************************   Macros   ****************************** */

#define BEEPER_PIN				(PIN_D0)
#define BEEPER_ACTIVE_STATE		(GPIO_LOW)
#define BEEPER_INACTIVE_STATE	(GPIO_HIGH)

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: beeperBspInit
//
// Description: Initializes this module.
//
//-------------------------------
void beeperBspInit(void)
{
	beeperBspActiveSet(false);
}

//-------------------------------
// Function: beeperBspInit
//
// Description: Sets the active/inactive state of the beeper
//
//-------------------------------
void beeperBspActiveSet(bool active)
{
	output_bit(BEEPER_PIN, active ? BEEPER_ACTIVE_STATE : BEEPER_INACTIVE_STATE);
}

//-------------------------------
// Function: beeperBspInit
//
// Description: Gets the active/inactive state of the beeper
//
//-------------------------------
bool beeperBspActiveGet(void)
{
	return (input(BEEPER_PIN) == BEEPER_ACTIVE_STATE) ? true : false;
}

// end of file.
//-------------------------------------------------------------------------
