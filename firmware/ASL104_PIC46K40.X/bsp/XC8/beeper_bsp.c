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
#include "common.h"

// from local
#include "beeper_bsp.h"

/* ******************************   Macros   ****************************** */

#define BEEPER_IS_ACTIVE()	(LATCbits.LC1 == GPIO_LOW)
#define BEEPER_SET(active)	INLINE_EXPR(LATCbits.LC1 = active ? GPIO_LOW : GPIO_HIGH)
#define BEEPER_TOGGLE()		INLINE_EXPR(BEEPER_SET(BEEPER_IS_ACTIVE() ? false : true))

#ifdef _18F46K40
    #define BEEPER_INIT()		INLINE_EXPR(TRISCbits.TRISC1 = GPIO_BIT_OUTPUT; ANSELCbits.ANSELC1 = 0; BEEPER_SET(false))
#else
    #define BEEPER_INIT()		INLINE_EXPR(TRISDbits.TRISD0 = GPIO_BIT_OUTPUT; BEEPER_SET(false))
#endif

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: beeperBspInit
//
// Description: Initializes this module.
//
//-------------------------------
void beeperBspInit(void)
{
	BEEPER_INIT();
}

//-------------------------------
// Function: beeperBspInit
//
// Description: Sets the active/inactive state of the beeper
//
//-------------------------------
void beeperBspActiveSet(bool active)
{
	BEEPER_SET(active);
}

//-------------------------------
// Function: beeperBspInit
//
// Description: Gets the active/inactive state of the beeper
//
//-------------------------------
bool beeperBspActiveGet(void)
{
	return BEEPER_IS_ACTIVE();
}

// end of file.
//-------------------------------------------------------------------------
