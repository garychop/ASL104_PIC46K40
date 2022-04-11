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
#include "user_button_bsp.h"

/* ******************************   Macros   ****************************** */


#ifdef _18F46K40
//#define BEEPER_INIT()		INLINE_EXPR(TRISDbits.TRISD0 = GPIO_BIT_OUTPUT; ANSELDbits.ANSELD0 = 0; BEEPER_SET(false))
#else

#define BEEPER_INIT()		INLINE_EXPR(TRISCbits.TRISC1 = GPIO_BIT_OUTPUT; BEEPER_SET(false))
#define BEEPER_IS_ACTIVE()	(LATCbits.LC1 == GPIO_LOW)
#define BEEPER_SET(active)	INLINE_EXPR(LATCbits.LC1 = active ? GPIO_LOW : GPIO_HIGH)
#define BEEPER_TOGGLE()		INLINE_EXPR(BEEPER_SET(BEEPER_IS_ACTIVE() ? false : true))

#endif

/* *******************   Public Function Definitions   ******************** */

//------------------------------------------------------------------------------

bool BEEPER_IS_ACTIVE(void)
{
    return (LATCbits.LC1 == GPIO_LOW);
}
void BEEPER_SET(bool active)
{
    LATCbits.LC1 = active ? GPIO_LOW : GPIO_HIGH;
}
void BEEPER_TOGGLE(void)
{
    BEEPER_SET(BEEPER_IS_ACTIVE() ? false : true);
}

//-------------------------------
// Function: beeperBspInit
//
// Description: Initializes this module.
//
//-------------------------------
void beeperBspInit(void)
{
    TRISDbits.TRISD3 = GPIO_BIT_INPUT;      // This is DIP Switch #6
    
    TRISCbits.TRISC1 = GPIO_BIT_OUTPUT;
    BEEPER_SET(false);
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

//-------------------------------
// Function: IsBeepFeatureEnable
//
// Description: Gets the active/inactive state of the beeper switch
//      on the back panel.
//
//-------------------------------
bool IsBeepFeatureEnable (void)
{
    return (Is_SW6_ON());
}

// end of file.
//------------------------------------------------------------------------------
