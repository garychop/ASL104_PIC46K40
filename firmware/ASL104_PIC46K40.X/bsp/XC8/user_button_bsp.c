//////////////////////////////////////////////////////////////////////////////
//
// Filename: user_button_bsp.c
//
// Description: Handles bsp level user button access.
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
#include "common.h"
#include "bsp.h"

// from local
#include "user_button_bsp.h"

/* ******************************   Macros   ****************************** */

#define USER_BTN_IS_ACTIVE()	(PORTBbits.RB1 == GPIO_LOW)
#define USER_BTN_INIT()			INLINE_EXPR(TRISBbits.TRISB1 = GPIO_BIT_INPUT; ANSELBbits.ANSELB1 = 0)

#define MODE_BTN_IS_ACTIVE()	(PORTCbits.RC5 == GPIO_LOW)
#define MODE_BTN_INIT()			INLINE_EXPR(TRISCbits.TRISC5 = GPIO_BIT_INPUT; ANSELCbits.ANSELC5 = 0)

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: ButtonBspInit
//
// Description: Initializes this module.
//
//-------------------------------
void ButtonBspInit(void)
{
    USER_BTN_INIT();
    MODE_BTN_INIT();        // Set up the Port for MODE Button input.
}

//-------------------------------
// Function: userButtonBspIsActive
//
// Description: Reads the digital input state of the user button
// Returns: TRUE if the button is pushed, else false.
//-------------------------------
bool userButtonBspIsActive(void)
{
	return USER_BTN_IS_ACTIVE();
}

//-------------------------------
// Function: ModeButtonBspIsActive
//
// Description: Reads the digital input state of the MODE button
// Returns: TRUE if the button is pushed, else false.
//-------------------------------
bool ModeButtonBspIsActive(void)
{
	return MODE_BTN_IS_ACTIVE();
}

// end of file.
//-------------------------------------------------------------------------
