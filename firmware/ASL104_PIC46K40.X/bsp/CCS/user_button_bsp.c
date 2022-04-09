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
#include "bsp.h"

// from local
#include "user_button_bsp.h"

/* ******************************   Macros   ****************************** */

#define USER_BTN_PIN   			PIN_B1
#define USER_BTN_ACTIVE_STATE	GPIO_LOW

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: userButtonBspInit
//
// Description: Initializes this module.
//
//-------------------------------
void userButtonBspInit(void)
{
	(void)0;
}

//-------------------------------
// Function: userButtonBspIsActive
//
// Description: Reads the digital input state of the user button
//
//-------------------------------
bool userButtonBspIsActive(void)
{
	return (input(USER_BTN_PIN) == USER_BTN_ACTIVE_STATE) ? true : false;
}

// end of file.
//-------------------------------------------------------------------------
