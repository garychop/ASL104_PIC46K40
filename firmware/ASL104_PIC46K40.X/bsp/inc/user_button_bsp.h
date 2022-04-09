//////////////////////////////////////////////////////////////////////////////
//
// Filename: user_button_bsp.h
//
// Description: Handles bsp level user button access.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef USER_BUTTON_BSP_H
#define USER_BUTTON_BSP_H

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdbool.h>

/* ***********************   Function Prototypes   ************************ */

void ButtonBspInit(void);
bool userButtonBspIsActive(void);
bool ModeButtonBspIsActive(void);

#endif // USER_BUTTON_BSP_H

// end of file.
//-------------------------------------------------------------------------
