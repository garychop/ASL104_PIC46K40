//////////////////////////////////////////////////////////////////////////////
//
// Filename: user_button.h
//
// Description: App level control and feedback for the user button.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef USER_BUTTON_H
#define USER_BUTTON_H

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdint.h>

/* ******************************   Types   ******************************* */

typedef enum
{
	USER_BTN_PRESS_NONE,
	USER_BTN_PRESS_SHORT,
	USER_BTN_PRESS_LONG,
	
	// Nothing else may be defined past this point!
	USER_BTN_PRESS_EOL
} UserButtonPress_t;

typedef enum
{
	BTN_ACTION_NONE,
	BTN_ACTION_FUNCTION,
	BTN_ACTION_PROFILE,
	BTN_ACTION_BT_PIN_MIRROR,
	BTN_ACTION_BT_POWER_ON_OFF,
	
	// Nothing else may be defined past this point!
	BTN_ACTION_EOL
} ButtonAction_t;

#define USER_SWITCH 0x01
#define MODE_SWITCH 0x02

/* ***********************   Function Prototypes   ************************ */

void userButtonInit(void);
uint8_t GetSwitchStatus(void);
bool IsModeSwitchActive(void);

#endif // USER_BUTTON_H

// end of file.
//-------------------------------------------------------------------------
