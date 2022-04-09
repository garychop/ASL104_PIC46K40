//////////////////////////////////////////////////////////////////////////////
//
// Filename: beeper.h
//
// Description: Provides beeper control to the application.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef BEEPER_H
#define BEEPER_H

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdint.h>
#include <stdbool.h>

// from RTOS
#include "cocoos.h"

/* ******************************   Types   ******************************* */

// Values for selecting a desired beep pattern.
//
// NOTE: The first ones have to correspond with the features declared in the FunctionalFeature_t enum

typedef enum
{
    ANNOUNCE_POWER_ON,
    ANNOUNCE_BLUETOOTH,
    ANNOUNCE_NEXT_FUNCTION,
    ANNOUNCE_NEXT_PROFILE,
    ANNOUNCE_RNET_SEATING_ACTIVE,
    ANNOUNCE_BEEPER_RNET_SLEEP,
	BEEPER_PATTERN_USER_BUTTON_SHORT_PRESS,
	BEEPER_PATTERN_USER_BUTTON_LONG_PRESS,
	BEEPER_PATTERN_EEPROM_NOT_INIT_ON_BOOT,
	BEEPER_PATTERN_MODE_ACTIVE,
	// Nothing else may be defined past this point!
	BEEPER_PATTERN_EOL
} BeepPattern_t;


/* ***********************   Function Prototypes   ************************ */

void beeperInit(void);
Evt_t beeperBeep(BeepPattern_t pattern);
Evt_t beeperBeepBlocking(BeepPattern_t pattern);
Evt_t BeeperWaitUntilPatternCompletes(void);

#endif // BEEPER_H

// end of file.
//-------------------------------------------------------------------------
