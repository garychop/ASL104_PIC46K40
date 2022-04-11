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
	BEEPER_PATTERN_GOTO_IDLE,
    BEEPER_PATTERN_RESUME_DRIVING,
	BEEPER_PATTERN_EEPROM_NOT_INIT_ON_BOOT,
	BEEPER_PATTERN_MODE_ACTIVE,
	BEEPER_PATTERN_PAD_ACTIVE,
	// Nothing else may be defined past this point!
	BEEPER_PATTERN_EOL
} BeepPattern_t;

typedef struct 
{
    Msg_t m_Super;
    uint8_t m_MsgID;
} BeepMsg_t;

// Mailbox definitions for sending info to Beep Task.
extern uint8_t g_BeeperTaskID;
extern BeepPattern_t g_NewBeepPattern;

/* ***********************   Function Prototypes   ************************ */

void beeperInit(void);
void beeperBeep(BeepPattern_t pattern);
//Evt_t beeperBeepBlocking(BeepPattern_t pattern);
//Evt_t BeeperWaitUntilPatternCompletes(void);
bool IsBeepEnabled(void);

#endif // BEEPER_H

// end of file.
//-------------------------------------------------------------------------
