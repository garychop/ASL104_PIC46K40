//////////////////////////////////////////////////////////////////////////////
//
// Filename: beeper_bsp.h
//
// Description: BSP level definitions for beeper
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef BEEPER_BSP_H
#define BEEPER_BSP_H

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdbool.h>

/* ***********************   Function Prototypes   ************************ */

void beeperBspInit(void);
void beeperBspActiveSet(bool active);
bool beeperBspActiveGet(void);
bool IsBeepFeatureEnable (void);

#endif // BEEPER_BSP_H

// end of file.
//-------------------------------------------------------------------------
