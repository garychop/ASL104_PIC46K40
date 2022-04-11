//////////////////////////////////////////////////////////////////////////////
//
// Filename: head_array.h
//
// Description: Core head array control and feedback.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef HEAD_ARRAY_H
#define HEAD_ARRAY_H

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdint.h>
#include <stdbool.h>

// from local
#include "head_array_common.h"

/* ******************************   Types   ******************************* */

/* ***********************   Function Prototypes   ************************ */

void headArrayinit(void);
bool headArrayDigitalInputValue(HeadArraySensor_t sensor);
bool headArrayPadIsConnected(HeadArraySensor_t sensor);
bool PadsInNeutralState (void);

#endif // HEAD_ARRAY_H

// end of file.
//-------------------------------------------------------------------------
