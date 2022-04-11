//////////////////////////////////////////////////////////////////////////////
//
// Filename: head_array_bsp.h
//
// Description: Provides functionality for interpretting head array input.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef HEAD_ARRAY_BSP_H
#define HEAD_ARRAY_BSP_H

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdint.h>
#include <stdbool.h>

// from project
#include "head_array_common.h"

/* ******************************   Macros   ****************************** */


/* ***********************   Function Prototypes   ************************ */

void headArrayBspInit(void);
bool headArrayBspDigitalState(HeadArraySensor_t sensor_id);

#endif // HEAD_ARRAY_BSP_H

// end of file.
//-------------------------------------------------------------------------
