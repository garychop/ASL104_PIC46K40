//////////////////////////////////////////////////////////////////////////////
//
// Filename: bluetooth_simple_if_bsp.h
//
// Description: Exposes control over the simple interface to a bluetooth module.
// TODO: Actually call out the name of the Bluetooth module.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef BLUETOOTH_SIMPLE_IF_BSP_H
#define BLUETOOTH_SIMPLE_IF_BSP_H

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdbool.h>

// from project
#include "head_array_common.h"

/* ***********************   Function Prototypes   ************************ */

void bluetoothSimpleIfBspInit(void);
void bluetoothSimpleIfBspPadMirrorDisable(void);
void bluetoothSimpleIfBspPadMirrorStateSet(HeadArraySensor_t sensor_id, bool active);
bool bluetoothSimpleIfBspPadMirrorStateGet(HeadArraySensor_t sensor_id);

#endif // BLUETOOTH_SIMPLE_IF_BSP_H

// end of file.
//-------------------------------------------------------------------------
