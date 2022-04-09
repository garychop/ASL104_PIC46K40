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

// The min/max values below are based on empherical data gathered from a single rev3 board head array.
// Temperature not taken into account during the test.
#define ADC_LEFT_PAD_MAX_VAL	(915)
#define ADC_RIGHT_PAD_MAX_VAL	(915)
#define ADC_CTR_PAD_MAX_VAL		(915)

// There is not enough difference between a pad being disconnected and no pressure being applied to a pad to be able
// reliably tell the two states apart.
#define ADC_LEFT_PAD_MIN_VAL	(0)
#define ADC_RIGHT_PAD_MIN_VAL	(0)
#define ADC_CTR_PAD_MIN_VAL		(0)

/* ***********************   Function Prototypes   ************************ */

void headArrayBspInit(void);
bool headArrayBspDigitalState(HeadArraySensor_t sensor_id);
uint16_t headArrayBspAnalogState(HeadArraySensor_t sensor_id);
uint16_t headArrayBspProportionalMaxValue(HeadArraySensor_t sensor_id);
uint16_t headArrayBspProportionalMinValue(HeadArraySensor_t sensor_id);

#endif // HEAD_ARRAY_BSP_H

// end of file.
//-------------------------------------------------------------------------
