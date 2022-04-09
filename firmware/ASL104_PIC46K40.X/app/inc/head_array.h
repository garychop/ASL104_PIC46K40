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

typedef enum
{
	HEAD_ARR_INPUT_DIGITAL,
	HEAD_ARR_INPUT_PROPORTIONAL,

	// Nothing else may be defined past this point!
	HEAD_ARR_INPUT_EOL
} HeadArrayInputType_t;

typedef enum
{
	HEAD_ARRAY_OUT_AXIS_LEFT_RIGHT,
	HEAD_ARRAY_OUT_AXIS_FWD_REV,

	// Nothing else may be defined past this point!
	HEAD_ARRAY_OUT_AXIS_EOL
} HeadArrayOutputAxis_t;

typedef enum
{
	HEAD_ARRAY_OUT_FUNC_LEFT,
	HEAD_ARRAY_OUT_FUNC_RIGHT,
	HEAD_ARRAY_OUT_FUNC_FWD,
	HEAD_ARRAY_OUT_FUNC_REV,
	HEAD_ARRAY_OUT_FUNC_NONE,

	// Nothing else may be defined past this point!
	HEAD_ARRAY_OUT_FUNC_EOL
} HeadArrayOutputFunction_t;

/* ***********************   Function Prototypes   ************************ */

void headArrayinit(void);

uint16_t headArrayOutputValue(HeadArrayOutputAxis_t axis_id);
uint16_t headArrayDigitalDacOutputValue(HeadArrayOutputAxis_t axis_id);
uint16_t headArrayProportionalDacOutputValue(HeadArrayOutputAxis_t axis_id);

bool headArrayDigitalInputValue(HeadArraySensor_t sensor);
uint16_t headArrayProportionalInputValueRaw(HeadArraySensor_t sensor);
uint16_t headArrayProportionalInputValue(HeadArraySensor_t sensor);

bool headArrayPadIsConnected(HeadArraySensor_t sensor);

bool headArrayNeutralTestFail(void);
void SetNeedForNeutralTest (void);

#endif // HEAD_ARRAY_H

// end of file.
//-------------------------------------------------------------------------
