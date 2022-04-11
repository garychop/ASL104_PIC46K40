//////////////////////////////////////////////////////////////////////////////
//
// Filename: head_array_bsp.c
//
// Description: Provides functionality for interpretting head array input.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////


/* **************************   Header Files   *************************** */

// NOTE: This must ALWAYS be the first include in a file.
#include "device.h"

// from stdlib
#include <stdint.h>
#include <stdbool.h>
#include "user_assert.h"

// from project
#include "bsp.h"
#include "common.h"
#include "head_array_common.h"

// from local
#include "head_array_bsp.h"

/* ******************************   Macros   ****************************** */

#define DIG_PAD_ACTIVE_STATE		GPIO_LOW
#define DIG_PAD_INACTIVE_STATE		GPIO_HIGH

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: headArrayBspInit
//
// Description: Initializes this module.
//
//-------------------------------
void headArrayBspInit(void)
{
    TRISBbits.TRISB1 = GPIO_BIT_INPUT;      // D1 Pad
    TRISBbits.TRISB2 = GPIO_BIT_INPUT;      // D2 Pad
    TRISBbits.TRISB3 = GPIO_BIT_INPUT;      // D3 Pad
    TRISBbits.TRISB4 = GPIO_BIT_INPUT;      // D4 Pad
}

//-------------------------------
// Function: headArrayBspDigitalState
//
// Description: Reads the digital input state of a single head array sensor.
//
//-------------------------------
bool headArrayBspDigitalState(HeadArraySensor_t sensor_id)
{
	switch (sensor_id)
	{
		case HEAD_ARRAY_SENSOR_LEFT:
			return (PORTBbits.RB1 == DIG_PAD_ACTIVE_STATE);

		case HEAD_ARRAY_SENSOR_CENTER:
			return (PORTBbits.RB4 == DIG_PAD_ACTIVE_STATE);
			
		case HEAD_ARRAY_SENSOR_RIGHT:
			return (PORTBbits.RB3 == DIG_PAD_ACTIVE_STATE);

        case HEAD_ARRAY_SENSOR_BACK:
            return (PORTBbits.RB2 == DIG_PAD_ACTIVE_STATE);

		case HEAD_ARRAY_SENSOR_EOL:
		default:
			ASSERT(sensor_id == HEAD_ARRAY_SENSOR_CENTER);
			return false; // Return something.
	}
    return false;
}

// end of file.
//-------------------------------------------------------------------------
