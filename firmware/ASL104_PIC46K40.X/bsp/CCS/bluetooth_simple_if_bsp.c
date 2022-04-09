//////////////////////////////////////////////////////////////////////////////
//
// Filename: bluetooth_simple_if_bsp.c
//
// Description: Exposes control over the simple interface to a bluetooth module.
// TODO: Actually call out the name of the Bluetooth module.
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
#include "head_array_common.h"

// from local
#include "bluetooth_simple_if_bsp.h"

/* ******************************   Macros   ****************************** */

#define BT_LEFT_PAD_MIRROR_PIN 		PIN_E1 // Pin 7 on Bluetooth header
#define BT_LEFT_PAD_TRIS_MASK		(0x02)

#define BT_RIGHT_PAD_MIRROR_PIN  	PIN_C1 // Pin 8 on Bluetooth header
#define BT_RIGHT_PAD_TRIS_MASK		(0x02)

#define BT_CTR_PAD_MIRROR_PIN   	PIN_D3 // Pin 9 on Bluetooth header
#define BT_CTR_PAD_TRIS_MASK		(0x08)

#define BT_PAD_MIRROR_ACTIVE_STATE	GPIO_LOW


/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: bluetoothSimpleIfBspInit
//
// Description: Initializes this module
//
//-------------------------------
void bluetoothSimpleIfBspInit(void)
{
	// Set all pins as outputs.
	set_tris_e(get_tris_e() & ~BT_LEFT_PAD_TRIS_MASK);
	set_tris_c(get_tris_c() & ~BT_RIGHT_PAD_TRIS_MASK);
	set_tris_d(get_tris_d() & ~BT_CTR_PAD_TRIS_MASK);
}

//-------------------------------
// Function: bluetoothSimpleIfBspInit
//
// Description: Tells the Bluetooth module that the input mirroring is disabled.
//
//-------------------------------
void bluetoothSimpleIfBspPadMirrorDisable(void)
{
	bluetoothSimpleIfBspPadMirrorStateSet(HEAD_ARRAY_SENSOR_LEFT, false);
	bluetoothSimpleIfBspPadMirrorStateSet(HEAD_ARRAY_SENSOR_RIGHT, false);
	bluetoothSimpleIfBspPadMirrorStateSet(HEAD_ARRAY_SENSOR_CENTER, false);
}

//-------------------------------
// Function: bluetoothSimpleIfBspPadMirrorStateSet
//
// Description: Sets the state of the mirrored heead array sensor output to the bluetooth module.
//
//-------------------------------
void bluetoothSimpleIfBspPadMirrorStateSet(HeadArraySensor_t sensor_id, bool active)
{
	uint8_t output_pin;

	switch (sensor_id)
	{
		case HEAD_ARRAY_SENSOR_LEFT:
			output_pin = BT_LEFT_PAD_MIRROR_PIN;
			break;

		case HEAD_ARRAY_SENSOR_RIGHT:
			output_pin = BT_RIGHT_PAD_MIRROR_PIN;
			break;
			
		case HEAD_ARRAY_SENSOR_CENTER:
			output_pin = BT_CTR_PAD_MIRROR_PIN;
			break;
			
		case HEAD_ARRAY_SENSOR_EOL:
		default:
			ASSERT(false);
			
			// Just so it's assigned to "something"
			output_pin = BT_CTR_PAD_MIRROR_PIN;
			break;
	}

	// TODO: See if there is a function that is like: output(PIN, STATE)
	if (active)
	{
#if (BT_PAD_MIRROR_ACTIVE_STATE == GPIO_LOW)
		output_low(output_pin);
#else
		output_high(output_pin);
#endif
	}
	else
	{
#if (BT_PAD_MIRROR_ACTIVE_STATE == GPIO_LOW)
		output_high(output_pin);
#else
		output_low(output_pin);
#endif
	}
}

//-------------------------------
// Function: bluetoothSimpleIfBspPadMirrorStateGet
//
// Description: Gets the state of the mirrored heead array sensor output to the bluetooth module.
//
//-------------------------------
bool bluetoothSimpleIfBspPadMirrorStateGet(HeadArraySensor_t sensor_id)
{
	uint8_t output_pin;

	switch (sensor_id)
	{
		case HEAD_ARRAY_SENSOR_LEFT:
			output_pin = BT_LEFT_PAD_MIRROR_PIN;
			break;

		case HEAD_ARRAY_SENSOR_RIGHT:
			output_pin = BT_RIGHT_PAD_MIRROR_PIN;
			break;
			
		case HEAD_ARRAY_SENSOR_CENTER:
			output_pin = BT_CTR_PAD_MIRROR_PIN;
			break;
			
		case HEAD_ARRAY_SENSOR_EOL:
		default:
			ASSERT(false);
			
			// Just so it's assigned to "something"
			output_pin = BT_CTR_PAD_MIRROR_PIN;
			break;
	}

	return (input(output_pin) == BT_PAD_MIRROR_ACTIVE_STATE);
}

// end of file.
//-------------------------------------------------------------------------
