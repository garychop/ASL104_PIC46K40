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
#include "head_array_common.h"
#include "rtos_app.h"

// from local
#include "head_array_bsp.h"

/* ******************************   Macros   ****************************** */

#define DIG_RIGHT_PAD_PIN 		PIN_B2
#define DIG_CTR_PAD_PIN  		PIN_B3
#define DIG_LEFT_PAD_PIN   		PIN_B4
#define DIG_PAD_ACTIVE_STATE	GPIO_LOW

#define ANA_RIGHT_PAD_PIN 		PIN_A1
#define ANA_CTR_PAD_PIN  		PIN_A2
#define ANA_LEFT_PAD_PIN   		PIN_A0

#define ADC_LEFT_PAD_MAX_VAL	(225)
#define ADC_RIGHT_PAD_MAX_VAL	(225)
#define ADC_CTR_PAD_MAX_VAL		(225)

/* ***********************   File Scope Variables   *********************** */

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: headArrayBspInit
//
// Description: Initializes this module.
//
//-------------------------------
void headArrayBspInit(void)
{
	set_analog_pins(AN0_TO_AN2, VSS_VDD);
	setup_adc(ADC_CLOCK_INTERNAL);
}

//-------------------------------
// Function: headArrayBspDigitalState
//
// Description: Reads the digital input state of a single head array sensor.
//
//-------------------------------
bool headArrayBspDigitalState(HeadArraySensor_t sensor_id)
{
	uint8_t sensor_pin;

	switch (sensor_id)
	{
		case HEAD_ARRAY_SENSOR_LEFT:
			sensor_pin = DIG_LEFT_PAD_PIN;
			break;

		case HEAD_ARRAY_SENSOR_RIGHT:
			sensor_pin = DIG_RIGHT_PAD_PIN;
			break;
			
		case HEAD_ARRAY_SENSOR_CENTER:
			sensor_pin = DIG_CTR_PAD_PIN;
			break;
			
		case HEAD_ARRAY_SENSOR_EOL:
		default:
			ASSERT(false);
			sensor_pin = DIG_RIGHT_PAD_PIN; // Just so it's assigned to "something"
			break;
	}

	return (input(sensor_pin) == DIG_PAD_ACTIVE_STATE);
}

//-------------------------------
// Function: headArrayBspAnalogState
//
// Description: Reads the analog input state of a single head array sensor.
//
//-------------------------------
uint16_t headArrayBspAnalogState(HeadArraySensor_t sensor_id)
{
	uint8_t sensor_pin;

	switch (sensor_id)
	{
		case HEAD_ARRAY_SENSOR_LEFT:
			sensor_pin = ANA_LEFT_PAD_PIN;
			break;

		case HEAD_ARRAY_SENSOR_RIGHT:
			sensor_pin = ANA_RIGHT_PAD_PIN;
			break;
			
		case HEAD_ARRAY_SENSOR_CENTER:
			sensor_pin = ANA_CTR_PAD_PIN;
			break;
			
		case HEAD_ARRAY_SENSOR_EOL:
		default:
			ASSERT(false);
			sensor_pin = DIG_RIGHT_PAD_PIN; // Just so it's assigned to "something"
			break;
	}

	set_adc_channel(sensor_pin);

	// TODO: The CCS documentation simply says "Calls to setup_adc(), setup_adc_ports() and
	// TODO: set_adc_channel() should be made sometime before this function is called." It would be good to
	// TODO: Have a more deterministic way to wait between set and read.
	rtosAppDelayMs(1);

	// This has the ADC in continuous mode. So, even after this read the ADC is still capturing at some
	// periodic rate.
	uint16_t raw_adc_reading = read_adc();

	// Do this instead of calling the func twice for speed.
	uint16_t max_val = headArrayBspProportionalMaxValue(sensor_id);
	
	// TODO: Figure out why this has been done.
	if (raw_adc_reading >= max_val)
	{
		raw_adc_reading = max_val;
	}

	return raw_adc_reading;
}

//-------------------------------
// Function: headArrayBspProportionalMaxValue
//
// Description: Let's the caller know what the maximum value a proportional signal can be, in terms of # of ADC counts.
//
//-------------------------------
uint16_t headArrayBspProportionalMaxValue(HeadArraySensor_t sensor_id)
{
	switch (sensor_id)
	{
		case HEAD_ARRAY_SENSOR_LEFT:
			return ADC_LEFT_PAD_MAX_VAL;
			
		case HEAD_ARRAY_SENSOR_RIGHT:
			return ADC_RIGHT_PAD_MAX_VAL;

		case HEAD_ARRAY_SENSOR_CENTER:
			return ADC_CTR_PAD_MAX_VAL;

		default:
			(void)0;
			break;
	}
}

// end of file.
//-------------------------------------------------------------------------
