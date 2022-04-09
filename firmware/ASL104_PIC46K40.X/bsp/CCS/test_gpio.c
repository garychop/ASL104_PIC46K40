//////////////////////////////////////////////////////////////////////////////
//
// Filename: test_gpio.c
//
// Description: Initializes and exposes control of GPIO used for test purposes.
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

// from project
#include "bsp.h"

// from local
#include "test_gpio.h"

/* *******************   Public Function Definitions   ******************** */

#if defined(DEBUG)
//-------------------------------
// Function: testGpioInit
//
// Description: Initializes this module.
//
//-------------------------------
void testGpioInit(void)
{
	for (int i = 0; i < (int)TEST_GPIO_EOL; i++)
	{
		testGpioSet((TestGpio_t)i, false);
	}
}

//-------------------------------
// Function: testGpioSet
//
// Description: Sets the state of a test GPIO.
//
// NOTE: Execution time for this function is ~11us @10MHz system clock
//
//-------------------------------
#inline void testGpioSet(TestGpio_t gpio_id, bool is_high)
{
	switch (gpio_id)
	{
		case TEST_GPIO_0:
			output_bit(PIN_A3, is_high ? GPIO_HIGH : GPIO_LOW);
			break;
			
		case TEST_GPIO_1:
			output_bit(PIN_A4, is_high ? GPIO_HIGH : GPIO_LOW);
			break;
		
		default:
			(void)0;
			break;
	}
}

//-------------------------------
// Function: testGpioToggle
//
// Description: Toggles a GPIO's state.
//
// NOTE: Execution time for this function is ~4.5us @10MHz system clock
//
//-------------------------------
#inline void testGpioToggle(TestGpio_t gpio_id)
{
	switch (gpio_id)
	{
		case TEST_GPIO_0:
			output_toggle(PIN_A3);
			break;
			
		case TEST_GPIO_1:
			output_toggle(PIN_A4);
			break;
		
		default:
			(void)0;
			break;
	}
}
#endif

// end of file.
//-------------------------------------------------------------------------
