//////////////////////////////////////////////////////////////////////////////
//
// Filename: test_gpio.h
//
// Description: Initializes and exposes control of GPIO used for test purposes.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef TEST_GPIO_H
#define TEST_GPIO_H

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdbool.h>

/* ******************************   Types   ******************************* */

typedef enum
{
	TEST_GPIO_0,
	TEST_GPIO_1,

	// Must be last in the list!
	TEST_GPIO_EOL
} TestGpio_t;

/* ***********************   Function Prototypes   ************************ */

#if defined(DEBUG)
	void testGpioInit(void);
	inline void testGpioSet(TestGpio_t gpio_id, bool is_high);
	inline void testGpioToggle(TestGpio_t gpio_id);
#else
	#define testGpioInit()
	#define testGpioSet(gpio_id, is_high)
	#define testGpioToggle(gpio_id)
#endif

#endif // TEST_GPIO_H

// end of file.
//-------------------------------------------------------------------------
