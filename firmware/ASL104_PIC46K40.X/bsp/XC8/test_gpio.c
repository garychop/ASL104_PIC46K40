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
#include "common.h"

// from local
#include "test_gpio.h"

// Suppress "function is never called" warning for this file.
#pragma warning disable 520
#pragma warning disable 2053

/* ******************************   Macros   ****************************** */

#ifdef _18F46K40
    #define TEST_GPIO_0_IS_ACTIVE()		(LATAbits.LA3 == GPIO_HIGH)
    #define TEST_GPIO_0_SET(active)		INLINE_EXPR(LATAbits.LA3 = active ? GPIO_HIGH : GPIO_LOW)
    #define TEST_GPIO_0_TOGGLE()		INLINE_EXPR(TEST_GPIO_0_SET(TEST_GPIO_0_IS_ACTIVE() ? false : true))
    #define TEST_GPIO_0_INIT()			INLINE_EXPR(TRISAbits.TRISA3 = GPIO_BIT_OUTPUT; TEST_GPIO_0_SET(false); ANSELAbits.ANSELA3 = 0)

    #define TEST_GPIO_1_IS_ACTIVE()		(LATAbits.LA4 == GPIO_HIGH)
    #define TEST_GPIO_1_SET(active)		INLINE_EXPR(LATAbits.LA4 = active ? GPIO_HIGH : GPIO_LOW)
    #define TEST_GPIO_1_TOGGLE()		INLINE_EXPR(TEST_GPIO_1_SET(TEST_GPIO_1_IS_ACTIVE() ? false : true))
    #define TEST_GPIO_1_INIT()			INLINE_EXPR(TRISAbits.TRISA4 = GPIO_BIT_OUTPUT; TEST_GPIO_1_SET(false); ANSELAbits.ANSELA4 = 0)
#else
    #define TEST_GPIO_0_IS_ACTIVE()		(LATAbits.LA3 == GPIO_HIGH)
    #define TEST_GPIO_0_SET(active)		INLINE_EXPR(LATAbits.LA3 = active ? GPIO_HIGH : GPIO_LOW)
    #define TEST_GPIO_0_TOGGLE()		INLINE_EXPR(TEST_GPIO_0_SET(TEST_GPIO_0_IS_ACTIVE() ? false : true))
    #define TEST_GPIO_0_INIT()			INLINE_EXPR(TRISAbits.TRISA3 = GPIO_BIT_OUTPUT; TEST_GPIO_0_SET(false))

    #define TEST_GPIO_1_IS_ACTIVE()		(LATAbits.LA4 == GPIO_HIGH)
    #define TEST_GPIO_1_SET(active)		INLINE_EXPR(LATAbits.LA4 = active ? GPIO_HIGH : GPIO_LOW)
    #define TEST_GPIO_1_TOGGLE()		INLINE_EXPR(TEST_GPIO_1_SET(TEST_GPIO_1_IS_ACTIVE() ? false : true))
    #define TEST_GPIO_1_INIT()			INLINE_EXPR(TRISAbits.TRISA4 = GPIO_BIT_OUTPUT; TEST_GPIO_1_SET(false))
#endif

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
	TEST_GPIO_0_INIT();
	TEST_GPIO_1_INIT();
}

//-------------------------------
// Function: testGpioSet
//
// Description: Sets the state of a test GPIO.
//
// NOTE: Execution time for this function is ~18.4us @10MHz system clock
//
//-------------------------------
inline void testGpioSet(TestGpio_t gpio_id, bool is_high)
{
	switch (gpio_id)
	{
		case TEST_GPIO_0:
			TEST_GPIO_0_SET(is_high);
			break;
			
		case TEST_GPIO_1:
			TEST_GPIO_1_SET(is_high);
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
// NOTE: Execution time for this function is between 14.8 us and 16.8 us @10MHz system clock
//
//-------------------------------
inline void testGpioToggle(TestGpio_t gpio_id)
{
	switch (gpio_id)
	{
		case TEST_GPIO_0:
			TEST_GPIO_0_TOGGLE();
			break;
			
		case TEST_GPIO_1:
			TEST_GPIO_1_TOGGLE();
			break;
		
		default:
			(void)0;
			break;
	}
}
#endif

// end of file.
//-------------------------------------------------------------------------
