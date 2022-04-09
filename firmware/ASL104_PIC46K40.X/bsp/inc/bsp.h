//////////////////////////////////////////////////////////////////////////////
//
// Filename: bsp.h
//
// Description: BSP level initialization and control for the MCU.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef BSP_H
#define BSP_H

/* ***************************    Includes     **************************** */

// for CCS compiler, #include "device.h" must be included in C file before this file.

// from stdlib
#include <stdint.h>

/* ******************************   Macros   ****************************** */

#define GPIO_LOW 		(0)
#define GPIO_HIGH 		(1)

#if defined(CCS_BUILD_CHAIN)
	#define GPIO_BIT_INPUT 	(0)
	#define GPIO_BIT_OUTPUT (1)
#elif defined(XC8_BUILD_CHAIN)
	#define GPIO_BIT_INPUT 	(1)
	#define GPIO_BIT_OUTPUT (0)
#endif

//-----------------------
// microsecond delay values
// NOTE: When calling bspDelayUs, only use these values!
//
// Through test (building with opt0/1/2) found that the only flag that gets set is this one.  Unfortunately,
// it is the same flag set for opt level 1 and 2. There is no difference in flags set between opt 1 and 2.
#if (__OPTIM_FLAGS & 0x01)
	// This is optimization level 2
	#define US_DELAY_20_us 		(4) // Actual: 20 us.  This is the minimum "optimization level common" delay value
	#define US_DELAY_50_us 		(10) // Actual: 51.6 us
	#define US_DELAY_75_us 		(19) // Actual: 76.8us
	#define US_DELAY_100_us 	(28) // Actual: 102 us
	#define US_DELAY_125_us 	(37) // Actual: 127.2 us
	#define US_DELAY_150_us 	(45) // Actual: 149.6 us
	#define US_DELAY_175_us 	(54) // Actual: 174.8us
	#define US_DELAY_200_us 	(63) // Actual: 200 us
	#define US_DELAY_225_us 	(72) // Actual: 225.2 us
	#define US_DELAY_250_us 	(81) // Actual: 250.4 us
	#define US_DELAY_275_us 	(90) // Actual: 275.6 us
	#define US_DELAY_300_us 	(99) // Actual: 300.8 us
	#define US_DELAY_325_us 	(108) // Actual: 326 us
	#define US_DELAY_350_us 	(117) // Actual: 351.2 us
	#define US_DELAY_375_us 	(126) // Actual: 376.4 us
	#define US_DELAY_400_us 	(134) // Actual: 398.8 us
	#define US_DELAY_425_us 	(144) // Actual: 426.8 us
	#define US_DELAY_450_us 	(152) // Actual: 449.2 us
	#define US_DELAY_475_us 	(162) // Actual: 477.2 us
	#define US_DELAY_500_us 	(171) // Actual: 499.6. us
#else
	// This is optimization level 0
	#define US_DELAY_20_us 		(2) // Actual: 20 us.  This is the minimum "optimization level common" delay value
	#define US_DELAY_50_us 		(12) // Actual: 50.8 us
	#define US_DELAY_75_us 		(14) // Actual: 76.4us
	#define US_DELAY_100_us 	(22) // Actual: 102 us
	#define US_DELAY_125_us 	(30) // Actual: 127.6 us
	#define US_DELAY_150_us 	(37) // Actual: 150 us
	#define US_DELAY_175_us 	(45) // Actual: 175.2us
	#define US_DELAY_200_us 	(53) // Actual: 201.2 us
	#define US_DELAY_225_us 	(61) // Actual: 226.8 us
	#define US_DELAY_250_us 	(69) // Actual: 252.4 us
	#define US_DELAY_275_us 	(76) // Actual: 274.8 us
	#define US_DELAY_300_us 	(84) // Actual: 300.4 us
	#define US_DELAY_325_us 	(92) // Actual: 326 us
	#define US_DELAY_350_us 	(100) // Actual: 351.6 us
	#define US_DELAY_375_us 	(108) // Actual: 377.2 us
	#define US_DELAY_400_us 	(115) // Actual: 399.6 us
	#define US_DELAY_425_us 	(123) // Actual: 425.2 us
	#define US_DELAY_450_us 	(131) // Actual: 450.8 us
	#define US_DELAY_475_us 	(139) // Actual: 476.4 us
	#define US_DELAY_500_us 	(146) // Actual: 498.8. us
#endif

#define US_DELAY_MIN			US_DELAY_20_us

/* ***********************   Function Prototypes   ************************ */

void bspInitCore(void);
void bspDisableInterrupts(void);
void bspEnableInterrupts(void);
void bspDelayUs(uint16_t delay);
void bspDelayMs(uint16_t delay);

#endif // BSP_H

// end of file.
//-------------------------------------------------------------------------
