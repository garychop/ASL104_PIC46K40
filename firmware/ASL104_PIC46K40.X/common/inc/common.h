//////////////////////////////////////////////////////////////////////////////
//
// Filename: common.h
//
// Description: Common macros and functions for BSP and application use.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef COMMON_H
#define COMMON_H

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdint.h>
#include <stdbool.h>

/* ******************************   Macros   ****************************** */

#define UNUSED(x) ((void)x)

#define INLINE_EXPR(x) do { x; } while (0)

// Figures out how many elements are in an array
#define NUM_ELEMENTS_IN_ARR(x) (sizeof(x) / sizeof(x[0]))

/* ******************************   Types   ******************************* */

typedef union
{
	uint16_t val;
	uint8_t bytes[2];
} TypeAccess16Bit_t;

typedef union
{
	uint32_t val;
	uint8_t bytes[4];
} TypeAccess32Bit_t;

#endif // COMMON_H

// end of file.
//-------------------------------------------------------------------------
