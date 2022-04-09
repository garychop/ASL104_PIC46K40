//////////////////////////////////////////////////////////////////////////////
//
// Filename: user_assert.h
//
// Description: Implement a somewhat sain assert.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef USER_ASSERT_H
#define USER_ASSERT_H

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdint.h>

/* ******************************   Macros   ****************************** */

#if defined(assert)
#undef assert
#endif

#if defined(DEBUG)
    #define ASSERT(test) do{ if (!(test)){ assertion_trap((char *)__FILE__, (uint16_t)__LINE__); } } while(0)
#else
    #define ASSERT(test) (0)
#endif

/* ***********************   Function Prototypes   ************************ */

void assertion_trap(char *file, uint16_t line);

#endif // USER_ASSERT_H

// end of file.
//-------------------------------------------------------------------------
