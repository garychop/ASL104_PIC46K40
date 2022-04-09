//////////////////////////////////////////////////////////////////////////////
//
// Filename: user_assert.c
//
// Description: 
//
//  DETAILS
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////


/* **************************   Header Files   *************************** */

// from stdlib
#include <stdint.h>

// from local
#include "user_assert.h"

static volatile char *_file;
static volatile uint16_t _line;

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: assertion_trap
//
// Description: 
//
//-------------------------------
void assertion_trap(char *file, uint16_t line)
{
	_file = file;
	_line = line;

	// TODO: Put the system into a safe state.
	
	// Can view the file and line here.
	while (1)
	{
		(void)0;
	}
}

// end of file.
//-------------------------------------------------------------------------
