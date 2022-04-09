//////////////////////////////////////////////////////////////////////////////
//
// Filename: device.h
//
// Description: Device specific definitions and configuration
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef DEVICE_H
#define DEVICE_H

/* **********************   Program Configuration   *********************** */

// Have config before includes as included files rely on program config values

// #define USE_WDT

/* ***************************    Includes     **************************** */

#if defined(CCS_BUILD_CHAIN)
	#include "device_ccs.h"
#elif defined(XC8_BUILD_CHAIN)
	#include "device_xc8.h"
#else
	#error "Must select a build chain!"
#endif

#endif // DEVICE_H

// end of file.
//-------------------------------------------------------------------------
