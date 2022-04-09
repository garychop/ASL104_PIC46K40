//////////////////////////////////////////////////////////////////////////////
//
// Filename: main.c
//
// Description: Main point for the program.
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

// from RTOS
#include "cocoos.h"

// from project
#include "bsp.h"
#include "test_gpio.h"
//#include "eeprom_app.h"
#include "head_array.h"
#include "beeper.h"
#include "user_button.h"
//#include "general_output_ctrl_app.h"
#include "app_common.h"
#include "Delay_Pot.h"

// Useful, but need all the space we can get.
#if 0
static void TestSetup(void);
#endif

/* *******************   Public Function Definitions   ******************** */

//------------------------------
// Function: main
//
// Description: Main entry point for the program.
//
//-------------------------------
void main(void)
{
    os_init();

    bspInitCore();
	testGpioInit();
	//GenOutCtrlApp_Init();
    DelayPot_INIT();
    
	beeperInit();
	userButtonInit();
	headArrayinit();

	AppCommonInit();
	
    // Kick off the RTOS. This will never return.
	// NOTE: Interrupts are enabled by this function
    os_start();
}

// end of file.
//-------------------------------------------------------------------------
