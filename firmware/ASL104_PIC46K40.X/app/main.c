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

#include <xc.h>

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
#include "eeprom_app.h"
#include "head_array.h"
#include "beeper.h"
#include "user_button.h"
//#include "general_output_ctrl_app.h"
#include "general_output_ctrl_bsp.h"
#include "ha_hhp_interface_app.h"
#include "app_common.h"
#include "MainState.h"
#include "beeper_bsp.h"
#include "inc/eFix_Communication.h"
#include "Delay_Pot.h"

#ifdef DEBUG
// We can put stuff when debug build.
#endif

// Useful, but need all the space we can get.
#if 0
static void TestSetup(void);
#endif

// #define DEV_TEST_MIRROR_PADS_ON_BLUETOOTH_OUTPUTS
#if defined(DEV_TEST_MIRROR_PADS_ON_BLUETOOTH_OUTPUTS)
static void DEV_TEST_MirrorPadsOnBluetoothOutputs(void);
#endif

/* *******************   Public Function Definitions   ******************** */

//------------------------------
// Function: main
//
// Description: Main entry point for the program.
//
//-------------------------------
int main(void)
{
#if defined(DEV_TEST_MIRROR_PADS_ON_BLUETOOTH_OUTPUTS)
    DEV_TEST_MirrorPadsOnBluetoothOutputs();
#endif

    os_init();

    bspInitCore();
	testGpioInit();
	//GenOutCtrlApp_Init();
    GenOutCtrlBsp_INIT();
    DelayPot_INIT();
    
	// Other high level modules depend on EEPROM being initialized, therefore it must be initialized here.
#ifdef ASL110
	bool eeprom_initialized_before = eepromAppInit();
#endif 
	beeperInit();
	userButtonInit();
	headArrayinit();
    
    //eFix_Communincation_Initialize();
    MainTaskInitialise();
    
//	haHhpApp_Init();

	// This must come after beeperInit() otherwise the pattern request will be ignored by the beeper module.
//	if (!eeprom_initialized_before)
//	{
//		(void)beeperBeep(BEEPER_PATTERN_EEPROM_NOT_INIT_ON_BOOT);
//	}

	AppCommonInit();
	
	// Enable to test system configurations without external control.
#if 0
	TestSetup();
#endif

    // Kick off the RTOS. This will never return.
	// NOTE: Interrupts are enabled by this function
    os_start();
    
    return (0);
}

// Useful, but need all the space we can get.
#if 0
static void TestSetup(void)
{
	eepromEnumSet(EEPROM_STORED_ITEM_LEFT_PAD_INPUT_TYPE, (EepromStoredEnumType_t)HEAD_ARR_INPUT_DIGITAL);
	eepromEnumSet(EEPROM_STORED_ITEM_RIGHT_PAD_INPUT_TYPE, (EepromStoredEnumType_t)HEAD_ARR_INPUT_DIGITAL);
	eepromEnumSet(EEPROM_STORED_ITEM_CTR_PAD_INPUT_TYPE, (EepromStoredEnumType_t)HEAD_ARR_INPUT_DIGITAL);
//	eepromEnumSet(EEPROM_STORED_ITEM_CURRENT_ACTIVE_FEATURE, (EepromStoredEnumType_t)FUNC_FEATURE_POWER_ON_OFF);
}
#endif

#if defined(DEV_TEST_MIRROR_PADS_ON_BLUETOOTH_OUTPUTS)
/**
 * More or less does what the name says.
 */
static void DEV_TEST_MirrorPadsOnBluetoothOutputs(void)
{
#define TEST_LED_ACTIVE         GPIO_LOW
#define TEST_LED_INACTIVE       GPIO_HIGH

#define TEST_BT_OUTPUT_ACTIVE   GPIO_LOW
#define TEST_BT_OUTPUT_INACTIVE GPIO_HIGH

#define TEST_PAD_ACTIVE         GPIO_LOW
#define TEST_PAD_INACTIVE       GPIO_HIGH

    TRISEbits.TRISE0 = GPIO_BIT_OUTPUT;         // LED1 control
    TRISEbits.TRISE1 = GPIO_BIT_OUTPUT;         // LED2 control
    TRISEbits.TRISE2 = GPIO_BIT_OUTPUT;         // LED3 control
    TRISCbits.TRISC0 = GPIO_BIT_OUTPUT;         // LED4 control
    TRISAbits.TRISA1 = GPIO_BIT_OUTPUT;         // LED5 control

    // Exercise all the LEDs
    LATEbits.LATE0 = TEST_LED_ACTIVE;
    LATEbits.LATE1 = TEST_LED_ACTIVE;
    LATEbits.LATE2 = TEST_LED_ACTIVE;
    LATCbits.LATC0 = TEST_LED_ACTIVE;
    LATAbits.LATA1 = TEST_LED_ACTIVE;
    LATEbits.LATE0 = TEST_LED_INACTIVE;
    LATEbits.LATE1 = TEST_LED_INACTIVE;
    LATEbits.LATE2 = TEST_LED_INACTIVE;
    LATCbits.LATC0 = TEST_LED_INACTIVE;
    LATAbits.LATA1 = TEST_LED_INACTIVE;

    
    TRISBbits.TRISB1 = GPIO_BIT_INPUT;         // D1 USB/pad input
    TRISBbits.TRISB2 = GPIO_BIT_INPUT;         // D2 USB/pad input
    TRISBbits.TRISB3 = GPIO_BIT_INPUT;         // D3 USB/pad input
    TRISBbits.TRISB4 = GPIO_BIT_INPUT;         // D4 USB/pad input

    
    TRISDbits.TRISD5 = GPIO_BIT_OUTPUT;         // BT1: Bluetooth Pad State
    TRISDbits.TRISD7 = GPIO_BIT_OUTPUT;         // BT2: Bluetooth Pad State
    TRISDbits.TRISD1 = GPIO_BIT_OUTPUT;         // BT3: Bluetooth Pad State
    TRISDbits.TRISD0 = GPIO_BIT_OUTPUT;         // BT4: Bluetooth Pad State

    while (1)
    {
        // D1 USB/pad input
        if (PORTBbits.RB1 == TEST_PAD_ACTIVE)
        {
            // BT1: Bluetooth Pad State
            LATDbits.LATD5 = TEST_BT_OUTPUT_ACTIVE;
            LATEbits.LATE0 = TEST_LED_ACTIVE;
        }
        else
        {
            // BT1: Bluetooth Pad State
            LATDbits.LATD5 = TEST_PAD_INACTIVE;
            LATEbits.LATE0 = TEST_LED_INACTIVE;
        }
        
        // D2 USB/pad input
        if (PORTBbits.RB2 == TEST_PAD_ACTIVE)
        {
            // BT2: Bluetooth Pad State
            LATDbits.LATD7 = TEST_BT_OUTPUT_ACTIVE;
            LATEbits.LATE1 = TEST_LED_ACTIVE;
        }
        else
        {
            // BT2: Bluetooth Pad State
            LATDbits.LATD7 = TEST_PAD_INACTIVE;
            LATEbits.LATE1 = TEST_LED_INACTIVE;
        }
        
        // D3 USB/pad input
        if (PORTBbits.RB3 == TEST_PAD_ACTIVE)
        {
            // BT3: Bluetooth Pad State
            LATDbits.LATD1 = TEST_BT_OUTPUT_ACTIVE;
            LATEbits.LATE2 = TEST_LED_ACTIVE;
        }
        else
        {
            // BT3: Bluetooth Pad State
            LATDbits.LATD1 = TEST_PAD_INACTIVE;
            LATEbits.LATE2 = TEST_LED_INACTIVE;
        }
        
        // D4 USB/pad input
        if (PORTBbits.RB4 == TEST_PAD_ACTIVE)
        {
            // BT4: Bluetooth Pad State
            LATDbits.LATD0 = TEST_BT_OUTPUT_ACTIVE;
            LATCbits.LATC0 = TEST_LED_INACTIVE;
        }
        else
        {
            // BT4: Bluetooth Pad State
            LATDbits.LATD0 = TEST_PAD_INACTIVE;
            LATCbits.LATC0 = TEST_LED_ACTIVE;
        }
    }
}
#endif

// end of file.
//-------------------------------------------------------------------------
