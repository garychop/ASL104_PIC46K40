//////////////////////////////////////////////////////////////////////////////
//
// Filename: user_button_bsp.c
//
// Description: Handles bsp level user button access.
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
#include <stdbool.h>

// from project
#include "common.h"
#include "bsp.h"
#include "head_array.h"
#include "head_array_common.h"

// from local
#include "user_button_bsp.h"

/* ******************************   Macros   ****************************** */
// The USER PORT jack socket is tied to RB7 which is also PGD which is used
// for debugging and programming. If you want to debug the MODE PORT operation
// or the USER PORT operation, use the following conditional compilation.
// It will allow the MODE PORT jack socket to be used
// to debug either the MODE PORT operation or the USER PORT operation, but
// unfortunately, not both.
// This conditional compilation also provides the proper reading/mapping
// of the ports when the RELEASE build is performed.

#ifdef DEBUG
#define MODE_BTN_IS_ACTIVE()	(PORTBbits.RB0 == GPIO_LOW)
#define USER_BTN_IS_ACTIVE()    (false)
//#define MODE_BTN_IS_ACTIVE()	(false)
//#define USER_BTN_IS_ACTIVE()    (PORTBbits.RB0 == GPIO_LOW)
#define BT_LED_IS_ACTIVE()      (PORTCbits.RC5 == GPIO_LOW)
#else
#define MODE_BTN_IS_ACTIVE()	(PORTBbits.RB0 == GPIO_LOW)
#define USER_BTN_IS_ACTIVE()    (PORTBbits.RB7 == GPIO_LOW)
#define BT_LED_IS_ACTIVE()      (PORTCbits.RC5 == GPIO_LOW)
#endif

//--------------------------- Forward Declarations --------------------------

void SW1_Init (void);
void SW3_Init (void);
void SW6_Init (void);

/* *******************   Public Function Definitions   ******************** */

void USER_BTN_INIT()
{
#ifdef DEBUG
//    TRISBbits.TRISB0 = GPIO_BIT_INPUT;  // This must be 
    ANSELBbits.ANSELB0 = 0;
#else
    ANSELBbits.ANSELB7 = 0;
    WPUBbits.WPUB7 = 1;         // Enable the Weak Pull-up
#endif
}
void MODE_BTN_INIT()
{
#ifdef DEBUG
    ANSELBbits.ANSELB0 = 0;
#else
    ANSELBbits.ANSELB0 = 0;
    WPUBbits.WPUB0 = 1;         // Enable the Weak Pull-up
#endif
}

void BT_LED_INPUT_INIT()
{
    // RC5 is the Bluetooth LED input.
    TRISCbits.TRISC5 = GPIO_BIT_INPUT;
    ANSELCbits.ANSELC5 = 0;
}

//-------------------------------
// Function: ButtonBspInit
//
// Description: Initializes this module.
//
//-------------------------------
void ButtonBspInit(void)
{
    USER_BTN_INIT();
    MODE_BTN_INIT();        // Set up the Port for MODE Button input.
    BT_LED_INPUT_INIT();
    SW1_Init();
    SW3_Init();
    SW6_Init();
}

//-------------------------------
// Function: userButtonBspIsActive
//
// Description: Reads the digital input state of the user button
// Returns: TRUE if the button is pushed, else false.
//-------------------------------
bool userButtonBspIsActive(void)
{
	return USER_BTN_IS_ACTIVE();
}

//-------------------------------
// Function: ModeButtonBspIsActive
//
// Description: Reads the digital input state of the MODE button.
//  It also considers the 4-pad as a MODE button if SW1 is set to ON.
// Returns: TRUE if the button is pushed, else false.
//-------------------------------
bool ModeButtonBspIsActive(void)
{
    bool retVal;
    
	retVal = MODE_BTN_IS_ACTIVE();
    if (Is_SW1_ON())    // Is the SW1 DIP switch ON?
    {
        if (headArrayDigitalInputValue(HEAD_ARRAY_SENSOR_BACK)) // Is 4th back pad active?
            retVal = true;  // Then, let's call this a MODE switch active.
    }
    return (retVal);
}

//-------------------------------------------------------------------------
// Function: BT_LED_IsActive()
// Description: Returns true if the BT LED is ON, else false.
//-------------------------------------------------------------------------
bool BT_LED_IsActive(void)
{
    return BT_LED_IS_ACTIVE();
}

//-------------------------------------------------------------------------
// DIP Switch #1 is on D2
void SW1_Init (void)
{
    // This pin is always an Input. The following code will not compile.
    TRISDbits.TRISD2 = GPIO_BIT_INPUT;
    ANSELDbits.ANSELD2 = 0;     // Disable Analog to allow digital input to operate.
}

bool Is_SW1_ON (void)
{
    return (PORTDbits.RD2 == GPIO_LOW);
}

//-------------------------------------------------------------------------
// DIP Switch #6 is on D3
void SW6_Init()
{
    // This pin is always an Input. The following code will not compile.
    TRISDbits.TRISD3 = GPIO_BIT_INPUT;
    ANSELDbits.ANSELD3 = 0;     // Disable Analog to allow digital input to operate.
}

bool Is_SW6_ON(void)
{
    return (PORTDbits.RD3 == GPIO_LOW);
}

//-------------------------------------------------------------------------
// DIP Switch #3 is on C4
void SW3_Init (void)
{
    // This pin is always an Input. The following code will not compile.
    TRISCbits.TRISC4 = GPIO_BIT_INPUT;
    ANSELCbits.ANSELC4 = 0;     // Disable Analog to allow digital input to operate.
}

bool Is_SW3_ON (void)
{
    return (PORTCbits.RC4 == GPIO_LOW);
}

// end of file.
//-------------------------------------------------------------------------
