/*
	MIT License

	Copyright (c) 2018 Trevor Parsh

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
 */

/**
 ********************************************************************************************************
 *
 * @brief       Defines the BSP level control for the general output control module for the ASL110.
 * @file		general_output_ctrl_bsp.c
 * @author      tparsh
 * @ingroup		BSP
 *
 ********************************************************************************************************
 */

/**
 * @addtogroup BSP
 * @{
 */

/*
 ********************************************************************************************************
 *                                           INCLUDE FILES
 ********************************************************************************************************
 */
// NOTE: This must ALWAYS be the first include in a file.
#include "device.h"

/********************************************** System *************************************************/
#include <stdbool.h>
#include <assert.h>
/********************************************    User   ************************************************/
#include "bsp.h"
#include "common.h"
#include "general_output_ctrl_cfg.h"
#include "general_output_ctrl_bsp.h"

#if defined(GENERAL_OUTPUT_CTRL_MODULE_ENABLE)

#define PCB_REV4    // Make this PCB_REV3 is using an older to control the Blue BT LED1.
                    // Use REV3 cautiously because driving the signal may damage the BT Module.
/*
 **************************************************************************************************
 *                                                 DEFINES
 **************************************************************************************************
 */
/******************************************* Symbolic Constants **********************************/

#ifdef _18F46K40
    #define INTERNAL_SYS_ACTION_SIGNAL_IS_ACTIVE()	(LATCbits.LATC7 == GPIO_HIGH)
    #define INTERNAL_SYS_ACTION_SIGNAL_SET(active)	INLINE_EXPR(LATCbits.LATC7 = active ? GPIO_HIGH : GPIO_LOW)
    #define INTERNAL_SYS_ACTION_SIGNAL_TOGGLE()		INLINE_EXPR(INTERNAL_SYS_ACTION_SIGNAL_SET(!INTERNAL_SYS_ACTION_SIGNAL_IS_ACTIVE()))
    #define INTERNAL_SYS_ACTION_SIGNAL_INIT()		INLINE_EXPR(TRISCbits.TRISC7 = GPIO_BIT_OUTPUT; INTERNAL_SYS_ACTION_SIGNAL_SET(false))
    #define INTERNAL_SYS_ACTION_SIGNAL_DEINIT()		INLINE_EXPR(TRISCbits.TRISC7 = GPIO_BIT_INPUT; ANSELCbits.ANSELC7 = 0)

    // LEDs 0 and 1 are Charlieplexed
    #define LED0_SIGNAL_IS_ACTIVE()					((LATCbits.LATC0 == GPIO_LOW) && (LATCbits.LATC6 == GPIO_HIGH))
    #define LED0_SIGNAL_SET(active)					INLINE_EXPR(LATCbits.LATC0 = GPIO_LOW; LATCbits.LATC6 = active ? GPIO_HIGH : GPIO_LOW)
    #define LED0_SIGNAL_TOGGLE()					INLINE_EXPR(LED0_SIGNAL_SET(!LED0_SIGNAL_IS_ACTIVE()))

    #define LED1_SIGNAL_IS_ACTIVE()					((LATCbits.LATC0 == GPIO_HIGH) && (LATCbits.LATC6 == GPIO_LOW))
    #define LED1_SIGNAL_SET(active)					INLINE_EXPR(LATCbits.LATC0 = active ? GPIO_HIGH : GPIO_LOW; LATCbits.LATC6 = GPIO_LOW)
    #define LED1_SIGNAL_TOGGLE()					INLINE_EXPR(LED1_SIGNAL_SET(!LED1_SIGNAL_IS_ACTIVE()))

    #define LED0and1_SIGNAL_INIT()					INLINE_EXPR(TRISCbits.TRISC0 = GPIO_BIT_OUTPUT; TRISCbits.TRISC6 = GPIO_BIT_OUTPUT; \
                                                                ANSELCbits.ANSELC0 = 0; ANSELCbits.ANSELC6 = 0; LED0_SIGNAL_SET(false))
    #define LED0and1_SIGNAL_DEINIT()				INLINE_EXPR(TRISCbits.TRISC0 = GPIO_BIT_INPUT; TRISCbits.TRISC6 = GPIO_BIT_INPUT)

#ifdef PCB_REV3
    // Bluetooth BLUE LED control macros
    #define BLUETOOTH_LED_SIGNAL_IS_ACTIVE()        (LATCbits.LATC4 == GPIO_HIGH)
    #define BLUETOOTH_LED_SIGNAL_SET(active)        INLINE_EXPR(LATCbits.LATC4 = active ? GPIO_HIGH : GPIO_LOW)
    #define BLUETOOTH_LED_SIGNAL_TOGGLE()           INLINE_EXPR(BLUETOOTH_LED_SIGNAL_SET(!BLUETOOTH_LED_SIGNAL_IS_ACTIVE()))
    #define BLUETOOTH_LED_SIGNAL_INIT()             INLINE_EXPR(TRISCbits.TRISC4 = GPIO_BIT_OUTPUT; BLUETOOTH_LED_SIGNAL_SET(false))
    #define BLUETOOTH_LED_SIGNAL_DEINIT()   		INLINE_EXPR(TRISCbits.TRISC4 = GPIO_BIT_INPUT; ANSELCbits.ANSELC4 = 0)
#else // Must be Rev 4 and later.
    #define BLUETOOTH_LED_SIGNAL_IS_ACTIVE()        (LATAbits.LATA4 == GPIO_HIGH)
    #define BLUETOOTH_LED_SIGNAL_SET(active)        INLINE_EXPR(LATAbits.LATA4 = active ? GPIO_HIGH : GPIO_LOW)
    #define BLUETOOTH_LED_SIGNAL_TOGGLE()           INLINE_EXPR(BLUETOOTH_LED_SIGNAL_SET(!BLUETOOTH_LED_SIGNAL_IS_ACTIVE()))
    #define BLUETOOTH_LED_SIGNAL_INIT()             INLINE_EXPR(TRISAbits.TRISA4 = GPIO_BIT_OUTPUT; BLUETOOTH_LED_SIGNAL_SET(false))
    #define BLUETOOTH_LED_SIGNAL_DEINIT()   		INLINE_EXPR(TRISAbits.TRISA4 = GPIO_BIT_INPUT; ANSELAbits.ANSELA4 = 0)
#endif // PCB_REV3

#else   // Must be 18LF4550 (ASL104)
//    #define LED1_FWD_SIGNAL_IS_ACTIVE()					((LATCbits.LATC0 == GPIO_LOW) && (LATCbits.LATC6 == GPIO_HIGH))
//    #define LED1_FWD_SIGNAL_SET(active)					INLINE_EXPR(LATCbits.LATC0 = GPIO_LOW; LATCbits.LATC6 = active ? GPIO_HIGH : GPIO_LOW)
//    #define LED1_FWD_SIGNAL_TOGGLE()					INLINE_EXPR(LED0_SIGNAL_SET(!LED0_SIGNAL_IS_ACTIVE()))
//
//    #define LED2_LEFT_SIGNAL_IS_ACTIVE()				((LATCbits.LATC0 == GPIO_HIGH) && (LATCbits.LATC6 == GPIO_LOW))
//    #define LED2_LEFT_SIGNAL_SET(active)				INLINE_EXPR(LATCbits.LATC0 = active ? GPIO_HIGH : GPIO_LOW; LATCbits.LATC6 = GPIO_LOW)
//    #define LED2_LEFT_SIGNAL_TOGGLE()					INLINE_EXPR(LED1_SIGNAL_SET(!LED1_SIGNAL_IS_ACTIVE()))


    // The Bluetooth LED is supported on the ASL104.
    // Bluetooth BLUE LED control macros
//    #define BLUETOOTH_LED_SIGNAL_IS_ACTIVE()        (LATCbits.LATC2 == GPIO_HIGH)
//    #define BLUETOOTH_LED_SIGNAL_SET(active)        INLINE_EXPR(LATCbits.LATC2 = active ? GPIO_HIGH : GPIO_LOW)
//    #define BLUETOOTH_LED_SIGNAL_TOGGLE()           INLINE_EXPR(BLUETOOTH_LED_SIGNAL_SET(!BLUETOOTH_LED_SIGNAL_IS_ACTIVE()))
//    #define BLUETOOTH_LED_SIGNAL_INIT()             INLINE_EXPR(TRISCbits.TRISC2 = GPIO_BIT_OUTPUT; BLUETOOTH_LED_SIGNAL_SET(false))
//    #define BLUETOOTH_LED_SIGNAL_DEINIT()   		INLINE_EXPR(TRISCbits.TRISC2 = GPIO_BIT_INPUT;)

#endif



/*
 ********************************************************************************************************
 *                                         PUBLIC FUNCTIONS DEFINITIONS
 ********************************************************************************************************
 */
void GenOutCtrlBsp_INIT(void)
{
    GenOutCtrlBsp_Enable (GEN_OUT_CTRL_ID_FORWARD_PAD_LED);
    GenOutCtrlBsp_Enable (GEN_OUT_CTRL_ID_LEFT_PAD_LED);
    GenOutCtrlBsp_Enable (GEN_OUT_CTRL_ID_RIGHT_PAD_LED);
    GenOutCtrlBsp_Enable (GEN_OUT_CTRL_ID_REVERSE_PAD_LED);
    GenOutCtrlBsp_Enable (GEN_OUT_CTRL_ID_POWER_LED);
    GenOutCtrlBsp_Enable (GEN_OUT_CTRL_ID_FORWARD_DEMAND);
    GenOutCtrlBsp_Enable (GEN_OUT_CTRL_ID_REVERSE_DEMAND);
    GenOutCtrlBsp_Enable (GEN_OUT_CTRL_ID_LEFT_DEMAND);
    GenOutCtrlBsp_Enable (GEN_OUT_CTRL_ID_RIGHT_DEMAND);
    GenOutCtrlBsp_Enable (GEN_OUT_CTRL_ID_RESET_OUT);


}

/**
 * Sets up GPIO to control the general output control item.
 *
 * @param item_id ID of the output control item that is to be modified
 *
 * @return true if everything was successful, false if something failed
 */
bool GenOutCtrlBsp_Enable(GenOutCtrlId_t item_id)
{
	bool ret_val = true;

	switch (item_id)
	{
        case GEN_OUT_CTRL_ID_POWER_LED:             // Power LED
            TRISEbits.TRISE0 = GPIO_BIT_OUTPUT;     // LED1 control
            LATEbits.LATE0 = GPIO_HIGH;             // This turns the LED off
            break;
            
        case GEN_OUT_CTRL_ID_FORWARD_PAD_LED:       // Forward Pad Feedback LED
            TRISEbits.TRISE1 = GPIO_BIT_OUTPUT;     // LED #5 control.
            LATEbits.LATE1 = GPIO_HIGH;
            break;

        case GEN_OUT_CTRL_ID_LEFT_PAD_LED:          // Left Pad Feedback LED
            TRISCbits.TRISC0 = GPIO_BIT_OUTPUT;     // LED2 control
            LATCbits.LATC0 = GPIO_HIGH;             // This turns the LED off
            break;

        case GEN_OUT_CTRL_ID_RIGHT_PAD_LED:         // Right Pad Feedback LED
            TRISEbits.TRISE2 = GPIO_BIT_OUTPUT;     // LED3 control
            LATEbits.LATE2 = GPIO_HIGH;             // This turns the LED off
            break;
            
        case GEN_OUT_CTRL_ID_REVERSE_PAD_LED:       // Reverse Pad Feedback LED
            TRISAbits.TRISA1 = GPIO_BIT_OUTPUT;     // LED4 control
            LATAbits.LATA1 = GPIO_HIGH;             // This turns the LED off
            break;
            
        case GEN_OUT_CTRL_ID_FORWARD_DEMAND:        // Forward Drive demand pin on 9-pin connector
            TRISAbits.TRISA4 = GPIO_BIT_OUTPUT;     // Set bit A4 for output control
            LATAbits.LATA4 = GPIO_LOW;              // Sets low for no forward demand
            break;
            
        case GEN_OUT_CTRL_ID_REVERSE_DEMAND:        // Reverse Drive demand pin on 9-pin connector
            TRISDbits.TRISD5 = GPIO_BIT_OUTPUT;     // Set bit D5 for output control
            LATDbits.LATD4 = GPIO_LOW;              // Set low for no reverse demand
            break;
            
        case GEN_OUT_CTRL_ID_LEFT_DEMAND:           // Left Drive demand pin on 9-pin connector
            TRISDbits.TRISD6 = GPIO_BIT_OUTPUT;     // Set bit D6 for output control
            LATDbits.LATD6 = GPIO_LOW;              // Set low for no Left demand
            break;
            
        case GEN_OUT_CTRL_ID_RIGHT_DEMAND:          // Right Drive demand pin on 9-pin connector
            TRISAbits.TRISA2 = GPIO_BIT_OUTPUT;     // Set bit A2 for output control
            LATAbits.LATA2 = GPIO_LOW;              // Set low for no Right demand
            break;
            
        case GEN_OUT_CTRL_ID_RESET_OUT:             // Reset Pin on 9-pin connector
            TRISAbits.TRISA3 = GPIO_BIT_OUTPUT;     // Set bit A3 for output control
            LATAbits.LATA3 = GPIO_LOW;              // Set low for no demand
            break;
            
//    	case GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION:
//			// Signal line used by this device to let the system know things like "resetting" and "user button short press"
//			INTERNAL_SYS_ACTION_SIGNAL_INIT();
//			break;
//            
        // The Bluetooth LED is being controlled by either enabling the port as an
        // output or as an input. This operation is being controlled elsewhere.
        // This function is called by a function that initializes all I/O.
        // We will initialize the pin as an output here to turn off the Blue
        // LED but will have to turn it on if the "Using Bluetooth" feature
        // is turned on.
//        case GEN_OUT_CTRL_ID_BT_LED:
//            BLUETOOTH_LED_SIGNAL_INIT(); 
//            break;

	   	case GEN_OUT_CTRL_ID_MAX:
	   	default:
		   	ret_val = false;
		   	return false;
	}
	
	assert(ret_val);
	return ret_val;
}

/**
 * Disable general output control item.
 *
 * @param item_id ID of the output control item that is to be modified
 *
 * @return true if everything was successful, false if something failed
 */
bool GenOutCtrlBsp_Disable(GenOutCtrlId_t item_id)
{
	bool ret_val = true;

	switch (item_id)
	{
        case GEN_OUT_CTRL_ID_POWER_LED:             // Power LED
            TRISEbits.TRISE0 = GPIO_BIT_INPUT;      // LED1 control
            break;
            
        case GEN_OUT_CTRL_ID_FORWARD_PAD_LED:       // Forward Pad Feedback LED
            TRISEbits.TRISE1 = GPIO_BIT_INPUT;      // LED #5 control.
            break;

        case GEN_OUT_CTRL_ID_LEFT_PAD_LED:          // Left Pad Feedback LED
            TRISCbits.TRISC0 = GPIO_BIT_INPUT;      // LED2 control
            break;

        case GEN_OUT_CTRL_ID_RIGHT_PAD_LED:         // Right Pad Feedback LED
            TRISEbits.TRISE2 = GPIO_BIT_INPUT;      // LED3 control
            break;
            
        case GEN_OUT_CTRL_ID_REVERSE_PAD_LED:       // Reverse Pad Feedback LED
            TRISAbits.TRISA1 = GPIO_BIT_INPUT;      // LED4 control
            break;
            
        case GEN_OUT_CTRL_ID_FORWARD_DEMAND:        // Forward Drive demand pin on 9-pin connector
            TRISAbits.TRISA4 = GPIO_BIT_INPUT;      // Set for input to disable
            break;
            
        case GEN_OUT_CTRL_ID_REVERSE_DEMAND:        // Reverse Drive demand pin on 9-pin connector
            TRISDbits.TRISD5 = GPIO_BIT_INPUT;      // Set for input to disable
            break;
            
        case GEN_OUT_CTRL_ID_LEFT_DEMAND:           // Left Drive demand pin on 9-pin connector
            TRISDbits.TRISD6 = GPIO_BIT_INPUT;      // Set for input to disable
            break;
            
        case GEN_OUT_CTRL_ID_RIGHT_DEMAND:          // Right Drive demand pin on 9-pin connector
            TRISAbits.TRISA2 = GPIO_BIT_INPUT;      // Set for input to disable
            break;
            
        case GEN_OUT_CTRL_ID_RESET_OUT:             // Reset Pin on 9-pin connector
            TRISAbits.TRISA3 = GPIO_BIT_INPUT;      // Set bit A3 for input to disable
            break;

            //    	case GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION:
//			// Signal line used by this device to let the system know things like "resetting" and "user button short press"
//			INTERNAL_SYS_ACTION_SIGNAL_DEINIT();
//			break;

//        case GEN_OUT_CTRL_ID_BT_LED:
//            BLUETOOTH_LED_SIGNAL_DEINIT();
//            break;
            
	   	case GEN_OUT_CTRL_ID_MAX:
	   	default:
		   	ret_val = false;
		   	return false;
	}
//	
	assert(ret_val);
	return ret_val;
}

/**
 * This function is used to put a general output control item into an active state.
 *
 * @param item_id ID of the output control item that is to be modified
 *
 * @return true if everything was successful, false if something failed
 */
bool GenOutCtrlBsp_SetActive(GenOutCtrlId_t item_id)
{
	bool ret_val = true;
    
	switch (item_id)
	{
        case GEN_OUT_CTRL_ID_POWER_LED:            // Power LED
            LATEbits.LATE0 = GPIO_LOW;             // This turns the LED #1 on
            break;

        case GEN_OUT_CTRL_ID_FORWARD_PAD_LED:       // Forward Pad Feedback LED
            LATEbits.LATE1 = GPIO_LOW;             // This turns the LED #5 on
            break;
            
        case GEN_OUT_CTRL_ID_LEFT_PAD_LED:          // Left Pad Feedback LED
            LATCbits.LATC0 = GPIO_LOW;             // This turns the LED on
            break;

        case GEN_OUT_CTRL_ID_RIGHT_PAD_LED:         // Right Pad Feedback LED
            LATEbits.LATE2 = GPIO_LOW;             // This turns the LED on
            break;
            
        case GEN_OUT_CTRL_ID_REVERSE_PAD_LED:       // Reverse Pad Feedback LED
            LATAbits.LATA1 = GPIO_LOW;              // This turns the LED on
            break;
            
        case GEN_OUT_CTRL_ID_FORWARD_DEMAND:        // Forward Drive demand pin on 9-pin connector
            LATAbits.LATA4 = GPIO_HIGH;             // Sets high to drive forward
            break;
            
        case GEN_OUT_CTRL_ID_REVERSE_DEMAND:        // Reverse Drive demand pin on 9-pin connector
            LATDbits.LATD5 = GPIO_HIGH;             // Sets high to drive in reverse
            break;
            
        case GEN_OUT_CTRL_ID_LEFT_DEMAND:           // Left Drive demand pin on 9-pin connector
            LATDbits.LATD6 = GPIO_HIGH;             // Sets high to drive left
            break;
            
        case GEN_OUT_CTRL_ID_RIGHT_DEMAND:          // Right Drive demand pin on 9-pin connector
            LATAbits.LATA2 = GPIO_HIGH;             // Sets high to drive right
            break;
            
        case GEN_OUT_CTRL_ID_RESET_OUT:             // Reset Pin on 9-pin connector
            LATAbits.LATA3 = GPIO_HIGH;             // Sets high to make Reset active
            break;
            
//    	case GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION:
//			// Signal line used by this device to let the system know things like "resetting" and "user button short press"
//			INTERNAL_SYS_ACTION_SIGNAL_INIT();
//			break;
//            
        // The Bluetooth LED is being controlled by either enabling the port as an
        // output or as an input. This operation is being controlled elsewhere.
        // This function is called by a function that initializes all I/O.
        // We will initialize the pin as an output here to turn off the Blue
        // LED but will have to turn it on if the "Using Bluetooth" feature
        // is turned on.
//        case GEN_OUT_CTRL_ID_BT_LED:
//            BLUETOOTH_LED_SIGNAL_INIT(); 
//            break;

	   	case GEN_OUT_CTRL_ID_MAX:
	   	default:
		   	ret_val = false;
		   	return false;
	}
	assert(ret_val);
	return ret_val;
}

/**
 * This function is used to put a general output control item into an inactive state.
 *
 * @param item_id ID of the output control item that is to be modified
 *
 * @return true if everything was successful, false if something failed
 */
bool GenOutCtrlBsp_SetInactive(GenOutCtrlId_t item_id)
{
	bool ret_val = true;

	switch (item_id)
	{
        case GEN_OUT_CTRL_ID_POWER_LED:            // Power LED
            LATEbits.LATE0 = GPIO_HIGH;             // This turns the LED off #1
            break;

        case GEN_OUT_CTRL_ID_FORWARD_PAD_LED:       // Forward Pad Feedback LED
            LATEbits.LATE1 = GPIO_HIGH;             // This turns the LED #5 off
            break;

        case GEN_OUT_CTRL_ID_LEFT_PAD_LED:          // Left Pad Feedback LED
            LATCbits.LATC0 = GPIO_HIGH;             // This turns the LED off
            break;

        case GEN_OUT_CTRL_ID_RIGHT_PAD_LED:         // Right Pad Feedback LED
            LATEbits.LATE2 = GPIO_HIGH;             // This turns the LED off
            break;
            
        case GEN_OUT_CTRL_ID_REVERSE_PAD_LED:       // Reverse Pad Feedback LED
            LATAbits.LATA1 = GPIO_HIGH;             // This turns the LED off
            break;
            
        case GEN_OUT_CTRL_ID_FORWARD_DEMAND:        // Forward Drive demand pin on 9-pin connector
            LATAbits.LATA4 = GPIO_LOW;              // This eliminates the Forward Drive demand
            break;
            
        case GEN_OUT_CTRL_ID_REVERSE_DEMAND:        // Reverse Drive demand pin on 9-pin connector
            LATDbits.LATD5 = GPIO_LOW;              // This eliminates the Reverse Drive demand
            break;
            
        case GEN_OUT_CTRL_ID_LEFT_DEMAND:           // Left Drive demand pin on 9-pin connector
            LATDbits.LATD6 = GPIO_LOW;              // This eliminates the Left Drive demand
            break;
            
        case GEN_OUT_CTRL_ID_RIGHT_DEMAND:          // Right Drive demand pin on 9-pin connector
            LATAbits.LATA2 = GPIO_LOW;              // This eliminates the RightDrive demand
            break;
            
        case GEN_OUT_CTRL_ID_RESET_OUT:             // Reset Pin on 9-pin connector
            LATAbits.LATA3 = GPIO_LOW;             // Sets high to make Reset active
            break;
            
//    	case GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION:
//			// Signal line used by this device to let the system know things like "resetting" and "user button short press"
//			INTERNAL_SYS_ACTION_SIGNAL_INIT();
//			break;
//            
        // The Bluetooth LED is being controlled by either enabling the port as an
        // output or as an input. This operation is being controlled elsewhere.
        // This function is called by a function that initializes all I/O.
        // We will initialize the pin as an output here to turn off the Blue
        // LED but will have to turn it on if the "Using Bluetooth" feature
        // is turned on.
//        case GEN_OUT_CTRL_ID_BT_LED:
//            BLUETOOTH_LED_SIGNAL_DEINIT(); 
//            break;

	   	case GEN_OUT_CTRL_ID_MAX:
	   	default:
		   	ret_val = false;
		   	return false;
	}

	assert(ret_val);
	return ret_val;
}

/**
 * This function is used to toggle a general output control item's state.
 *
 * @param item_id ID of the output control item that is to be modified
 *
 * @return true if everything was successful, false if something failed
 */
bool GenOutCtrlBsp_Toggle(GenOutCtrlId_t item_id)
{
	bool ret_val = true;

	switch (item_id)
	{
        case GEN_OUT_CTRL_ID_POWER_LED:            // Power LED
            LATEbits.LATE0 = (LATEbits.LATE0 == GPIO_HIGH ? GPIO_LOW : GPIO_HIGH);             // This turns the LED #1 off
            break;
            
        case GEN_OUT_CTRL_ID_FORWARD_PAD_LED:       // Forward Pad Feedback LED
            LATEbits.LATE1 = (LATEbits.LATE1 == GPIO_HIGH ? GPIO_LOW : GPIO_HIGH);             // This turns the LED #5 off
            break;

        case GEN_OUT_CTRL_ID_LEFT_PAD_LED:          // Left Pad Feedback LED
            LATCbits.LATC0 = (LATCbits.LATC0 == GPIO_HIGH ? GPIO_LOW : GPIO_HIGH);             // This turns the LED #2 off
            break;

        case GEN_OUT_CTRL_ID_RIGHT_PAD_LED:         // Right Pad Feedback LED
            LATEbits.LATE2 = (LATEbits.LATE2 == GPIO_HIGH ? GPIO_LOW : GPIO_HIGH);             // This turns the LED #3 off
            break;
            
        case GEN_OUT_CTRL_ID_REVERSE_PAD_LED:       // Reverse Pad Feedback LED
            LATAbits.LATA1 = (LATAbits.LATA1 == GPIO_HIGH ? GPIO_LOW : GPIO_HIGH);             // This turns the LED #4 off
            break;
            
//    	case GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION:
//			// Signal line used by this device to let the system know things like "resetting" and "user button short press"
//			INTERNAL_SYS_ACTION_SIGNAL_INIT();
//			break;
//            
        // The Bluetooth LED is being controlled by either enabling the port as an
        // output or as an input. This operation is being controlled elsewhere.
        // This function is called by a function that initializes all I/O.
        // We will initialize the pin as an output here to turn off the Blue
        // LED but will have to turn it on if the "Using Bluetooth" feature
        // is turned on.
//        case GEN_OUT_CTRL_ID_BT_LED:
//            BLUETOOTH_LED_SIGNAL_INIT(); 
//            break;

	   	case GEN_OUT_CTRL_ID_MAX:
	   	default:
		   	ret_val = false;
		   	return false;
	}
	
	assert(ret_val);
	return ret_val;
}

// End of Doxygen grouping
/** @} */

#endif  // #if defined(GENERAL_OUTPUT_CTRL_MODULE_ENABLE)

/**********************************************************************************************************************
---          End of File          ------          End of File          ------          End of File          ---
**********************************************************************************************************************/
