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
#include "app_common.h"
#include "general_output_ctrl_cfg.h"
#include "general_output_ctrl_bsp.h"

#if defined(GENERAL_OUTPUT_CTRL_MODULE_ENABLE)

#define BT_PAD_MIRROR_ACTIVE_STATE		GPIO_LOW
#define BT_PAD_MIRROR_INACTIVE_STATE	GPIO_HIGH

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

    #define BT_LEFT_PAD_IS_ACTIVE()                 (LATEbits.LE1 == BT_PAD_MIRROR_ACTIVE_STATE)
    #define BT_LEFT_PAD_SET(active)                 INLINE_EXPR(LATEbits.LE1 = active ? BT_PAD_MIRROR_ACTIVE_STATE : BT_PAD_MIRROR_INACTIVE_STATE)
    #define BT_LEFT_PAD_TOGGLE()                    INLINE_EXPR(BT_LEFT_PAD_SET(BT_LEFT_PAD_IS_ACTIVE() ? false : true))
    #define BT_LEFT_PAD_INIT()                      INLINE_EXPR(TRISEbits.TRISE1 = GPIO_BIT_OUTPUT; ANSELEbits.ANSELE1 = 0; BT_LEFT_PAD_SET(false))
    #define BT_LEFT_PAD_DEINIT()                    INLINE_EXPR(TRISEbits.TRISE1 = GPIO_BIT_INPUT)

    #define BT_RIGHT_PAD_IS_ACTIVE()                (LATCbits.LC1 == BT_PAD_MIRROR_ACTIVE_STATE)
    #define BT_RIGHT_PAD_SET(active)                INLINE_EXPR(LATCbits.LC1 = active ? BT_PAD_MIRROR_ACTIVE_STATE : BT_PAD_MIRROR_INACTIVE_STATE)
    #define BT_RIGHT_PAD_TOGGLE()                   INLINE_EXPR(BT_RIGHT_PAD_SET(BT_RIGHT_PAD_IS_ACTIVE() ? false : true))
    #define BT_RIGHT_PAD_INIT()                     INLINE_EXPR(TRISCbits.TRISC1 = GPIO_BIT_OUTPUT; ANSELCbits.ANSELC1 = 0; BT_RIGHT_PAD_SET(false))
    #define BT_RIGHT_PAD_DEINIT()                   INLINE_EXPR(TRISCbits.TRISC1 = GPIO_BIT_INPUT)

    #define BT_CTR_PAD_IS_ACTIVE()                  (LATDbits.LD3 == BT_PAD_MIRROR_ACTIVE_STATE)
    #define BT_CTR_PAD_SET(active)                  INLINE_EXPR(LATDbits.LD3 = active ? BT_PAD_MIRROR_ACTIVE_STATE : BT_PAD_MIRROR_INACTIVE_STATE)
    #define BT_CTR_PAD_TOGGLE()                     INLINE_EXPR(BT_CTR_PAD_SET(BT_CTR_PAD_IS_ACTIVE() ? false : true))
    #define BT_CTR_PAD_INIT()                       INLINE_EXPR(TRISDbits.TRISD3 = GPIO_BIT_OUTPUT; ANSELDbits.ANSELD3 = 0; BT_CTR_PAD_SET(false))
    #define BT_CTR_PAD_DEINIT()                     INLINE_EXPR(TRISDbits.TRISD3 = GPIO_BIT_INPUT)

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

#else
    #define INTERNAL_SYS_ACTION_SIGNAL_IS_ACTIVE()	(LATCbits.LATC7 == GPIO_HIGH)
    #define INTERNAL_SYS_ACTION_SIGNAL_SET(active)	INLINE_EXPR(LATCbits.LATC7 = active ? GPIO_HIGH : GPIO_LOW)
    #define INTERNAL_SYS_ACTION_SIGNAL_TOGGLE()		INLINE_EXPR(INTERNAL_SYS_ACTION_SIGNAL_SET(!INTERNAL_SYS_ACTION_SIGNAL_IS_ACTIVE()))
    #define INTERNAL_SYS_ACTION_SIGNAL_INIT()		INLINE_EXPR(TRISCbits.TRISC7 = GPIO_BIT_OUTPUT; INTERNAL_SYS_ACTION_SIGNAL_SET(false))
    #define INTERNAL_SYS_ACTION_SIGNAL_DEINIT()		INLINE_EXPR(TRISCbits.TRISC7 = GPIO_BIT_INPUT)

    // LEDs 0 and 1 are Charlieplexed
    #define LED0_SIGNAL_IS_ACTIVE()					((LATCbits.LATC0 == GPIO_LOW) && (LATCbits.LATC6 == GPIO_HIGH))
    #define LED0_SIGNAL_SET(active)					INLINE_EXPR(LATCbits.LATC0 = GPIO_LOW; LATCbits.LATC6 = active ? GPIO_HIGH : GPIO_LOW)
    #define LED0_SIGNAL_TOGGLE()					INLINE_EXPR(LED0_SIGNAL_SET(!LED0_SIGNAL_IS_ACTIVE()))

    #define LED1_SIGNAL_IS_ACTIVE()					((LATCbits.LATC0 == GPIO_HIGH) && (LATCbits.LATC6 == GPIO_LOW))
    #define LED1_SIGNAL_SET(active)					INLINE_EXPR(LATCbits.LATC0 = active ? GPIO_HIGH : GPIO_LOW; LATCbits.LATC6 = GPIO_LOW)
    #define LED1_SIGNAL_TOGGLE()					INLINE_EXPR(LED1_SIGNAL_SET(!LED1_SIGNAL_IS_ACTIVE()))

    #define LED0and1_SIGNAL_INIT()					INLINE_EXPR(TRISCbits.TRISC0 = GPIO_BIT_OUTPUT; TRISCbits.TRISC6 = GPIO_BIT_OUTPUT; LED0_SIGNAL_SET(false))
    #define LED0and1_SIGNAL_DEINIT()				INLINE_EXPR(TRISCbits.TRISC0 = GPIO_BIT_INPUT; TRISCbits.TRISC6 = GPIO_BIT_INPUT)
#endif

/*
 ********************************************************************************************************
 *                                         PUBLIC FUNCTIONS DEFINITIONS
 ********************************************************************************************************
 */

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
		case GEN_OUT_CTRL_ID_LED0:
			// Primary feedback LED
			LED0and1_SIGNAL_INIT();
			break;
			
    	case GEN_OUT_CTRL_ID_LED1:
			// Primary feedback LED
			LED0and1_SIGNAL_INIT();
			break;

    	case GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION:
			// Signal line used by this device to let the system know things like "resetting" and "user button short press"
			INTERNAL_SYS_ACTION_SIGNAL_INIT();
			break;
            
        // The Bluetooth LED is being controlled by either enabling the port as an
        // output or as an input. This operation is being controlled elsewhere.
        // This function is called by a function that initializes all I/O.
        // We will initialize the pin as an output here to turn off the Blue
        // LED but will have to turn it on if the "Using Bluetooth" feature
        // is turned on.
        case GEN_OUT_CTRL_ID_BT_LED:
            BLUETOOTH_LED_SIGNAL_INIT(); 
            break;

        case GEN_OUT_CTRL_ID_BT_FORWARD_DEMAND:
            BT_CTR_PAD_INIT();
            break;
            
        case GEN_OUT_CTRL_ID_BT_LEFT_DEMAND:
            BT_LEFT_PAD_INIT();
            break;
            
        case GEN_OUT_CTRL_ID_BT_RIGHT_DEMAND:
            BT_RIGHT_PAD_INIT();
            break;

        case GEN_OUT_CTRL_ID_BT_SEQUENCE:   // Make the BT sequence is disabled.
            AppCommonSetBTSequenceActive (false);
            break;
            
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
		case GEN_OUT_CTRL_ID_LED0:
			// Primary feedback LED
			LED0and1_SIGNAL_DEINIT();
			break;
			
    	case GEN_OUT_CTRL_ID_LED1:
			// Primary feedback LED
			LED0and1_SIGNAL_DEINIT();
			break;

    	case GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION:
			// Signal line used by this device to let the system know things like "resetting" and "user button short press"
			INTERNAL_SYS_ACTION_SIGNAL_DEINIT();
			break;

        case GEN_OUT_CTRL_ID_BT_LED:
            BLUETOOTH_LED_SIGNAL_DEINIT();
            break;
            
        case GEN_OUT_CTRL_ID_BT_FORWARD_DEMAND:
            BT_CTR_PAD_DEINIT();
            break;
            
        case GEN_OUT_CTRL_ID_BT_LEFT_DEMAND:
            BT_LEFT_PAD_DEINIT();
            break;
            
        case GEN_OUT_CTRL_ID_BT_RIGHT_DEMAND:
            BT_RIGHT_PAD_DEINIT();
            break;
            
        case GEN_OUT_CTRL_ID_BT_SEQUENCE:   // Make the BT sequence is disabled.
            AppCommonSetBTSequenceActive (false);
            break;
            
	   	case GEN_OUT_CTRL_ID_MAX:
	   	default:
		   	ret_val = false;
		   	return false;
	}
	
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
		case GEN_OUT_CTRL_ID_LED0:
			// Primary feedback LED
			LED0_SIGNAL_SET(true);
			break;
			
    	case GEN_OUT_CTRL_ID_LED1:
			// Primary feedback LED
			LED1_SIGNAL_SET(true);
			break;

    	case GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION:
			// Signal line used by this device to let the system know things like "resetting" and "user button short press"
			INTERNAL_SYS_ACTION_SIGNAL_SET(true);
			break;
            
        case GEN_OUT_CTRL_ID_BT_LED:
            BLUETOOTH_LED_SIGNAL_SET(true);  // Turn on the LED
            break;

        case GEN_OUT_CTRL_ID_BT_FORWARD_DEMAND:
            BT_CTR_PAD_SET(true);
            break;
            
        case GEN_OUT_CTRL_ID_BT_LEFT_DEMAND:
            BT_LEFT_PAD_SET(true);
            break;
            
        case GEN_OUT_CTRL_ID_BT_RIGHT_DEMAND:
            BT_RIGHT_PAD_SET(true);
            break;

        case GEN_OUT_CTRL_ID_BT_SEQUENCE:   // Indicate the BT module is being enabled or disabled.
            AppCommonSetBTSequenceActive (true);
            break;
            
	   	case GEN_OUT_CTRL_ID_MAX:
	   	default:
		  	break; // Intentionally do nothing.
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
		case GEN_OUT_CTRL_ID_LED0:
			// Primary feedback LED
			LED0_SIGNAL_SET(false);
			break;
			
    	case GEN_OUT_CTRL_ID_LED1:
			// Primary feedback LED
			LED1_SIGNAL_SET(false);
			break;

    	case GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION:
			// Signal line used by this device to let the system know things like "resetting" and "user button short press"
			INTERNAL_SYS_ACTION_SIGNAL_SET(false);
			break;
            
        case GEN_OUT_CTRL_ID_BT_LED:
            BLUETOOTH_LED_SIGNAL_SET(false);  // Turn off the LED.
            break;
		
        case GEN_OUT_CTRL_ID_BT_FORWARD_DEMAND:
            BT_CTR_PAD_SET(false);
            break;
            
        case GEN_OUT_CTRL_ID_BT_LEFT_DEMAND:
            BT_LEFT_PAD_SET(false);
            break;
            
        case GEN_OUT_CTRL_ID_BT_RIGHT_DEMAND:
            BT_RIGHT_PAD_SET(false);
            break;
            
        case GEN_OUT_CTRL_ID_BT_SEQUENCE:   // Indicate the BT module is NOT being enabled or disabled.
            AppCommonSetBTSequenceActive (false);
            break;
            
	   	case GEN_OUT_CTRL_ID_MAX:
	   	default:
		   	break; // Intentionally do nothing.
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
		case GEN_OUT_CTRL_ID_LED0:
			// Primary feedback LED
			LED0_SIGNAL_TOGGLE();
			break;
			
    	case GEN_OUT_CTRL_ID_LED1:
			// Primary feedback LED
			LED1_SIGNAL_TOGGLE();
			break;

    	case GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION:
			// Signal line used by this device to let the system know things like "resetting" and "user button short press"
			INTERNAL_SYS_ACTION_SIGNAL_TOGGLE();
			break;

        case GEN_OUT_CTRL_ID_BT_LED:
            BLUETOOTH_LED_SIGNAL_TOGGLE ();
            break;
		
        case GEN_OUT_CTRL_ID_BT_FORWARD_DEMAND:
            BT_CTR_PAD_TOGGLE();
            break;
            
        case GEN_OUT_CTRL_ID_BT_LEFT_DEMAND:
            BT_LEFT_PAD_TOGGLE();
            break;
            
        case GEN_OUT_CTRL_ID_BT_RIGHT_DEMAND:
            BT_RIGHT_PAD_TOGGLE();
            break;
            
	   case GEN_OUT_CTRL_ID_MAX:
	   default:
		   break; // Intentionally do nothing.
	}
	
	assert(ret_val);
	return ret_val;
}

// End of Doxygen grouping
/** @} */

#endif

/**********************************************************************************************************************
---          End of File          ------          End of File          ------          End of File          ---
**********************************************************************************************************************/
