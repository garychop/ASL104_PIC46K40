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
 **************************************************************************************************
 *
 * @brief       Configures the general output control module.
 * @file        general_output_ctrl_cfg.h
 * @author      tparsh
 * @ingroup     USER_DRIVER
 *
 **************************************************************************************************
 */

#ifndef GENERAL_OUTPUT_CTRL_CFG_H_
#define GENERAL_OUTPUT_CTRL_CFG_H_

/**
 * @addtogroup USER_DRIVER
 * @{
 */

/*
 ********************************************************************************************************
 *                                           INCLUDE FILES
 ********************************************************************************************************
 */
/********************************************** System *************************************************/
#include <stdint.h>

/*
 **************************************************************************************************
 *                                                 DEFINES
 **************************************************************************************************
 */
/******************************************* Symbolic Constants **********************************/
/**
 * Uncomment to enable mutex usage in the general output control module. Relevant functions must be defined by the
 * application or there will be compiler errors.
 */
/******************************************* Symbolic Constants ****************************************/
/// Uncomment to enable mutex usage in the general output control module. Relevant functions must be defined by the application or there will be compiler errors.
// #define GENERAL_OUTPUT_CTRL_USE_MUTEX

// Enables/disables this module
#define GENERAL_OUTPUT_CTRL_MODULE_ENABLE

/// Rate at which the general output update manager updates the time tracker for each general output, in milliseconds.
#define GENERAL_OUTPUT_CTRL_UPDATE_RATE_ms  (10) // "2" works OK.

/*
 ********************************************************************************************************
 *                                               DATA TYPES
 ********************************************************************************************************
 */

// Used by this module to determine the container size for thing like timeouts for API function call execution.
typedef uint16_t GenOutCtrlTime_t;

/**
 * Provides unique IDs for each controllable general output.
 *
 * @note The naming is redundant, but the LED prefix is to denote this module.  The pin
 * definitions match these names and a distinction is needed to be made between the GPIO
 * naming abstraction and the definitions below.
 * 
 * TODO: Make module support groups.  LEDs 0 and 1 are groups. Need to tweak driver to allow pins to be grouped together
 */
typedef enum
{
    GEN_OUT_CTRL_ID_LED0, /// Primary feedback LED
    GEN_OUT_CTRL_ID_LED1, /// Primary feedback LED
    GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION, /// Signal line used by this device to let the system know things like "resetting" and "user button short press"
    GEN_OUT_CTRL_ID_BT_LED,
    GEN_OUT_CTRL_ID_BT_FORWARD_DEMAND,
    GEN_OUT_CTRL_ID_BT_LEFT_DEMAND,
    GEN_OUT_CTRL_ID_BT_RIGHT_DEMAND,
    GEN_OUT_CTRL_ID_BT_SEQUENCE,    // This is used to indicate that the BT Module
                                    // .. sequence is active or inactive.
    /// Defines the list size. MUST BE THE LAST ITEM IN THE LIST! Also, this must be in the list.
    GEN_OUT_CTRL_ID_MAX
} GenOutCtrlId_t;

/**
 * Provides unique IDs for each operational state a general output can be in.  Each state may control any number of
 * general outputs.  Those that a state does not control are not modified on the given state change.
 */
typedef enum
{
    /// Name is self explanatory
    GEN_OUT_CTRL_STATE_IDLE,

    //-------------------------------------------------------------------------
    // States that only pertain to the IDs GEN_OUT_CTRL_LED0 and GEN_OUT_CTRL_LED1
    //-------------------------------------------------------------------------
    //
    /// Mode where wheelchair output control is routed to the Bluetooth module
    GEN_OUT_CTRL_STATE_BLUETOOTH_OUTPUT,

    /// Mode where wheelchair output control is routed to the Bluetooth module
    GEN_OUT_CTRL_STATE_HEAD_ARRAY_ACTIVE,
    
    /// Mode where there is no control output at all.
    GEN_OUT_CTRL_STATE_NO_OUTPUT,

    //-------------------------------------------------------------------------
    // States that only pertain to the IDs GEN_OUT_CTRL_INTERNAL_SYS_ACTION
    //-------------------------------------------------------------------------
    //
    /// The head array is resetting
    GEN_OUT_CTRL_STATE_HEAD_ARRAY_RESETTING,

    /// Buddy button short press invoking next function feature
    GEN_OUT_CTRL_STATE_STATE_USER_BTN_NEXT_FUNCTION,
    
    /// Buddy button short press invoking next profile feature
    GEN_OUT_CTRL_STATE_USER_BTN_NEXT_PROFILE,
            
    /// Mode Button input control when pressed and when released.
    GEN_OUT_CTRL_STATE_MODE_ACTIVE,
    GEN_OUT_CTRL_STATE_MODE_INACTIVE,
    
    /// User Port button is pressed while RNET SLEEP is active
    GEN_OUT_CTRL_RNET_SLEEP,

    GEN_OUT_CTRL_BT_FORWARD_DEMAND_ACTIVE,
    GEN_OUT_CTRL_BT_FORWARD_DEMAND_INACTIVE,
    GEN_OUT_CTRL_BT_RIGHT_DEMAND_ACTIVE,
    GEN_OUT_CTRL_BT_RIGHT_DEMAND_INACTIVE,
    GEN_OUT_CTRL_BT_LEFT_DEMAND_ACTIVE,
    GEN_OUT_CTRL_BT_LEFT_DEMAND_INACTIVE,
            
    //-------------------------------------------------------------------------
    // Output controller agnostic states
    //-------------------------------------------------------------------------
    //
    /// Tests all outputs, nothing more and nothing less
    GEN_OUT_STATE_CTRL_TEST,
    GEN_OUT_BLUETOOTH_ENABLED,
    GEN_OUT_BLUETOOTH_DISABLED,

    /// Defines the list size. MUST EXIST AND BE THE LAST ITEM IN THE LIST!
    GEN_OUT_CTRL_STATE_MAX
} GenOutState_t;

// End of Doxygen grouping
/** @} */

#endif // End of GENERAL_OUTPUT_CTRL_CFG_H_

/**************************************************************************************************
---          End of File          ------          End of File          ---
**************************************************************************************************/
