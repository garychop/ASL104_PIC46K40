//////////////////////////////////////////////////////////////////////////////
//
// Filename: bluetooth_simple_if_bsp.c
//
// Description: Exposes control over the simple interface to a bluetooth module.
// TODO: Actually call out the name of the Bluetooth module.
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

// from project
#include "bsp.h"
#include "common.h"
#include "head_array_common.h"

// from local
#include "bluetooth_simple_if_bsp.h"

/* ******************************   Macros   ****************************** */

#define BT_PAD_MIRROR_ACTIVE_STATE		GPIO_LOW
#define BT_PAD_MIRROR_INACTIVE_STATE	GPIO_HIGH

#ifdef _18F46K40
// New compiler does not like inline functions like this
//    #define BT_LEFT_PAD_IS_ACTIVE()		(LATEbits.LE1 == BT_PAD_MIRROR_ACTIVE_STATE)
//    #define BT_LEFT_PAD_SET(active)		INLINE_EXPR(LATEbits.LE1 = active ? BT_PAD_MIRROR_ACTIVE_STATE : BT_PAD_MIRROR_INACTIVE_STATE)
//    #define BT_LEFT_PAD_TOGGLE()		INLINE_EXPR(BT_LEFT_PAD_SET(BT_LEFT_PAD_IS_ACTIVE() ? false : true))
//    #define BT_LEFT_PAD_INIT()			INLINE_EXPR(TRISEbits.TRISE1 = GPIO_BIT_OUTPUT; ANSELEbits.ANSELE1 = 0; BT_LEFT_PAD_SET(false))
//
//    #define BT_RIGHT_PAD_IS_ACTIVE()	(LATCbits.LC1 == BT_PAD_MIRROR_ACTIVE_STATE)
//    #define BT_RIGHT_PAD_SET(active)	INLINE_EXPR(LATCbits.LC1 = active ? BT_PAD_MIRROR_ACTIVE_STATE : BT_PAD_MIRROR_INACTIVE_STATE)
//    #define BT_RIGHT_PAD_TOGGLE()		INLINE_EXPR(BT_RIGHT_PAD_SET(BT_RIGHT_PAD_IS_ACTIVE() ? false : true))
//    #define BT_RIGHT_PAD_INIT()			INLINE_EXPR(TRISCbits.TRISC1 = GPIO_BIT_OUTPUT; ANSELCbits.ANSELC1 = 0; BT_RIGHT_PAD_SET(false))
//
//    #define BT_CTR_PAD_IS_ACTIVE()		(LATDbits.LD3 == BT_PAD_MIRROR_ACTIVE_STATE)
//    #define BT_CTR_PAD_SET(active)		INLINE_EXPR(LATDbits.LD3 = active ? BT_PAD_MIRROR_ACTIVE_STATE : BT_PAD_MIRROR_INACTIVE_STATE)
//    #define BT_CTR_PAD_TOGGLE()			INLINE_EXPR(BT_CTR_PAD_SET(BT_CTR_PAD_IS_ACTIVE() ? false : true))
//    #define BT_CTR_PAD_INIT()			INLINE_EXPR(TRISDbits.TRISD3 = GPIO_BIT_OUTPUT; ANSELDbits.ANSELD3 = 0; BT_CTR_PAD_SET(false))
#else
    #define BT_LEFT_PAD_IS_ACTIVE()		(LATDbits.LD1 == BT_PAD_MIRROR_ACTIVE_STATE)
    #define BT_LEFT_PAD_SET(active)		INLINE_EXPR(LATDbits.LD1 = active ? BT_PAD_MIRROR_ACTIVE_STATE : BT_PAD_MIRROR_INACTIVE_STATE)
    #define BT_LEFT_PAD_TOGGLE()		INLINE_EXPR(BT_LEFT_PAD_SET(BT_LEFT_PAD_IS_ACTIVE() ? false : true))
    #define BT_LEFT_PAD_INIT()			INLINE_EXPR(TRISDbits.TRISD1 = GPIO_BIT_OUTPUT; BT_LEFT_PAD_SET(false))

    #define BT_RIGHT_PAD_IS_ACTIVE()	(LATDbits.LD7 == BT_PAD_MIRROR_ACTIVE_STATE)
    #define BT_RIGHT_PAD_SET(active)	INLINE_EXPR(LATDbits.LD7 = active ? BT_PAD_MIRROR_ACTIVE_STATE : BT_PAD_MIRROR_INACTIVE_STATE)
    #define BT_RIGHT_PAD_TOGGLE()		INLINE_EXPR(BT_RIGHT_PAD_SET(BT_RIGHT_PAD_IS_ACTIVE() ? false : true))
    #define BT_RIGHT_PAD_INIT()			INLINE_EXPR(TRISDbits.TRISD7 = GPIO_BIT_OUTPUT; BT_RIGHT_PAD_SET(false))

    #define BT_CTR_PAD_IS_ACTIVE()		(LATDbits.LD4 == BT_PAD_MIRROR_ACTIVE_STATE)
    #define BT_CTR_PAD_SET(active)		INLINE_EXPR(LATDbits.LD4 = active ? BT_PAD_MIRROR_ACTIVE_STATE : BT_PAD_MIRROR_INACTIVE_STATE)
    #define BT_CTR_PAD_TOGGLE()			INLINE_EXPR(BT_CTR_PAD_SET(BT_CTR_PAD_IS_ACTIVE() ? false : true))
    #define BT_CTR_PAD_INIT()			INLINE_EXPR(TRISDbits.TRISD4 = GPIO_BIT_OUTPUT; BT_CTR_PAD_SET(false))
    
    #define BT_REV_PAD_IS_ACTIVE()      (LATDbits.LD0 == BT_PAD_MIRROR_ACTIVE_STATE)
    #define BT_REV_PAD_SET(active)      INLINE_EXPR(LATDbits.LD0 = active ? BT_PAD_MIRROR_ACTIVE_STATE : BT_PAD_MIRROR_INACTIVE_STATE)
    #define BT_REV_PAD_TOGGLE()	        INLINE_EXPR(BT_REV_PAD_SET(BT_REV_PAD_IS_ACTIVE() ? false : true))
    #define BT_REV_PAD_INIT()           INLINE_EXPR(TRISDbits.TRISD0 = GPIO_BIT_OUTPUT; BT_REV_PAD_SET(false))
#endif

/* *******************   Public Function Definitions   ******************** */

//------------------------------------------------------------------------------
bool BT_LEFT_PAD_IS_ACTIVE(void)
{
    return (LATDbits.LD1 == BT_PAD_MIRROR_ACTIVE_STATE);
}
void BT_LEFT_PAD_SET(bool active)
{
    LATDbits.LD1 = active ? BT_PAD_MIRROR_ACTIVE_STATE : BT_PAD_MIRROR_INACTIVE_STATE;
}
void BT_LEFT_PAD_TOGGLE(void)
{
    BT_LEFT_PAD_SET (BT_LEFT_PAD_IS_ACTIVE() ? false : true); 
}
void BT_LEFT_PAD_INIT(void)
{
    TRISDbits.TRISD1 = GPIO_BIT_OUTPUT;
    ANSELDbits.ANSELD1 = 0;
    BT_LEFT_PAD_SET(false);
}
//------------------------------------------------------------------------------
bool BT_RIGHT_PAD_IS_ACTIVE(void)
{
    return (LATDbits.LD7 == BT_PAD_MIRROR_ACTIVE_STATE);
}
void BT_RIGHT_PAD_SET(bool active)
{
    LATDbits.LD7 = active ? BT_PAD_MIRROR_ACTIVE_STATE : BT_PAD_MIRROR_INACTIVE_STATE;
}
void BT_RIGHT_PAD_TOGGLE(void)
{
    BT_RIGHT_PAD_SET(BT_RIGHT_PAD_IS_ACTIVE() ? false : true);
}
void BT_RIGHT_PAD_INIT(void)
{
    TRISDbits.TRISD7 = GPIO_BIT_OUTPUT;
    ANSELDbits.ANSELD7 = 0;
    BT_RIGHT_PAD_SET(false);
}

//------------------------------------------------------------------------------
bool BT_CTR_PAD_IS_ACTIVE(void )
{
    return (LATDbits.LD4 == BT_PAD_MIRROR_ACTIVE_STATE);
}
void BT_CTR_PAD_SET(bool active)
{
    LATDbits.LD4 = active ? BT_PAD_MIRROR_ACTIVE_STATE : BT_PAD_MIRROR_INACTIVE_STATE;
}
void BT_CTR_PAD_TOGGLE(void)
{
    BT_CTR_PAD_SET(BT_CTR_PAD_IS_ACTIVE() ? false : true);
}
void BT_CTR_PAD_INIT(void)
{
    TRISDbits.TRISD4 = GPIO_BIT_OUTPUT;
    ANSELDbits.ANSELD4 = 0;
    BT_CTR_PAD_SET(false);
}

//-------------------------------
// Function: bluetoothSimpleIfBspInit
//
// Description: Initializes this module
//
//-------------------------------
void bluetoothSimpleIfBspInit(void)
{
	BT_LEFT_PAD_INIT();
	BT_RIGHT_PAD_INIT();
	BT_CTR_PAD_INIT();
//    BT_REV_PAD_INIT();
}

//-------------------------------
// Function: bluetoothSimpleIfBspInit
//
// Description: Tells the Bluetooth module that the input mirroring is disabled.
//
//-------------------------------
void bluetoothSimpleIfBspPadMirrorDisable(void)
{
	bluetoothSimpleIfBspPadMirrorStateSet(HEAD_ARRAY_SENSOR_LEFT, false);
	bluetoothSimpleIfBspPadMirrorStateSet(HEAD_ARRAY_SENSOR_RIGHT, false);
	bluetoothSimpleIfBspPadMirrorStateSet(HEAD_ARRAY_SENSOR_CENTER, false);
	bluetoothSimpleIfBspPadMirrorStateSet(HEAD_ARRAY_SENSOR_BACK, false);
}

//-------------------------------
// Function: bluetoothSimpleIfBspPadMirrorStateSet
//
// Description: Sets the state of the mirrored head array sensor output to the bluetooth module.
//
//-------------------------------
void bluetoothSimpleIfBspPadMirrorStateSet(HeadArraySensor_t sensor_id, bool active)
{
	switch (sensor_id)
	{
		case HEAD_ARRAY_SENSOR_LEFT:
			BT_LEFT_PAD_SET(active);
			break;

		case HEAD_ARRAY_SENSOR_RIGHT:
			BT_RIGHT_PAD_SET(active);
			break;
			
		case HEAD_ARRAY_SENSOR_CENTER:
			BT_CTR_PAD_SET(active);
			break;

        case HEAD_ARRAY_SENSOR_BACK:
            //BT_REV_PAD_SET(active);
            break;
            
		case HEAD_ARRAY_SENSOR_EOL:
		default:
			ASSERT(sensor_id == HEAD_ARRAY_SENSOR_CENTER);
			break;
	}
}

//-------------------------------
// Function: bluetoothSimpleIfBspPadMirrorStateGet
//
// Description: Gets the state of the mirrored heead array sensor output to the bluetooth module.
//
//-------------------------------
bool bluetoothSimpleIfBspPadMirrorStateGet(HeadArraySensor_t sensor_id)
{
	uint8_t output_pin;

	switch (sensor_id)
	{
		case HEAD_ARRAY_SENSOR_LEFT:
			return BT_LEFT_PAD_IS_ACTIVE();

		case HEAD_ARRAY_SENSOR_RIGHT:
			return BT_RIGHT_PAD_IS_ACTIVE();
			
		case HEAD_ARRAY_SENSOR_CENTER:
			return BT_CTR_PAD_IS_ACTIVE();
			
        case HEAD_ARRAY_SENSOR_BACK:
            //return BT_REV_PAD_IS_ACTIVE();
            
		case HEAD_ARRAY_SENSOR_EOL:
		default:
			ASSERT(sensor_id == HEAD_ARRAY_SENSOR_CENTER);
			return false;
	}
    return false;   // TODO: Replace with real code.
}

// end of file.
//-------------------------------------------------------------------------
