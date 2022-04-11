//////////////////////////////////////////////////////////////////////////////
//
// Filename: dac_bsp.c
//
// Description: BSP level control of all on-board DACs.
//
//  DAC used is the the LTC1257. See datasheet below for more details:
//	https://www.analog.com/media/en/technical-documentation/data-sheets/1257fc.pdf
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
#include "dac_bsp.h"

/* ******************************   Macros   ****************************** */

// Number of bits that the DAC is.
#define DAC_BSP_NUM_BITS ((uint16_t)12)

//-------------------------------
// Common DAC control
//
#define DAC_LATCH_ACTIVE_STATE					GPIO_LOW
#define DAC_LATCH_INACTIVE_STATE				GPIO_HIGH

#ifdef _18F46K40
    //-------------------------------
    // forward/reverse movement DAC
    //
    // Pin that latches data pushed into the DAC's shift register
    // Pin D4
    #define DAC_FWD_REV_MOVEMENT_DATA_LATCH_IS_ACTIVE()		(LATDbits.LD4 == DAC_LATCH_ACTIVE_STATE)
    #define DAC_FWD_REV_MOVEMENT_DATA_LATCH_SET(active)		INLINE_EXPR(LATDbits.LD4 = active ? DAC_LATCH_ACTIVE_STATE : DAC_LATCH_INACTIVE_STATE)
    #define DAC_FWD_REV_MOVEMENT_DATA_LATCH_TOGGLE()		INLINE_EXPR(DAC_FWD_REV_MOVEMENT_DATA_LATCH_SET(DAC_FWD_REV_MOVEMENT_DATA_LATCH_IS_ACTIVE() ? false : true))
    #define DAC_FWD_REV_MOVEMENT_DATA_LATCH_INIT()			INLINE_EXPR(TRISDbits.TRISD4 = GPIO_BIT_OUTPUT; ANSELDbits.ANSELD4 = 0)

    // Pin D5
    #define DAC_FWD_REV_MOVEMENT_DATA_IS_ACTIVE()			(LATDbits.LATD5 == GPIO_HIGH)
    #define DAC_FWD_REV_MOVEMENT_DATA_SET(active)			INLINE_EXPR(LATDbits.LATD5 = active ? GPIO_HIGH : GPIO_LOW)
    #define DAC_FWD_REV_MOVEMENT_DATA_TOGGLE()				INLINE_EXPR(DAC_FWD_REV_MOVEMENT_DATA_SET(!DAC_FWD_REV_MOVEMENT_DATA_IS_ACTIVE()))
    #define DAC_FWD_REV_MOVEMENT_DATA_INIT()				INLINE_EXPR(TRISDbits.TRISD5 = GPIO_BIT_OUTPUT; ANSELDbits.ANSELD5 = 0)

    // Pin D6
    #define DAC_FWD_REV_MOVEMENT_CLK_IS_ACTIVE()			(LATDbits.LATD6 == GPIO_HIGH)
    #define DAC_FWD_REV_MOVEMENT_CLK_SET(active)			INLINE_EXPR(LATDbits.LATD6 = active ? GPIO_HIGH : GPIO_LOW)
    #define DAC_FWD_REV_MOVEMENT_CLK_TOGGLE()				INLINE_EXPR(DAC_FWD_REV_MOVEMENT_CLK_SET(!DAC_FWD_REV_MOVEMENT_CLK_IS_ACTIVE()))
    #define DAC_FWD_REV_MOVEMENT_CLK_INIT()					INLINE_EXPR(TRISDbits.TRISD6 = GPIO_BIT_OUTPUT; ANSELDbits.ANSELD6 = 0)

    //-------------------------------
    // left/right movement DAC
    //
    // Pin D7
    #define DAC_LEFT_RIGHT_MOVEMENT_DATA_LATCH_IS_ACTIVE()	(LATDbits.LD7 == DAC_LATCH_ACTIVE_STATE)
    #define DAC_LEFT_RIGHT_MOVEMENT_DATA_LATCH_SET(active)	INLINE_EXPR(LATDbits.LD7 = active ? DAC_LATCH_ACTIVE_STATE : DAC_LATCH_INACTIVE_STATE)
    #define DAC_LEFT_RIGHT_MOVEMENT_DATA_LATCH_TOGGLE()		INLINE_EXPR(DAC_LEFT_RIGHT_MOVEMENT_DATA_LATCH_SET(DAC_LEFT_RIGHT_MOVEMENT_DATA_LATCH_IS_ACTIVE() ? false : true))
    #define DAC_LEFT_RIGHT_MOVEMENT_DATA_LATCH_INIT()		INLINE_EXPR(TRISDbits.TRISD7 = GPIO_BIT_OUTPUT; ANSELDbits.ANSELD7 = 0)

    // Pin D2
    #define DAC_LEFT_RIGHT_MOVEMENT_DATA_IS_ACTIVE()		(LATDbits.LATD2 == GPIO_HIGH)
    #define DAC_LEFT_RIGHT_MOVEMENT_DATA_SET(active)		INLINE_EXPR(LATDbits.LATD2 = active ? GPIO_HIGH : GPIO_LOW)
    #define DAC_LEFT_RIGHT_MOVEMENT_DATA_TOGGLE()			INLINE_EXPR(DAC_LEFT_RIGHT_MOVEMENT_DATA_SET(!DAC_LEFT_RIGHT_MOVEMENT_DATA_IS_ACTIVE()))
    #define DAC_LEFT_RIGHT_MOVEMENT_DATA_INIT()				INLINE_EXPR(TRISDbits.TRISD2 = GPIO_BIT_OUTPUT; ANSELDbits.ANSELD2 = 0)

    // Pin D1
    #define DAC_LEFT_RIGHT_MOVEMENT_CLK_IS_ACTIVE()			(LATDbits.LATD1 == GPIO_HIGH)
    #define DAC_LEFT_RIGHT_MOVEMENT_CLK_SET(active)			INLINE_EXPR(LATDbits.LATD1 = active ? GPIO_HIGH : GPIO_LOW)
    #define DAC_LEFT_RIGHT_MOVEMENT_CLK_TOGGLE()			INLINE_EXPR(DAC_LEFT_RIGHT_MOVEMENT_CLK_SET(!DAC_LEFT_RIGHT_MOVEMENT_CLK_IS_ACTIVE()))
    #define DAC_LEFT_RIGHT_MOVEMENT_CLK_INIT()				INLINE_EXPR(TRISDbits.TRISD1 = GPIO_BIT_OUTPUT; ANSELDbits.ANSELD1 = 0)
#else
    //-------------------------------
    // forward/reverse movement DAC
    //
    // Pin that latches data pushed into the DAC's shift register
    // Pin D4
//    #define DAC_FWD_REV_MOVEMENT_DATA_LATCH_IS_ACTIVE()		(LATDbits.LD4 == DAC_LATCH_ACTIVE_STATE)
//    #define DAC_FWD_REV_MOVEMENT_DATA_LATCH_SET(active)		INLINE_EXPR(LATDbits.LD4 = active ? DAC_LATCH_ACTIVE_STATE : DAC_LATCH_INACTIVE_STATE)
//    #define DAC_FWD_REV_MOVEMENT_DATA_LATCH_TOGGLE()		INLINE_EXPR(DAC_FWD_REV_MOVEMENT_DATA_LATCH_SET(DAC_FWD_REV_MOVEMENT_DATA_LATCH_IS_ACTIVE() ? false : true))
//    #define DAC_FWD_REV_MOVEMENT_DATA_LATCH_INIT()			INLINE_EXPR(TRISDbits.TRISD4 = GPIO_BIT_OUTPUT)
//
//    // Pin D5
//    #define DAC_FWD_REV_MOVEMENT_DATA_IS_ACTIVE()			(LATDbits.LATD5 == GPIO_HIGH)
//    #define DAC_FWD_REV_MOVEMENT_DATA_SET(active)			INLINE_EXPR(LATDbits.LATD5 = active ? GPIO_HIGH : GPIO_LOW)
//    #define DAC_FWD_REV_MOVEMENT_DATA_TOGGLE()				INLINE_EXPR(DAC_FWD_REV_MOVEMENT_DATA_SET(!DAC_FWD_REV_MOVEMENT_DATA_IS_ACTIVE()))
//    #define DAC_FWD_REV_MOVEMENT_DATA_INIT()				INLINE_EXPR(TRISDbits.TRISD5 = GPIO_BIT_OUTPUT)
//
//    // Pin D6
//    #define DAC_FWD_REV_MOVEMENT_CLK_IS_ACTIVE()			(LATDbits.LATD6 == GPIO_HIGH)
//    #define DAC_FWD_REV_MOVEMENT_CLK_SET(active)			INLINE_EXPR(LATDbits.LATD6 = active ? GPIO_HIGH : GPIO_LOW)
//    #define DAC_FWD_REV_MOVEMENT_CLK_TOGGLE()				INLINE_EXPR(DAC_FWD_REV_MOVEMENT_CLK_SET(!DAC_FWD_REV_MOVEMENT_CLK_IS_ACTIVE()))
//    #define DAC_FWD_REV_MOVEMENT_CLK_INIT()					INLINE_EXPR(TRISDbits.TRISD6 = GPIO_BIT_OUTPUT)
//
    //-------------------------------
    // left/right movement DAC
    //
    // Pin D7
//    #define DAC_LEFT_RIGHT_MOVEMENT_DATA_LATCH_IS_ACTIVE()	(LATDbits.LD7 == DAC_LATCH_ACTIVE_STATE)
//    #define DAC_LEFT_RIGHT_MOVEMENT_DATA_LATCH_SET(active)	INLINE_EXPR(LATDbits.LD7 = active ? DAC_LATCH_ACTIVE_STATE : DAC_LATCH_INACTIVE_STATE)
//    #define DAC_LEFT_RIGHT_MOVEMENT_DATA_LATCH_TOGGLE()		INLINE_EXPR(DAC_LEFT_RIGHT_MOVEMENT_DATA_LATCH_SET(DAC_LEFT_RIGHT_MOVEMENT_DATA_LATCH_IS_ACTIVE() ? false : true))
//    #define DAC_LEFT_RIGHT_MOVEMENT_DATA_LATCH_INIT()		INLINE_EXPR(TRISDbits.TRISD7 = GPIO_BIT_OUTPUT)
//
//    // Pin D2
//    #define DAC_LEFT_RIGHT_MOVEMENT_DATA_IS_ACTIVE()		(LATDbits.LATD2 == GPIO_HIGH)
//    #define DAC_LEFT_RIGHT_MOVEMENT_DATA_SET(active)		INLINE_EXPR(LATDbits.LATD2 = active ? GPIO_HIGH : GPIO_LOW)
//    #define DAC_LEFT_RIGHT_MOVEMENT_DATA_TOGGLE()			INLINE_EXPR(DAC_LEFT_RIGHT_MOVEMENT_DATA_SET(!DAC_LEFT_RIGHT_MOVEMENT_DATA_IS_ACTIVE()))
//    #define DAC_LEFT_RIGHT_MOVEMENT_DATA_INIT()				INLINE_EXPR(TRISDbits.TRISD2 = GPIO_BIT_OUTPUT)
//
//    // Pin D1
//    #define DAC_LEFT_RIGHT_MOVEMENT_CLK_IS_ACTIVE()			(LATDbits.LATD1 == GPIO_HIGH)
//    #define DAC_LEFT_RIGHT_MOVEMENT_CLK_SET(active)			INLINE_EXPR(LATDbits.LATD1 = active ? GPIO_HIGH : GPIO_LOW)
//    #define DAC_LEFT_RIGHT_MOVEMENT_CLK_TOGGLE()			INLINE_EXPR(DAC_LEFT_RIGHT_MOVEMENT_CLK_SET(!DAC_LEFT_RIGHT_MOVEMENT_CLK_IS_ACTIVE()))
//    #define DAC_LEFT_RIGHT_MOVEMENT_CLK_INIT()				INLINE_EXPR(TRISDbits.TRISD1 = GPIO_BIT_OUTPUT)
#endif

/* ***********************   Function Prototypes   ************************ */

static void DataStateSet(DacSelect_t dac_id, bool high);
static void ClockStateSet(DacSelect_t dac_id, bool high);
static void LatchStateSet(DacSelect_t dac_id, bool active);

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: dacBspInit
//
// Description: Initializes this module.
//
//-------------------------------
void dacBspInit(void)
{
//	DAC_FWD_REV_MOVEMENT_DATA_LATCH_INIT();
//	DAC_FWD_REV_MOVEMENT_DATA_INIT();
//	DAC_FWD_REV_MOVEMENT_CLK_INIT();
//	
//	DAC_LEFT_RIGHT_MOVEMENT_DATA_LATCH_INIT();
//	DAC_LEFT_RIGHT_MOVEMENT_DATA_INIT();
//	DAC_LEFT_RIGHT_MOVEMENT_CLK_INIT();
//	
//	// Put DAC control lines into "idle" state.
//	LatchStateSet(DAC_SELECT_FORWARD_BACKWARD, false);
//	LatchStateSet(DAC_SELECT_LEFT_RIGHT, false);
//	
//	ClockStateSet(DAC_SELECT_FORWARD_BACKWARD, true);
//	ClockStateSet(DAC_SELECT_LEFT_RIGHT, true);
//	
//	DataStateSet(DAC_SELECT_FORWARD_BACKWARD, false);
//	DataStateSet(DAC_SELECT_LEFT_RIGHT, false);
}

//-------------------------------
// Function: dacBspSet
//
// Description: Set the output value of a DAC.
//
// param: val - msbit must be at bit position 11 and lsbit at bit position 0.
//
//	It is a 12-bit DAC. The update sequence is as follows:
//  1. Ensure the LATCH signal is inactive and clock is high.
//  2. Set clock low.
//	3. Clock in 12 bits of data. Data is clocked in on the rising edge of the serial clock.
//	   Clock must be high for at least 350 ns and low for at least 350 ns.
//  	a. Set data before rising edge of clock, so, set data then pulse clock line high->low
//	4. Pulse latch/load line low->high. Must be at least 150 ns for the low pulse.
//
// NOTE: There is no need to put an explicit delay in the code.  It takes many us to execute a function call.
// NOTE:  That is enough of a delay.
//
// TODO: Determine how much time it takes to do this operation!
//
//-------------------------------
void dacBspSet(DacSelect_t dac_id, uint16_t val)
{
//	ClockStateSet(dac_id, false);
//	
//	for (uint16_t i = 0; i < DAC_BSP_NUM_BITS; i++)
//	{
//		DataStateSet(dac_id, (val & ((uint16_t)1 << ((DAC_BSP_NUM_BITS - (uint16_t)1) - (uint16_t)i))) ? true : false);
//
//		ClockStateSet(dac_id, true);
//		
//		if (i == (DAC_BSP_NUM_BITS - 1))
//		{
//			LatchStateSet(dac_id, true);
//		}
//
//		ClockStateSet(dac_id, false);
//	}
//
//	LatchStateSet(dac_id, false);
//	ClockStateSet(dac_id, true);
}

/* ********************   Private Function Definitions   ****************** */

//-------------------------------
// Function: DataStateSet
//
// Description: Sets the state of a DAC's data line
//
//-------------------------------
static void DataStateSet(DacSelect_t dac_id, bool high)
{
//	switch (dac_id)
//	{
//		case DAC_SELECT_FORWARD_BACKWARD:
//			DAC_FWD_REV_MOVEMENT_DATA_SET(high);
//			break;
//
//		case DAC_SELECT_LEFT_RIGHT:
//			DAC_LEFT_RIGHT_MOVEMENT_DATA_SET(high);
//			break;
//			
//		default:
//			ASSERT(dac_id == DAC_SELECT_LEFT_RIGHT);
//			break;
//	}
}

//-------------------------------
// Function: ClockStateSet
//
// Description: Sets the state of a DAC's clock line
//
//-------------------------------
static void ClockStateSet(DacSelect_t dac_id, bool high)
{
//	switch (dac_id)
//	{
//		case DAC_SELECT_FORWARD_BACKWARD:
//			DAC_FWD_REV_MOVEMENT_CLK_SET(high);
//			break;
//
//		case DAC_SELECT_LEFT_RIGHT:
//			DAC_LEFT_RIGHT_MOVEMENT_CLK_SET(high);
//			break;
//			
//		default:
//			ASSERT(dac_id == DAC_SELECT_LEFT_RIGHT);
//			break;
//	}
}

//-------------------------------
// Function: LatchStateSet
//
// Description: Sets the state of a DAC's latch line
//
//-------------------------------
static void LatchStateSet(DacSelect_t dac_id, bool active)
{
//	switch (dac_id)
//	{
//		case DAC_SELECT_FORWARD_BACKWARD:
//			DAC_FWD_REV_MOVEMENT_DATA_LATCH_SET(active);
//			break;
//
//		case DAC_SELECT_LEFT_RIGHT:
//			DAC_LEFT_RIGHT_MOVEMENT_DATA_LATCH_SET(active);
//			break;
//			
//		default:
//			ASSERT(dac_id == DAC_SELECT_LEFT_RIGHT);
//			break;
//	}
}

// end of file.
//-------------------------------------------------------------------------
