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

// from project
#include "bsp.h"
#include "head_array_common.h"
#include "rtos_app.h"

// from local
#include "dac_bsp.h"

/* ******************************   Macros   ****************************** */

// Number of bits that the DAC is.
#define DAC_BSP_NUM_BITS ((uint16_t)12)

//-------------------------------
// forward/reverse movement DAC
//
// Pin that latches data pushed into the DAC's shift register
#define DAC_FWD_REV_MOVEMENT_DATA_LATCH_PIN		(PIN_D4)
#define DAC_FWD_REV_MOVEMENT_DATA_PIN			(PIN_D5)
#define DAC_FWD_REV_MOVEMENT_CLK_PIN			(PIN_D6)

//-------------------------------
// left/right movement DAC
//
#define DAC_LEFT_RIGHT_MOVEMENT_DATA_LATCH_PIN	(PIN_D7)
#define DAC_LEFT_RIGHT_MOVEMENT_DATA_PIN		(PIN_D2)
#define DAC_LEFT_RIGHT_MOVEMENT_CLK_PIN			(PIN_D1)

//-------------------------------
// Common DAC control
//
#define DAC_LATCH_ACTIVE_STATE					GPIO_LOW
#define DAC_LATCH_INACTIVE_STATE				GPIO_HIGH

/* ******************************   Types   ******************************* */

typedef struct
{
	uint8_t clk_pin;
	uint8_t data_pin;
	uint8_t latch_pin;
} DacPhy_t;

/* ***********************   File Scope Variables   *********************** */

static DacPhy_t dac_phy[(int)DAC_SELECT_SENSOR_EOL] =
{
    // clk_pin							data_pin							latch_pin
	{DAC_FWD_REV_MOVEMENT_CLK_PIN,		DAC_FWD_REV_MOVEMENT_DATA_PIN,		DAC_FWD_REV_MOVEMENT_DATA_LATCH_PIN},
	{DAC_LEFT_RIGHT_MOVEMENT_CLK_PIN,	DAC_LEFT_RIGHT_MOVEMENT_DATA_PIN,	DAC_LEFT_RIGHT_MOVEMENT_DATA_LATCH_PIN}
};

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
	// Put latches into inactive state.
	LatchStateSet(DAC_SELECT_FORWARD_BACKWARD, false);
	LatchStateSet(DAC_SELECT_LEFT_RIGHT, false);
	
	ClockStateSet(DAC_SELECT_FORWARD_BACKWARD, true);
	ClockStateSet(DAC_SELECT_LEFT_RIGHT, true);
	
	DataStateSet(DAC_SELECT_FORWARD_BACKWARD, false);
	DataStateSet(DAC_SELECT_LEFT_RIGHT, false);
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
// TODO: Consider using a UART.
//
//-------------------------------
void dacBspSet(DacSelect_t dac_id, uint16_t val)
{
	ClockStateSet(dac_id, false);
	
	for (uint16_t i = 0; i < DAC_BSP_NUM_BITS; i++)
	{
		DataStateSet(dac_id, (val & ((uint16_t)1 << ((DAC_BSP_NUM_BITS - (uint16_t)1) - (uint16_t)i))) ? true : false);

		ClockStateSet(dac_id, true);
		
		if (i == (DAC_BSP_NUM_BITS - 1))
		{
			LatchStateSet(dac_id, true);
		}

		ClockStateSet(dac_id, false);
	}

	LatchStateSet(dac_id, false);
	ClockStateSet(dac_id, true);
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
	output_bit(dac_phy[(int)dac_id].data_pin, high ? GPIO_HIGH : GPIO_LOW);
}

//-------------------------------
// Function: ClockStateSet
//
// Description: Sets the state of a DAC's clock line
//
//-------------------------------
static void ClockStateSet(DacSelect_t dac_id, bool high)
{
	output_bit(dac_phy[(int)dac_id].clk_pin, high ? GPIO_HIGH : GPIO_LOW);
}

//-------------------------------
// Function: LatchStateSet
//
// Description: Sets the state of a DAC's latch line
//
//-------------------------------
static void LatchStateSet(DacSelect_t dac_id, bool active)
{
	if (active)
	{
		output_bit(dac_phy[(int)dac_id].latch_pin, DAC_LATCH_ACTIVE_STATE);
	}
	else
	{
		output_bit(dac_phy[(int)dac_id].latch_pin, DAC_LATCH_INACTIVE_STATE);
	}
}

// end of file.
//-------------------------------------------------------------------------
