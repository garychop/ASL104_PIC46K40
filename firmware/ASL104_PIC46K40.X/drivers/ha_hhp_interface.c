//////////////////////////////////////////////////////////////////////////////
//
// Filename: ha_hhp_interface.c
//
// Description: Defines the communications interface between a head array and HHP display device.
//
//	 This device is set to slave
//	 
//   I2C send message
//   The communication protocol I am using is not the real I2C.
//   I only use I2C start bit and 8 bit data trans method. 
//   The trans stop controlled by length byte. 
//   After data transmition stop, set io and clk to high, then set io and
//   clk as input. only when sending data can set io and clk
//   as output. no stop bit and ack bit.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

/* ***************************    Includes     **************************** */

// NOTE: This must ALWAYS be the first include in a file.
#include "device.h"

// from stdlib
#include <stdint.h>
#include <stdbool.h>
#include "user_assert.h"

// from project
#include "config.h"
#include "test_gpio.h"

// from local
#include "ha_hhp_interface_bsp.h"
#include "ha_hhp_interface.h"

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: haHhp_Init
//
// Description: Initializes this module.
//
//-------------------------------
void haHhp_Init(void)
{
	haHhpBsp_Init();
}

//-------------------------------
// Function: haHhp_RxPacket
//
// Description: Reads in an ensure packet from over the HHP communications interface.
//
// return: true: read in packet, false: timed out trying to read in packet.
//-------------------------------
bool haHhp_RxPacket(uint8_t *rx_buff)
{
	bool ret_val = true;

	// Receive the length of the packet
	if (haHhpBsp_RxByte(&rx_buff[0]) &&
		(rx_buff[0] <= HHP_RX_TX_BUFF_LEN))
	{
		// Get the rest of the packet.
		// Start at 1 because the message length byte itself is in the packet and required for the checksum calc
		for (unsigned int i = 1; (i < rx_buff[0]) && ret_val; i++)
		{
			ret_val = haHhpBsp_RxByte(&rx_buff[i]);
		}
	}
	else
	{
		// Message is too long or timed out.
		ret_val = false;
	}

	return ret_val;
}

// end of file.
//-------------------------------------------------------------------------
