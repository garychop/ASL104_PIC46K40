//////////////////////////////////////////////////////////////////////////////
//
// Filename: eeprom_bsp.c
//
// Description: Control driver for the PIC18F4550's internal EEPROM
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
#include "config.h"
#include "common.h"

// from local
#include "eeprom_bsp.h"

/* ******************************   Macros   ****************************** */

// TODO: The size of 46K40 EEPROM is 1024. However, this driver only supports 1-byte
// TODO: addressing.
#define EEPROM_SIZE_OF_EEPROM ((uint16_t)256)

/* ***********************   Function Prototypes   ************************ */

static bool writeBuffer(uint8_t start_address, uint8_t num_bytes_to_write, uint8_t *data, uint16_t timeout_ms);
static void writeByte(uint8_t address, uint8_t data);
static bool waitForEepromToBeWritable(uint16_t timeout_ms);
static void readIntoBuffer(uint8_t start_address, uint8_t num_bytes_to_read, uint8_t *buffer);

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: eepromBspInit
//
// Description: Initializes this module
//
//-------------------------------
void eepromBspInit(void)
{
	(void)0;
    
#ifdef TEST_BASIC_EEPROM_CONTROL
    static uint8_t read_bytes[6] = {0};
    eepromBspWriteByte(0, 0xAA, 1);
    (void)eepromBspReadSection(0, 1, read_bytes, 1);
    
    uint8_t write_buff[4] = {1,2,3,4};
    eepromBspWriteBuffer(0, 4, write_buff, 4);
    (void)eepromBspReadSection(0, 4, read_bytes, 1);
    while(1);
#endif
}

//-------------------------------
// Function: eepromBspWriteByte
//
// Description: Writes a single byte to the internal EEPROM
//
// timeout_ms: Time to wait before giving up on waiting for EEPROM write access to be available.
//
//-------------------------------
bool eepromBspWriteByte(uint8_t address, uint8_t byte_to_write, uint16_t timeout_ms)
{
	UNUSED(timeout_ms);

	bool ret_val = writeBuffer(address, 1, &byte_to_write, timeout_ms);

	return ret_val;
}

//-------------------------------
// Function: eepromBspWriteBuffer
//
// Description: Writes an entire buffer to the internal EEPROM
//
// timeout_ms: Time to wait before giving up on waiting for EEPROM write access to be available.
//
//-------------------------------
bool eepromBspWriteBuffer(uint8_t start_address, uint8_t num_bytes_to_write, uint8_t *data, uint16_t timeout_ms)
{
	ASSERT(((uint16_t)start_address + (uint16_t)num_bytes_to_write) < (uint16_t)EEPROM_SIZE_OF_EEPROM);
	ASSERT(data != NULL);

	UNUSED(timeout_ms);

	bool ret_val = writeBuffer(start_address, num_bytes_to_write, data, timeout_ms);

	return ret_val;
}

//-------------------------------
// Function: eepromBspReadSection
//
// Description: Reads a section of data from internal EEPROM into a data buffer
//
// buffer: Buffer that stores data read from the EEPROM.
// timeout_ms: Time to wait before giving up on waiting for EEPROM read access to be available.
//
//-------------------------------
bool eepromBspReadSection(uint8_t start_address, uint8_t num_bytes_to_read, uint8_t *buffer, uint16_t timeout_ms)
{
	ASSERT(((uint16_t)start_address + (uint16_t)num_bytes_to_read) < (uint16_t)EEPROM_SIZE_OF_EEPROM);
	ASSERT(buffer != NULL);

	UNUSED(timeout_ms);

	readIntoBuffer(start_address, num_bytes_to_read, buffer);

	return true;
}

//-------------------------------
// Function: eepromBspSizeOfEeprom
//
// Description: Returns the total number of available bytes in the EEPROM.
//
//-------------------------------
uint16_t eepromBspSizeOfEeprom(void)
{
    return (uint16_t)EEPROM_SIZE_OF_EEPROM;
}

/* ********************   Private Function Definitions   ****************** */

//-------------------------------
// Function: writeBuffer
//
// Description: Writes a buffer full of data to the internal EEPROM.
//
// NOTE: Bounds checking is performed by the calling function.
//
//-------------------------------
static bool writeBuffer(uint8_t start_address, uint8_t num_bytes_to_write, uint8_t *data, uint16_t timeout_ms)
{
	UNUSED(timeout_ms);

	bool no_timeout = true;

	for (uint8_t i = 0; i < num_bytes_to_write; i++)
	{
		no_timeout = waitForEepromToBeWritable(timeout_ms);
		if (!no_timeout)
		{
			// EEPROM has been unable to be written to for too long, bounce.
			break;
		}

		writeByte(start_address + i, data[i]);
	}

	return no_timeout;
}

//-------------------------------
// Function: writeByte
//
// Description: Writes a single byte to the internal EEPROM
//
// TODO: Dig more into this process and clean up code a bit more.
//
//-------------------------------
static void writeByte(uint8_t address, uint8_t data)
{
#ifdef _18F46K40
	NVMCON1bits.NVMREG = 0;
	NVMADRL = address;
	NVMADRH = 0; // This application does not use that much EEPROM.
	NVMDAT = data;
	NVMCON1bits.WREN = 1;

	uint8_t start_gie_state = INTCONbits.GIE;
	INTCONbits.GIE = 0;
	NVMCON2 = 0x55;
	NVMCON2 = 0xAA;
	NVMCON1bits.WR = 1;
	INTCONbits.GIE = start_gie_state; // Re-Enable global interrupts if required
	NVMCON1bits.WREN = 0;
#else
	// Set address and data registers accordingly.
	EEADR = address;
	EEDATA = data;

	// Point to data section of EEPROM and enable EEPROM writes
	EECON1bits.CFGS = 0;
	EECON1bits.EEPGD = 0;
	EECON1bits.WREN = 1;

	// Critical section.  Cannot let any interrupts fire here, so disable global interrupts
	uint8_t start_gie_state = INTCONbits.GIE;
	INTCONbits.GIE = 0;
	EECON2 = 0x55;
	EECON2 = 0xAA;
	EECON1bits.WR = 1;
	INTCONbits.GIE = start_gie_state; // Re-Enable global interrupts if required

	EECON1bits.WREN = 0; // Disable EEPROM writes
	EECON1bits.WR = 0;
#endif
}


//-------------------------------
// Function: waitForEepromToBeWritable
//
// Description: Waits for the internal EEPROM to be ready for a write.
//
// timeout_ms: Time to wait before giving up on waiting for EEPROM write access to be available.
//
//-------------------------------
static bool waitForEepromToBeWritable(uint16_t timeout_ms)
{
	UNUSED(timeout_ms);

#ifdef _18F46K40
	// Make sure last attempt to write is complete
	// TODO: Add timeout and feedback on failure.
	while(NVMCON1bits.WR)
	{
		(void)0;
	}
#else
	// Make sure last attempt to write is complete
	// TODO: Add timeout and feedback on failure.
	while(EECON1bits.WR)
	{
		(void)0;
	}
#endif

	return true;
}

//-------------------------------
// Function: readIntoBuffer
//
// Description: Reads a section of data from internal EEPROM into a data buffer
//
// buffer: Buffer that stores data read from the EEPROM.
//
//-------------------------------
static void readIntoBuffer(uint8_t start_address, uint8_t num_bytes_to_read, uint8_t *buffer)
{
	for (uint8_t i = 0; i < num_bytes_to_read; i++)
	{
#ifdef _18F46K40
		NVMCON1bits.NVMREG = 0;
		NVMADRL = (start_address + i);
		NVMADRH = 0; // This application does not use that much EEPROM.
        
        NVMCON1bits.RD = 1;
        
		// There's no mention of needing delay between setting up for a read and actually reading
		// for PIC18(L)F46K40 MCUs.
		buffer[i] = NVMDAT;
#endif 
#ifdef USE_OLD_CODE
		EEADR = (start_address + i);

		// Point to data section of EEPROM and enable EEPROM
		EECON1bits.CFGS = 0;
		EECON1bits.EEPGD = 0;

		// Kick off the read operation
		NVMCON1bits.RD = 1;

		// Nop may be required for latency at high frequencies
		Nop();
		Nop();

		buffer[i] = EEDATA;
#endif
	}
}

// end of file.
//-------------------------------------------------------------------------
