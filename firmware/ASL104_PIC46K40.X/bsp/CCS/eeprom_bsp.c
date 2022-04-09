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
#include <stddef.h>
#include "user_assert.h"

// from project
#include "common.h"
#include "rtos_app.h"

// from local
#include "eeprom_bsp.h"

/* ******************************   Macros   ****************************** */

#define EEPROM_SIZE_OF_EEPROM ((uint16_t)256)

/* ***********************   File Scope Variables   *********************** */

static volatile uint8_t data_lock_mutex;

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
    data_lock_mutex = 1;
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
	
    rtosAppSemTake(&data_lock_mutex);
	bool ret_val = writeBuffer(address, 1, &byte_to_write, timeout_ms);
    rtosAppSemGive(&data_lock_mutex);

	return ret_val;
}

//-------------------------------
// Function: waitForEepromToBeReady
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
	
    rtosAppSemTake(&data_lock_mutex);
	bool ret_val = writeBuffer(start_address, num_bytes_to_write, data, timeout_ms);
    rtosAppSemGive(&data_lock_mutex);

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

    rtosAppSemTake(&data_lock_mutex);
	readIntoBuffer(start_address, num_bytes_to_read, buffer);
    rtosAppSemGive(&data_lock_mutex);

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
    return EEPROM_SIZE_OF_EEPROM;
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
	
	// TODO: Add timeout and feedback on failure.
	for (int i = 0; i < num_bytes_to_write; i++)
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
//-------------------------------
static void writeByte(uint8_t address, uint8_t data)
{
    // Address and data are 8-bit
    write_eeprom(address, data);
}


//-------------------------------
// Function: waitForEepromToBeWritable
//
// Description: Waits for the internal EEPROM to be ready for a write.
//
// timeout_ms: Time to wait before giving up on waiting for EEPROM write access to be available.
//
// NOTE: There is no way to probe the ready status with CCS.
//
//-------------------------------
static bool waitForEepromToBeWritable(uint16_t timeout_ms)
{
	UNUSED(timeout_ms);
    
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
        // Address and return is 8-bit
        buffer[i] = read_eeprom((start_address + i));
	}
}

// end of file.
//-------------------------------------------------------------------------
