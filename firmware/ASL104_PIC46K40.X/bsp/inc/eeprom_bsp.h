//////////////////////////////////////////////////////////////////////////////
//
// Filename: eeprom_bsp.h
//
// Description: Control driver for the PIC18F4550's internal EEPROM
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef EEPROM_BSP_H
#define EEPROM_BSP_H

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdint.h>
#include <stdbool.h>

/* ***********************   Function Prototypes   ************************ */

void eepromBspInit(void);
bool eepromBspWriteByte(uint8_t address, uint8_t byte_to_write, uint16_t timeout_ms);
bool eepromBspWriteBuffer(uint8_t start_address, uint8_t num_bytes_to_write, uint8_t *data, uint16_t timeout_ms);
bool eepromBspReadSection(uint8_t start_address, uint8_t num_bytes_to_read, uint8_t *buffer, uint16_t timeout_ms);
uint16_t eepromBspSizeOfEeprom(void);

#endif // EEPROM_BSP_H

// end of file.
//-------------------------------------------------------------------------
