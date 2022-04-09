//////////////////////////////////////////////////////////////////////////////
//
// Filename: ha_hhp_interface.h
//
// Description: Defines the communications interface between a head array and HHP display device.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef HA_HHP_INTERFACE_H_
#define HA_HHP_INTERFACE_H_

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdint.h>
#include <stdbool.h>

/* ******************************   Macros   ****************************** */

// Length of the HHP communications rx/tx buffer
#define HHP_RX_TX_BUFF_LEN (8)

/* ***********************   Function Prototypes   ************************ */

void haHhp_Init(void);
bool haHhp_RxPacket(uint8_t *rx_buff);

#endif // End of HA_HHP_INTERFACE_H_

// end of file.
//-------------------------------------------------------------------------
