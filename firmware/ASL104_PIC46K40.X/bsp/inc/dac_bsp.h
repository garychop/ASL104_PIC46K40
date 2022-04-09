//////////////////////////////////////////////////////////////////////////////
//
// Filename: dac_bsp.h
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

#ifndef DAC_BSP_H
#define DAC_BSP_H

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdint.h>

// from project
#include "head_array_common.h"

/* ***********************   Function Prototypes   ************************ */

void dacBspInit(void);
void dacBspSet(DacSelect_t dac_id, uint16_t val);

#endif // DAC_BSP_H

// end of file.
//-------------------------------------------------------------------------
