//////////////////////////////////////////////////////////////////////////////
//
// Filename: ha_hhp_interface_bsp.h
//
// Description: Defines the BSP level for the communications interface between a head array and HHP display device.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef HA_HHP_INTERFACE_BSP_H_
#define HA_HHP_INTERFACE_BSP_H_

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdint.h>
#include <stdbool.h>

/* ******************************   Macros   ****************************** */

// Through test (building with opt0/1/2) found that the only flag that gets set is this one.  Unfortunately,
// it is the same flag set for opt level 1 and 2. There is no difference in flags set between opt 1 and 2.
#if (__OPTIM_FLAGS & 0x01)
	// This is optimization level 2
	#define HA_HHP_LINE_INPUT_TO_25_us 		(4) // Actual: 25.9 us
	#define HA_HHP_LINE_INPUT_TO_50_us 		(10) // Actual: 52.3 us
	#define HA_HHP_LINE_INPUT_TO_100_us 	(21) // Actual: 100.7us
	#define HA_HHP_LINE_INPUT_TO_150_us 	(33) // Actual: 153.5 us
	#define HA_HHP_LINE_INPUT_TO_200_us 	(44) // Actual: 201.9 us
	#define HA_HHP_LINE_INPUT_TO_250_us 	(55) // Actual: 250.3us
	#define HA_HHP_LINE_INPUT_TO_300_us 	(67) // Actual: 303.1 us
	#define HA_HHP_LINE_INPUT_TO_400_us 	(89) // Actual: 399.9 us
	#define HA_HHP_LINE_INPUT_TO_500_us 	(112) // Actual: 501.1 us
	#define HA_HHP_LINE_INPUT_TO_750_us 	(169) // Actual: 751.9 us
	#define HA_HHP_LINE_INPUT_TO_1000_us 	(226) // Actual: 1002.7 us
#else
	// This is optimization level 0
	#define HA_HHP_LINE_INPUT_TO_25_us 		(3) // Actual: 29.2 us
	#define HA_HHP_LINE_INPUT_TO_50_us 		(7) // Actual: 54.8 us
	#define HA_HHP_LINE_INPUT_TO_100_us 	(14) // Actual: 99.6us
	#define HA_HHP_LINE_INPUT_TO_150_us 	(22) // Actual: 150.8 us
	#define HA_HHP_LINE_INPUT_TO_200_us 	(30) // Actual: 202 us
	#define HA_HHP_LINE_INPUT_TO_250_us 	(38) // Actual: 253.2us
	#define HA_HHP_LINE_INPUT_TO_300_us 	(46) // Actual: 304.4 us
	#define HA_HHP_LINE_INPUT_TO_400_us 	(61) // Actual: 400.4 us
	#define HA_HHP_LINE_INPUT_TO_500_us 	(77) // Actual: 502.8 us
	#define HA_HHP_LINE_INPUT_TO_750_us 	(116) // Actual: 752.4 us
	#define HA_HHP_LINE_INPUT_TO_1000_us 	(155) // Actual: 1002 us
#endif

/* ***********************   Function Prototypes   ************************ */

void haHhpBsp_Init(void);
bool haHhpBsp_ReadyToReceivePacket(void);
void haHhpBsp_TransmitPacket(uint8_t *tx_pkt, uint8_t len);
bool haHhpBsp_RxByte(uint8_t *rxd_byte);
bool haHhpBsp_MasterRtsAsserted(void);
void haHhpBsp_SlaveReadyToReceivePacket(void);

#endif // End of HA_HHP_INTERFACE_BSP_H_

// end of file.
//-------------------------------------------------------------------------
