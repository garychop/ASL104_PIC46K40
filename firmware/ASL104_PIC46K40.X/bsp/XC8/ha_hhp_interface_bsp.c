//////////////////////////////////////////////////////////////////////////////
//
// Filename: ha_hhp_interface_bsp.c
//
// Description: Defines the BSP level for the communications interface between a head array and HHP display device.
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

// from project
#include "config.h"
#include "bsp.h"
#include "common.h"
#include "test_gpio.h"

// from local
#include "ha_hhp_interface_bsp.h"


/* ******************************   Macros   ****************************** */

#ifdef _18F46K40
    // Master ready to send line
    // X6: Pin 6
    #define COMMS_MASTER_RTS_ACTIVE_VAL     (GPIO_LOW)
    #define COMMS_MASTER_RTS_INACTIVE_VAL   (GPIO_HIGH)
    #define COMMS_MASTER_RTS_IS_ACTIVE()	(PORTCbits.RC2 == COMMS_MASTER_RTS_ACTIVE_VAL)
    #define COMMS_MASTER_RTS_INIT()			INLINE_EXPR(TRISCbits.TRISC2 = GPIO_BIT_INPUT; ANSELCbits.ANSELC2 = 0)
    #define COMMS_MASTER_RTS_DEINIT()

    // Slave ready to send/clear to send line
    // X6: Pin 2
    #define COMMS_RTS_CTS_ACTIVE_VAL		(GPIO_LOW)
    #define COMMS_RTS_CTS_INACTIVE_VAL		(GPIO_HIGH)
    #define COMMS_RTS_CTS_IS_ACTIVE()		(LATBbits.LATB0 == COMMS_RTS_CTS_ACTIVE_VAL)
    #define COMMS_RTS_CTS_SET(active)		INLINE_EXPR(LATBbits.LATB0 = active ? COMMS_RTS_CTS_ACTIVE_VAL : COMMS_RTS_CTS_INACTIVE_VAL)
    #define COMMS_RTS_CTS_TOGGLE()			INLINE_EXPR(COMMS_RTS_CTS_SET(!COMMS_RTS_CTS_IS_ACTIVE()))
    #define COMMS_RTS_CTS_INIT()			INLINE_EXPR(TRISBbits.TRISB0 = GPIO_BIT_OUTPUT; COMMS_RTS_CTS_SET(false); ANSELBbits.ANSELB0 = 0)
    #define COMMS_RTS_CTS_DEINIT()			INLINE_EXPR(TRISBbits.TRISB0 = GPIO_BIT_INPUT)

    // RX/TX data line
    // X6: Pin 3
    #define COMMS_DATA_ACTIVE_VAL			(GPIO_HIGH)
    #define COMMS_DATA_INACTIVE_VAL			(GPIO_LOW)
    #define COMMS_DATA_OUT_IS_HIGH()		(LATEbits.LATE2 == GPIO_HIGH)
    #define COMMS_DATA_IN_IS_HIGH()			(PORTEbits.RE2 == GPIO_HIGH)
    #define COMMS_DATA_SET(active)			INLINE_EXPR(LATEbits.LATE2 = active ? COMMS_DATA_ACTIVE_VAL : COMMS_DATA_INACTIVE_VAL)
    #define COMMS_DATA_TOGGLE()				INLINE_EXPR(COMMS_DATA_SET(!COMMS_DATA_IS_ACTIVE()))
    #define COMMS_DATA_CONFIG_RX()			INLINE_EXPR(TRISEbits.TRISE2 = GPIO_BIT_INPUT; ANSELEbits.ANSELE2 = 0)
    #define COMMS_DATA_CONFIG_TX()			INLINE_EXPR(TRISEbits.TRISE2 = GPIO_BIT_OUTPUT; ANSELEbits.ANSELE2 = 0)

    // Clock line
    // X6: Pin 5
    #define COMMS_CLK_ACTIVE_VAL			(GPIO_HIGH)
    #define COMMS_CLK_INACTIVE_VAL			(GPIO_LOW)
    #define COMMS_CLK_OUT_IS_HIGH()			(LATEbits.LATE0 == GPIO_HIGH)
    #define COMMS_CLK_IN_IS_HIGH()			(PORTEbits.RE0 == GPIO_HIGH)
    #define COMMS_CLK_SET(active)			INLINE_EXPR(LATEbits.LATE0 = active ? COMMS_CLK_ACTIVE_VAL : COMMS_CLK_INACTIVE_VAL)
    #define COMMS_CLK_TOGGLE()				INLINE_EXPR(COMMS_CLK_SET(!COMMS_CLK_IS_ACTIVE()))
    #define COMMS_CLK_CONFIG_RX()			INLINE_EXPR(TRISEbits.TRISE0 = GPIO_BIT_INPUT; ANSELEbits.ANSELE0 = 0)
    #define COMMS_CLK_CONFIG_TX()			INLINE_EXPR(TRISEbits.TRISE0 = GPIO_BIT_OUTPUT; ANSELEbits.ANSELE0 = 0)
#else
    // Master ready to send line
    // X6: Pin 2
    #define COMMS_MASTER_RTS_ACTIVE_VAL     (GPIO_LOW)
    #define COMMS_MASTER_RTS_INACTIVE_VAL   (GPIO_HIGH)
    #define COMMS_MASTER_RTS_IS_ACTIVE()	(PORTBbits.RB0 == COMMS_MASTER_RTS_ACTIVE_VAL)
    #define COMMS_MASTER_RTS_INIT()			INLINE_EXPR(TRISBbits.TRISB0 = GPIO_BIT_INPUT)
    #define COMMS_MASTER_RTS_DEINIT()

    // Slave ready to send/clear to send line
    // X6: Pin 6
    #define COMMS_RTS_CTS_ACTIVE_VAL		(GPIO_LOW)
    #define COMMS_RTS_CTS_INACTIVE_VAL		(GPIO_HIGH)
    #define COMMS_RTS_CTS_IS_ACTIVE()		(LATCbits.LATC2 == COMMS_RTS_CTS_ACTIVE_VAL)
    #define COMMS_RTS_CTS_SET(active)		INLINE_EXPR(LATCbits.LATC2 = active ? COMMS_RTS_CTS_ACTIVE_VAL : COMMS_RTS_CTS_INACTIVE_VAL)
    #define COMMS_RTS_CTS_TOGGLE()			INLINE_EXPR(COMMS_RTS_CTS_SET(!COMMS_RTS_CTS_IS_ACTIVE()))
    #define COMMS_RTS_CTS_INIT()			INLINE_EXPR(TRISCbits.TRISC2 = GPIO_BIT_OUTPUT; COMMS_RTS_CTS_SET(false))
    #define COMMS_RTS_CTS_DEINIT()			INLINE_EXPR(TRISCbits.TRISC2 = GPIO_BIT_INPUT)

    // RX/TX data line
    // X6: Pin 3
    #define COMMS_DATA_ACTIVE_VAL			(GPIO_HIGH)
    #define COMMS_DATA_INACTIVE_VAL			(GPIO_LOW)
    #define COMMS_DATA_OUT_IS_HIGH()		(LATEbits.LATE2 == GPIO_HIGH)
    #define COMMS_DATA_IN_IS_HIGH()			(PORTEbits.RE2 == GPIO_HIGH)
    #define COMMS_DATA_SET(active)			INLINE_EXPR(LATEbits.LATE2 = active ? COMMS_DATA_ACTIVE_VAL : COMMS_DATA_INACTIVE_VAL)
    #define COMMS_DATA_TOGGLE()				INLINE_EXPR(COMMS_DATA_SET(!COMMS_DATA_IS_ACTIVE()))
    #define COMMS_DATA_CONFIG_RX()			INLINE_EXPR(TRISEbits.TRISE2 = GPIO_BIT_INPUT)
    #define COMMS_DATA_CONFIG_TX()			INLINE_EXPR(TRISEbits.TRISE2 = GPIO_BIT_OUTPUT)

    // Clock line
    // X6: Pin 5
    #define COMMS_CLK_ACTIVE_VAL			(GPIO_HIGH)
    #define COMMS_CLK_INACTIVE_VAL			(GPIO_LOW)
    #define COMMS_CLK_OUT_IS_HIGH()			(LATEbits.LATE0 == GPIO_HIGH)
    #define COMMS_CLK_IN_IS_HIGH()			(PORTEbits.RE0 == GPIO_HIGH)
    #define COMMS_CLK_SET(active)			INLINE_EXPR(LATEbits.LATE0 = active ? COMMS_CLK_ACTIVE_VAL : COMMS_CLK_INACTIVE_VAL)
    #define COMMS_CLK_TOGGLE()				INLINE_EXPR(COMMS_CLK_SET(!COMMS_CLK_IS_ACTIVE()))
    #define COMMS_CLK_CONFIG_RX()			INLINE_EXPR(TRISEbits.TRISE0 = GPIO_BIT_INPUT)
    #define COMMS_CLK_CONFIG_TX()			INLINE_EXPR(TRISEbits.TRISE0 = GPIO_BIT_OUTPUT)
#endif

/* ******************************   Types   ******************************* */

typedef enum
{
	PIN_ID_MASTER_RTS,
	PIN_ID_RTS_CTS,
	PIN_ID_DATA,
	PIN_ID_CLK
} HaHhpPinId_t;

/* ***********************   Function Prototypes   ************************ */

static void StartPacketTransmit(void);
static void ConfigureIoForResponse(void);
static void ConfigureIoForNewTransaction(void);
static void TxByte(uint8_t tx_byte);
static inline bool WaitForDataPinToGoHigh(uint8_t timeout_cycles);
static inline bool WaitForDataPinToGoLow(uint8_t timeout_cycles);
static inline bool WaitForClkPinToGoHigh(uint8_t timeout_cycles);
static inline bool WaitForClkPinToGoLow(uint8_t timeout_cycles);
static inline bool WaitForMstRtsPinToGoActive(uint8_t timeout_cycles);
static inline bool WaitForMstRtsPinToGoInactive(uint8_t timeout_cycles);

#if defined(HA_HHP_COMMS_BSP_TIMING_TEST) || defined(HA_HHP_COMMS_TEST_BASIC_HARDWARE) || defined(HA_HHP_BSP_DELAY_TEST)
    static void RunTests(void);
    
	#if defined(HA_HHP_COMMS_BSP_TIMING_TEST)
		static void BspTimingTest(void);
	#elif defined(HA_HHP_BSP_DELAY_TEST)
		static void DelayTimingTest(void);
	#elif defined(HA_HHP_COMMS_TEST_BASIC_HARDWARE)
		static void BasicHardwareTest(void);
	#endif
#endif

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: haHhpBsp_Init
//
// Description: Initializes this module
//
//-------------------------------
void haHhpBsp_Init(void)
{
	COMMS_MASTER_RTS_INIT();
	COMMS_RTS_CTS_INIT();
	COMMS_DATA_CONFIG_RX();
	COMMS_CLK_CONFIG_RX();
	
#if defined(HA_HHP_COMMS_BSP_TIMING_TEST) || defined(HA_HHP_COMMS_TEST_BASIC_HARDWARE) || defined(HA_HHP_BSP_DELAY_TEST)
	RunTests();
#endif
}

//-------------------------------
// Function: haHhpBsp_ReadyToReceivePacket
//
// Description: Waits for the system to be in a valid state to receive a packet from the HHP.
//
//-------------------------------
bool haHhpBsp_ReadyToReceivePacket(void)
{
	bool ret_val;

	// Clock and data must be high at this point
	if (WaitForDataPinToGoHigh(1) && WaitForClkPinToGoHigh(1))
	{
		ret_val = true;
	}
	else
	{
		ret_val = false;
	}

	return ret_val;
}

//-------------------------------
// Function: haHhpBsp_TransmitPacket
//
// Description: Sends a packet from slave to master device.
//
//-------------------------------
void haHhpBsp_TransmitPacket(uint8_t *tx_pkt, uint8_t len)
{
	if (WaitForMstRtsPinToGoInactive(HA_HHP_LINE_INPUT_TO_100_us))
	{
		StartPacketTransmit();

		// Send all bytes, waiting a small amount of time in between each to allow master device
		// to process each byte.
		for (unsigned int i = 0; i < len; i++)
		{
			TxByte(tx_pkt[i]);
		}

		// Set data and clock lines back to inputs.
		ConfigureIoForNewTransaction();
		
		// Let master know that the slave device is giving up control of the communications channel.
		COMMS_RTS_CTS_SET(false);
	}
}

//-------------------------------
// Function: haHhpBsp_RxByte
//
// Description: Receives a single byte from master device
//
// return: true: read in byte, false: timed out trying to read in byte
//-------------------------------
bool haHhpBsp_RxByte(uint8_t *rxd_byte)
{
	unsigned int num_bits_read_in = 0;
	bool ret_val = true;

	*rxd_byte = 0;

	while ((num_bits_read_in < 8) && ret_val)
	{
		// Wait for the clock input to go high.
		ret_val = WaitForClkPinToGoHigh(HA_HHP_LINE_INPUT_TO_100_us);

		if (ret_val)
		{
			// Wait for the clock line to go low.
			ret_val = WaitForClkPinToGoLow(HA_HHP_LINE_INPUT_TO_100_us);
			
            if (ret_val)
            {
                // Didn't time out.
                // Shove bit value into the byte
                *rxd_byte <<= 1;
                if (COMMS_DATA_IN_IS_HIGH())
                {
                    *rxd_byte |= 0x01;
                }
                else
                {
                    *rxd_byte &= 0xFE;
                }

                num_bits_read_in++;
            }
		}
	}

	return ret_val;
}

//-------------------------------
// Function: haHhpBsp_MasterRtsAsserted
//
// Description: Let's caller know whether or not the master device's RTS line is asserted.
// 		Meaning, figuring out whether the master device is idle or if it wants to communicate.
//
//-------------------------------
bool haHhpBsp_MasterRtsAsserted(void)
{
	return COMMS_MASTER_RTS_IS_ACTIVE();
}

//-------------------------------
// Function: haHhpBsp_SlaveReadyToReceivePacket
//
// Description: Let master device know that the slave is ready to process the packet.
//
//-------------------------------
void haHhpBsp_SlaveReadyToReceivePacket(void)
{
	// Acting as CTS.
	COMMS_RTS_CTS_SET(true);
	bspDelayUs(US_DELAY_100_us);
	COMMS_RTS_CTS_SET(false);
}

/* ********************   Private Function Definitions   ****************** */

//-------------------------------
// Function: StartPacketTransmit
//
// Description: Starts a packet transmission from slave to master
//
//-------------------------------
static void StartPacketTransmit(void)
{
	ConfigureIoForResponse();

	// Drive clock and data lines low.
	COMMS_DATA_SET(true);
	COMMS_CLK_SET(true);

	// Let master know that the slave device is about to take control of the communications channel.
	// Acting as RTS.
	COMMS_RTS_CTS_SET(true);

	// Wait for the master device to configure data and clock lines as inputs.
	bspDelayUs(US_DELAY_50_us);
}

//-------------------------------
// Function: ConfigureIoForResponse
//
// Description: Configures hardware to prepare for response to/from current transaction.
//-------------------------------
static void ConfigureIoForResponse(void)
{
	COMMS_DATA_CONFIG_TX();
	COMMS_CLK_CONFIG_TX();
}

//-------------------------------
// Function: ConfigureIoForNewTransaction
//
// Description: Configures hardware to prepare for a new transaction
//-------------------------------
static void ConfigureIoForNewTransaction(void)
{
	COMMS_DATA_CONFIG_RX();
	COMMS_CLK_CONFIG_RX();
}

//-------------------------------
// Function: TxByte
//
// Description: Transmit a single byte of data over the communications link.
//
// NOTE: The hardware on both slave and master devices must be set up before this function is called
// NOTE: to allow the slave to transmit or this function fails to do the only thing it's meant to do.
//
//-------------------------------
static void TxByte(uint8_t tx_byte)
{
	for (unsigned int i = 0; i < 8; i++)
	{
		COMMS_DATA_SET((tx_byte & 0x80) > 0);
		COMMS_CLK_SET(true);
		bspDelayUs(US_DELAY_50_us);
		COMMS_CLK_SET(false);
		bspDelayUs(US_DELAY_50_us - 4);
		
		tx_byte <<= 1;
	}
}

//-------------------------------
// Function: WaitForDataPinToGoHigh
//
// Description: Has program wait until the data pin is high. Inlined because function calls take forever
//		to execute in comparison.
//
// NOTE: The hardware on both slave and master devices must be set up before this function is called
// NOTE: to allow the slave to transmit or this function fails to do the only thing it's meant to do.
//
//-------------------------------
static inline bool WaitForDataPinToGoHigh(uint8_t timeout_cycles)
{
	for (unsigned int i = 0; (i < timeout_cycles) && !COMMS_DATA_IN_IS_HIGH(); i++)
	{
	}

	return COMMS_DATA_IN_IS_HIGH();
}

//-------------------------------
// Function: WaitForDataPinToGoLow
//
// Description: Has program wait until the data pin is low. Inlined because function calls take forever
//		to execute in comparison.
//
// NOTE: The hardware on both slave and master devices must be set up before this function is called
// NOTE: to allow the slave to transmit or this function fails to do the only thing it's meant to do.
//
//-------------------------------
static inline bool WaitForDataPinToGoLow(uint8_t timeout_cycles)
{
	for (unsigned int i = 0; (i < timeout_cycles) && COMMS_DATA_IN_IS_HIGH(); i++)
	{
	}

	return !COMMS_DATA_IN_IS_HIGH();
}

//-------------------------------
// Function: WaitForClkPinToGoHigh
//
// Description: Has program wait until the clock pin is high. Inlined because function calls take forever
//		to execute in comparison.
//
// NOTE: The hardware on both slave and master devices must be set up before this function is called
// NOTE: to allow the slave to transmit or this function fails to do the only thing it's meant to do.
//
//-------------------------------
static inline bool WaitForClkPinToGoHigh(uint8_t timeout_cycles)
{
	for (unsigned int i = 0; (i < timeout_cycles) && !COMMS_CLK_IN_IS_HIGH(); i++)
	{
	}

	return COMMS_CLK_IN_IS_HIGH();
}

//-------------------------------
// Function: WaitForClkPinToGoLow
//
// Description: Has program wait until the clock pin is low. Inlined because function calls take forever
//		to execute in comparison.
//
// NOTE: The hardware on both slave and master devices must be set up before this function is called
// NOTE: to allow the slave to transmit or this function fails to do the only thing it's meant to do.
//
//-------------------------------
static inline bool WaitForClkPinToGoLow(uint8_t timeout_cycles)
{
	for (unsigned int i = 0; (i < timeout_cycles) && COMMS_CLK_IN_IS_HIGH(); i++)
	{
	}

	return !COMMS_CLK_IN_IS_HIGH();
}

//-------------------------------
// Function: WaitForMstRtsPinToGoHigh
//
// Description: Has program wait until the master RTS pin is active. Inlined because function calls take forever
//		to execute in comparison.
//
// NOTE: The hardware on both slave and master devices must be set up before this function is called
// NOTE: to allow the slave to transmit or this function fails to do the only thing it's meant to do.
//
//-------------------------------
static inline bool WaitForMstRtsPinToGoActive(uint8_t timeout_cycles)
{
	for (unsigned int i = 0; (i < timeout_cycles) && !COMMS_MASTER_RTS_IS_ACTIVE(); i++)
	{
	}

	return COMMS_MASTER_RTS_IS_ACTIVE();
}

//-------------------------------
// Function: WaitForMstRtsPinToGoLow
//
// Description: Has program wait until the master RTS pin is inactive. Inlined because function calls take forever
//		to execute in comparison.
//
// NOTE: The hardware on both slave and master devices must be set up before this function is called
// NOTE: to allow the slave to transmit or this function fails to do the only thing it's meant to do.
//
//-------------------------------
static inline bool WaitForMstRtsPinToGoInactive(uint8_t timeout_cycles)
{
	for (unsigned int i = 0; (i < timeout_cycles) && COMMS_MASTER_RTS_IS_ACTIVE(); i++)
	{
	}

	return !COMMS_MASTER_RTS_IS_ACTIVE();
}

#if defined(HA_HHP_COMMS_BSP_TIMING_TEST) || defined(HA_HHP_COMMS_TEST_BASIC_HARDWARE) || defined(HA_HHP_BSP_DELAY_TEST)
//-------------------------------
// Function: RunTests
//
// Description: Runs a BSP level test(s)
//
//-------------------------------
static void RunTests(void)
{
#if defined(HA_HHP_COMMS_BSP_TIMING_TEST)
	BspTimingTest();
	while (1);
#elif defined(HA_HHP_BSP_DELAY_TEST)
	DelayTimingTest();
	while(1);
#elif defined(HA_HHP_COMMS_TEST_BASIC_HARDWARE)
	BasicHardwareTest();
#endif
}

//-------------------------------
// Function: BspTimingTest
//
// Description: Check the timing of each input "check and wait with timeout" function
//
// NOTE: Must connect test GPIO1 to data and clk lines.
//
//-------------------------------
#if defined(HA_HHP_COMMS_BSP_TIMING_TEST)
static void BspTimingTest(void)
{
	// Figure out how long it takes to drive the IO low to be able to subtract that value from
	// each reading.
	{
		for (int j = 0; j < 3; j++)
		{
			testGpioSet(TEST_GPIO_0, true);
			testGpioSet(TEST_GPIO_0, false);
		}
	}

	for (int i = 0; i < 100; i++);

	// Check timing for data high check
	{
		// Check timing of success
		testGpioSet(TEST_GPIO_1, true);

		testGpioSet(TEST_GPIO_0, true);
		(void)WaitForDataPinToGoHigh(1);
		testGpioSet(TEST_GPIO_0, false);

		testGpioSet(TEST_GPIO_1, false);
		for (int i = 0; i < 5; i++);

		// Check timing of timeouts
		for (int j = 1; j < 6; j++)
		{
			testGpioSet(TEST_GPIO_0, true);
			(void)WaitForDataPinToGoHigh(j);
			testGpioSet(TEST_GPIO_0, false);
			for (int i = 0; i < 5; i++);
		}
	}
    
	for (int i = 0; i < 100; i++);

	// Check timing for data low check
	{
		// Check timing of success
		testGpioSet(TEST_GPIO_1, false);

		testGpioSet(TEST_GPIO_0, true);
		(void)WaitForDataPinToGoLow(1);
		testGpioSet(TEST_GPIO_0, false);

		testGpioSet(TEST_GPIO_1, true);
		for (int i = 0; i < 5; i++);

		for (int j = 1; j < 6; j++)
		{
			testGpioSet(TEST_GPIO_0, true);
			(void)WaitForDataPinToGoLow(j);
			testGpioSet(TEST_GPIO_0, false);
			for (int i = 0; i < 5; i++);
		}
	}
    
	for (int i = 0; i < 100; i++);

	// Check timing for clock high check
	{
		// Check timing of success
		testGpioSet(TEST_GPIO_1, true);

		testGpioSet(TEST_GPIO_0, true);
		(void)WaitForClkPinToGoHigh(1);
		testGpioSet(TEST_GPIO_0, false);

		testGpioSet(TEST_GPIO_1, false);
		for (int i = 0; i < 5; i++);

		for (int j = 1; j < 6; j++)
		{
			testGpioSet(TEST_GPIO_0, true);
			(void)WaitForClkPinToGoHigh(j);
			testGpioSet(TEST_GPIO_0, false);
			for (int i = 0; i < 5; i++);
		}
	}
    
	for (int i = 0; i < 100; i++);

	// Check timing for clock low check
	{
		// Check timing of success
		testGpioSet(TEST_GPIO_1, false);

		testGpioSet(TEST_GPIO_0, true);
		(void)WaitForClkPinToGoLow(1);
		testGpioSet(TEST_GPIO_0, false);

		testGpioSet(TEST_GPIO_1, true);
		for (int i = 0; i < 5; i++);

		for (int j = 1; j < 6; j++)
		{
			testGpioSet(TEST_GPIO_0, true);
			(void)WaitForClkPinToGoLow(j);
			testGpioSet(TEST_GPIO_0, false);
			for (int i = 0; i < 5; i++);
		}
	}
}

#elif defined(HA_HHP_BSP_DELAY_TEST)

//-------------------------------
// Function: DelayTimingTest
//
// Description: Use to figure out what values are needed for various delays for microsecond delay.
//
//-------------------------------
static void DelayTimingTest(void)
{
	// Figure out how long it takes to drive the IO low to be able to subtract that value from
	// each reading.
	{
		for (int j = 0; j < 3; j++)
		{
			testGpioSet(TEST_GPIO_0, true);
			testGpioSet(TEST_GPIO_0, false);
		}
	}

	for (int i = 0; i < 100; i++);

	// Check timing of timeouts for values <256
	for (int j = 1; j < 6; j++)
	{
		testGpioSet(TEST_GPIO_0, true);
		bspDelayUs(j);
		testGpioSet(TEST_GPIO_0, false);
		for (int i = 0; i < 5; i++);
	}

	// Check timing of timeouts for values >256
	for (int j = 256; j < 261; j++)
	{
		testGpioSet(TEST_GPIO_0, true);
		bspDelayUs(j);
		testGpioSet(TEST_GPIO_0, false);
		for (int i = 0; i < 5; i++);
	}
}

#elif defined(HA_HHP_COMMS_TEST_BASIC_HARDWARE)

static volatile bool rts_state = false;
static volatile bool data_state = false;
static volatile bool clk_state = false;

//-------------------------------
// Function: BasicHardwareTest
//
// Description: Runs basic connectivity/hook-up test between HA and HHP
//
// NOTE: Ensure an HHP is hooked up and also programmed to be in the proper test module before running this test.
//
//-------------------------------
static void BasicHardwareTest(void)
{
	ConfigureIoForResponse();
	
	while (1)
	{
#if 1
		// Input test
		rts_state = COMMS_MASTER_RTS_IS_ACTIVE();
		clk_state = COMMS_CLK_IN_IS_HIGH();
		data_state = COMMS_DATA_IN_IS_HIGH();
#else
		// Output test
		COMMS_RTS_CTS_SET(true);
		COMMS_CLK_SET(true);
		COMMS_DATA_SET(true);

		COMMS_RTS_CTS_SET(false);
		COMMS_CLK_SET(false);
		COMMS_DATA_SET(false);
#endif
	}
}
#endif
#endif

// end of file.
//-------------------------------------------------------------------------
