//////////////////////////////////////////////////////////////////////////////
//
// Filename: ha_hhp_interface_app.c
//
// Description: App level definition for the comms interface between a head array and HHP.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

/* ***************************    Includes     **************************** */

// NOTE: This must ALWAYS be the first include in a file.
#include "device.h"

// from RTOS
#include "cocoos.h"

// from stdlib
#include <stdint.h>
#include <stdbool.h>
#include "user_assert.h"

// from project
#include "rtos_task_priorities.h"
#include "bsp.h"
#include "common.h"
#include "isrs.h"
#include "test_gpio.h"
#include "version.h"
#include "eeprom_app.h"
#include "app_common.h"
#include "config.h"

// from local
#include "ha_hhp_interface_bsp.h"
#include "ha_hhp_interface.h"
#include "ha_hhp_interface_app.h"

/* ******************************   Types   ******************************* */

#ifdef OK_TO_USE_HHP_CODE

typedef enum
{
	HA_HHP_CMD_PAD_ASSIGNMENT_GET = 0x30,
	HA_HHP_CMD_PAD_ASSIGNMENT_SET = 0x31,
	HA_HHP_CMD_CAL_RANGE_GET = 0x32,
	HA_HHP_CMD_CAL_RANGE_SET = 0x33,
	HA_HHP_CMD_PAD_CALIBRATION_SESSION_START = 0x34,
	HA_HHP_CMD_PAD_CALIBRATION_SESSION_STOP = 0x35,
	HA_HHP_CMD_PAD_DATA_GET = 0x36,
	HA_HHP_CMD_VERSION_GET = 0x37,
	HA_HHP_CMD_ENABLED_FEATURES_GET = 0x38,
	HA_HHP_CMD_ENABLED_FEATURES_SET = 0x39,
	HA_HHP_CMD_MODE_OF_OPERATION_SET = 0x3A,
	HA_HHP_CMD_HEARTBEAT = 0x3B,
    HA_HHP_CMD_NEUTRAL_DAC_GET = 0x3C,
    HA_HHP_CMD_NEUTRAL_DAC_SET = 0x3D,
    HA_HHP_CMD_SAVE_PARAMETERS = 0x3E,
    HA_HHP_CMD_RESET_PARAMETERS = 0x3F,
    HA_HHP_CMD_DRIVE_OFFSET_GET = 0x40,
    HA_HHP_CMD_DRIVE_OFFSET_SET = 0x41
} HaHhpIfCmd_t;

// Slave responses to commands from master.
typedef enum
{
	HA_HHP_RESP_ACK = 0x06,
	HA_HHP_RESP_NACK = 0x15
} HaHhpIfResp_t;

/* ******************************   Macros   ****************************** */

#define COMMS_PAD_SEL_LEFT 			('L')
#define COMMS_PAD_SEL_RIGHT 		('R')
#define COMMS_PAD_SEL_CTR			('C')

#define COMMS_OUTPUT_SEL_LEFT 		('L')
#define COMMS_OUTPUT_SEL_RIGHT 		('R')
#define COMMS_OUTPUT_SEL_FWD		('F')
#define COMMS_OUTPUT_SEL_BACKWARDS	('B')
#define COMMS_OUTPUT_SEL_OFF		('O')

#define COMMS_PAD_TYPE_DIGITAL 		('D')
#define COMMS_PAD_TYPE_PROPORTIONAL ('P')


//-------------------------------
// Normally these macros would be functions, but we're flash space constrained on this device
//-------------------------------

//-------------------------------
// Function: BuildEnabledFeaturesResponsePacket
//
// Description: Builds up a packet to send as a response to a HA_HHP_CMD_ENABLED_FEATURES_GET command.
//
//-------------------------------
/* ***********************   File Scope Variables   *********************** */

static uint8_t hhp_rx_data_buff[HHP_RX_TX_BUFF_LEN];
static uint8_t hhp_tx_pkt_buff[HHP_RX_TX_BUFF_LEN];

static volatile uint8_t ha_hhp_if_task_id;

/* ***********************   Function Prototypes   ************************ */

static void HaHhpInterfaceHandlingTask(void);
static void ProcessRxdPacket(uint8_t *rxd_pkt, uint8_t *pkt_to_tx);
static void BuildAckPacket(uint8_t *pkt_to_tx);
static void BuildNackPacket(uint8_t *pkt_to_tx);
static void BuildVersionsResponsePacket(uint8_t *pkt_to_tx);
static void HandlePadAssignmentGet(uint8_t *rxd_pkt, uint8_t *pkt_to_tx);
static void HandlePadAssignmentSet(uint8_t *rxd_pkt, uint8_t *pkt_to_tx);
static void HandlePadCalibrationDataGet(uint8_t *rxd_pkt, uint8_t *pkt_to_tx);
static void HandlePadCalibrationDataSet(uint8_t *rxd_pkt, uint8_t *pkt_to_tx);
static void HandlePadDataGet(uint8_t *rxd_pkt, uint8_t *pkt_to_tx);
static void BuildEnabledFeaturesResponsePacket(uint8_t *pkt_to_tx);
static void HandleEnabledFeaturesSet(uint8_t *rxd_pkt, uint8_t *pkt_to_tx);
static void HandleModeOfOpSet(uint8_t *rxd_pkt, uint8_t *pkt_to_tx);
static void BuildHeartBeatResponsePacket(uint8_t *rxd_pkt, uint8_t *pkt_to_tx);
static void Buid_DAC_Get_Response(uint8_t *rxd_pkt, uint8_t *pkt_to_tx);
static void Handle_DAC_SetCommand(uint8_t *rxd_pkt, uint8_t *pkt_to_tx);
static uint8_t TranslateInputToOutputMapValFromEnum(HeadArrayOutputFunction_t output_assignment);
static HeadArrayOutputFunction_t TranslateInputToOutputMapValToEnum(uint8_t output_assignment);
static HeadArrayInputType_t TranslateInputTypeValToEnum(uint8_t input_type);
static uint8_t CalcChecksum(uint8_t *packet, uint8_t len);
static void CreateGetOffsetResponse (uint8_t *pkt_to_tx);
static void CreateSetOffsetResponse (uint8_t *rxd_pkt, uint8_t *pkt_to_tx);

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: haHhpApp_Init
//
// Description: Intializes this module.
//
//-------------------------------
void haHhpApp_Init(void)
{
	haHhp_Init();

    ha_hhp_if_task_id = task_create(HaHhpInterfaceHandlingTask, NULL, HA_HHP_IF_MGMT_TASK_PRIO, NULL, 0, 0);
}

/* ********************   Private Function Definitions   ****************** */

//-------------------------------
// Function: HaHhpInterfaceHandlingTask
//
// Description: Handles monitoring, reception, and response to packets from HHP.
//
//-------------------------------
static void HaHhpInterfaceHandlingTask(void)
{
    task_open();
	while (1)
    {
		if (haHhpBsp_MasterRtsAsserted())
		{
			// Receive, process, and respond to the packet without fear of OS ticks in
			// and mucking with timing.  Note: sys tick is active, but only processes the
			// absolute essentials.
			isrsOsTickEnable(false);

			if (haHhpBsp_ReadyToReceivePacket())
			{
				haHhpBsp_SlaveReadyToReceivePacket();

				if(haHhp_RxPacket(hhp_rx_data_buff))
				{
					ProcessRxdPacket(hhp_rx_data_buff, hhp_tx_pkt_buff);

					// There's always a response from slave->master
					haHhpBsp_TransmitPacket(hhp_tx_pkt_buff, hhp_tx_pkt_buff[0]);
				}
			}
			
			isrsOsTickEnable(true);
		}

		// Need to keep the wait very short, but need to wait.
		task_wait(MILLISECONDS_TO_TICKS(1));
	}
    task_close();
}

//-------------------------------
// Function: ProcessRxdPacket
//
// Description: Processes a received packet from a master device (HHP)
//
// Basic command->response format:
// Request from master: <LEN><DATA><CHKSUM>
// Response from slave: <LEN><DATA><CHKSUM>
//
//-------------------------------
static void ProcessRxdPacket(uint8_t *rxd_pkt, uint8_t *pkt_to_tx)
{
    uint8_t myData[8];
    
	// Don't include the checksum value when computing the checksum.
	// If this dewvice is in idle mode, then command processing outside of heartbeat is disabled.
//	if ((AppCommonDeviceActiveGet() || (!AppCommonDeviceActiveGet() && (HA_HHP_CMD_HEARTBEAT == (HaHhpIfCmd_t)rxd_pkt[1]))) &&
//		(rxd_pkt[0] <= HHP_RX_TX_BUFF_LEN) &&
//		(CalcChecksum(rxd_pkt, rxd_pkt[0] - 1) == rxd_pkt[rxd_pkt[0] - 1]))
	if (CalcChecksum(rxd_pkt, rxd_pkt[0] - 1) == rxd_pkt[rxd_pkt[0] - 1])
	{
		// At a basic structural level, the packet is constructed properly.
		switch((HaHhpIfCmd_t)rxd_pkt[1])
		{
			// Gets pad left/right/forward/backwards, and proportional/digital input assignments
			case HA_HHP_CMD_PAD_ASSIGNMENT_GET:
				// Received packet structure:
				// <LEN><PAD_ASSIGNMENT_GET_CMD><PHYSCIAL PAD><CHKSUM>
				//
				// Response packet structure:
				// <LEN><LOGICAL PAD><SENSOR TYPE><CHKSUM> 
				// or
				// <LEN><NACK><CHKSUM>
				//
				// Where:	<PAD_ASSIGNMENT_GET> = 0x30
				// 			<PHYSICAL PAD> = "L", "R", "C"
				// 			<LOGICAL PAD> = "F", "R", "L", "B", "O"
				// 			<SENSOR TYPE> = "D", "P"
				HandlePadAssignmentGet(rxd_pkt, pkt_to_tx);
				break;
			
			// Sets pad left/right/forward/backwards, and proportional/digital input assignments
			case HA_HHP_CMD_PAD_ASSIGNMENT_SET:
				// Received packet structure:
				// <LEN><PAD_ASSIGNMENT_SET_CMD><PHYSCIAL_PAD><LOGICAL_PAD><SENSOR TYPE><CHKSUM>
				//
				// Response packet structure:
				// <LEN><ACK/NACK><CHKSUM>
				//
				// Where:	<PAD_ASSIGNMENT_SET_CMD> = 0x31
				// 			<PHYSICAL_PAD> = "L", "R", "C"
				// 			<LOGICAL_PAD> = "F", "R", "L", "B", "O" 
				// 			<SENSOR TYPE> = "D", "P" 
				//  		<ACK/NAK> = ACK if all is good, NACK if packet is malformed on reception.	
				HandlePadAssignmentSet(rxd_pkt, pkt_to_tx);
				BuildAckPacket(pkt_to_tx);
				break;

			// Sets a pad's input min/max raw input value as well as the min/max threshold values for input->output translation.
			case HA_HHP_CMD_CAL_RANGE_GET:
				// Received packet structure:
				// <LEN><CAL_RANGE_GET_CMD><PHYSICAL_PAD><CHKSUM>
				//
				// Response packet structure:
				// <LEN><PHYSICAL_PAD><MIN_ADC><MAX_ADC><MIN_THRESH><MAX_THRESH><CHKSUM> 
				// or
				// <LEN><NACK><CHKSUM>
				//
				// Where: 	<CAL_RANGE_GET_CMD> = 0x32
				// 			<PHYSICAL_PAD> = "L", "R" or "C"
				// 			<MIN_ADC> = A value in the range [0,1023].
				// 			<MAX_ADC> = A value in the range [0,1023].
				// 			<MIN_TRESH> = A value in the range [0,1023].
				// 			<MAX_THRESH> = A value in the range [0,1023].
				HandlePadCalibrationDataGet(rxd_pkt, pkt_to_tx);
				break;

			// Sets a pad's input min/max raw input value as well as the min/max threshold values for input->output translation.
			case HA_HHP_CMD_CAL_RANGE_SET:
				// Received packet structure:
				// <LEN><CAL_RANGE_SET_CMD><PHYSICAL_PAD><MIN_THRESH><MAX_THRESH><CHKSUM>
				//
				// Response packet structure:
				// <LEN><ACK/NACK><CHKSUM>
				//
				// Where:	<CAL_RANGE_SET> = 0x33
				// 			<PHYSICAL_PAD> = "L", "R" or "C" 
				// 			<MIN_TRESH> = A value in the range [0,1023].
				// 			<MAX_THRESH> = A value in the range [0,1023]. 
				// 			<ACK/NAK> = ACK if all is good, NACK if packet is malformed on reception.	
				HandlePadCalibrationDataSet(rxd_pkt, pkt_to_tx);
				BuildAckPacket(pkt_to_tx);
				break;
			
			// Start calibration session
			case HA_HHP_CMD_PAD_CALIBRATION_SESSION_START:
				// Received packet structure:
				// <LEN><CAL_START_CMD><CHKSUM>
				//
				// Response packet structure:
				// <LEN><ACK/NAK><CHKSUM>
				//
				// Where:	<CAL_START_CMD> = 0x34
				// 			<ACK/NAK> = ACK if all is good, NACK if packet is malformed on reception.
				AppCommonCalibrationActiveSet(true);
				BuildAckPacket(pkt_to_tx);
				break;

			// Stop calibration session
			case HA_HHP_CMD_PAD_CALIBRATION_SESSION_STOP:
				// Received packet structure:
				// <LEN><CAL_STOP_CMD><CHKSUM> 
				//
				// Response packet structure:
				// <LEN><ACK/NAK><CHKSUM>
				//
				// Where:	<CAL_STOP_CMD> = 0x35
				// 			<ACK/NAK> = ACK if all is good, NACK if packet is malformed on reception.

				AppCommonCalibrationActiveSet(false);
				BuildAckPacket(pkt_to_tx);
				break;

			// Get pad related information
			case HA_HHP_CMD_PAD_DATA_GET:
				// Received packet structure:
				// <LEN><PAD_DATA_GET_CMD><PHYSICAL PAD><CHKSUM>
				//
				// Response packet structure:
				// <LEN><RAW DATA><ADJUSTED DATA><CHKSUM>
				// or
				// <LEN><NACK><CHKSUM>
				// Where:	<PAD_DATA_GET_CMD> = 0x36
				// 			<PHYSICAL PAD> = "L", "R", "C"
				// 			<RAW DATA> = the actual analog/digital raw sensor data
				// 			<ADJUSTED DATA> = the processed Joystick Demand.
				HandlePadDataGet(rxd_pkt, pkt_to_tx);
				break;

			// Gets version of the head array firmware.
			case HA_HHP_CMD_VERSION_GET:
				// Received packet structure:
				// <LEN><VERSIONS_GET_CMD><CHKSUM>
				//
				// Response packet structure:
				// <LEN><MAJOR><MINOR><BUILD><DATA_VERSION><CHKSUM> 
				// or
				// <LEN><NACK><CHKSUM>
				//
				// Where: 	<REQUEST_VERSIONS_CMD> = 0x37
				// 			<MAJOR>, <MINOR>, <BUILD> Firmware version of head array represented as binary numbers.
				// 			<DATA VERSION> is the version of EEPROM data packing definition.
				BuildVersionsResponsePacket(pkt_to_tx);
				break;

			// Get currently enabled features of the head array.
			case HA_HHP_CMD_ENABLED_FEATURES_GET:
				// Received packet structure:
				// <LEN><FEATURES_GET_CMD><CHKSUM>
				//
				// Response packet structure:
				// <LEN><FEATURE_BITS><LONG_PRESS_TIME><CHKSUM>
				//
				// Where:	<REQUEST_FEATURE_SETTING_CMD> = 0x38
				// 			<FEATURE_BITS> is a bit pattern as follows:
				// 				0x01 is POWER ON/OFF setting where 1 is enabled, 0 is disabled.
				// 				0x02 is BLUETOOTH where 1 is enabled, 0 is disabled.
				// 				0x04 is NEXT FEATURE where 1 is enabled, 0 is disabled.
				// 				0x08 is NEXT PROFILE where 1 is enabled, 0 is disabled.
				// 				0x10 is the SOUND setting where 1 is enabled, 0 is disabled.
                //              0x20 is the POWER UP IN IDLE setting where 1 is enabled, 0 is disabled.
                //              0x40 is the RNet MENU setting where 1 is enabled, 0 is disabled.
				//          <LONG_PRESS_TIME> is length of time for a Long Press of the Mode Switch in tenths of seconds. "15" is 1.5 seconds.
                //          <FEATURE_BITS_2>
                //              0x01 is RNET_SLEEP, 1=enabled, 0=disabled.
                //              0x02 is MODE_REVERSE, 1=enabled, 0=disabled (Normal Pin5 operation)
				BuildEnabledFeaturesResponsePacket(pkt_to_tx);
				break;
				
			case HA_HHP_CMD_ENABLED_FEATURES_SET:
				// Received packet structure:
				// <LEN><FEATURES_SET_CMD><FEATURE_BITS><LONG_PRESS_TIME><CHKSUM>
				//
				// Response packet structure:
				// <LEN><ACK/NAK><CHKSUM>
				// Where:	<FEATURES_SET_CMD> = 0x39
				// 			<FEATURE_BITS> is a bit pattern as follows:
				// 				0x01: POWER ON/OFF, 1=enabled, 0=disabled.
				// 				0x02: BLUETOOTH, 1=enabled, 0=disabled.
				// 				0x04: NEXT FEATURE, 1=enabled, 0=disabled. 
				// 				0x08: NEXT PROFILE, 1=enabled, 0=disabled.
				// 				0x10: SOUND ENABLED, 1=enabled, 0=disabled.
                //              0x20 is the POWER UP IN IDLE setting where 1 is enabled, 0 is disabled.
                //              0x40 is the RNet MENU setting where 1 is enabled, 0 is disabled.
				// 			<LONG_PRESS_TIME> is length of time for a Long Press of the Mode Switch in tenths of seconds.
				// 					"15" is 1.5 seconds. 
                //          <FEATURE_BITS_2>
                //              0x01 is RNET_SLEEP, 1=enabled, 0=disabled.
                //              0x02 is MODE_REVERSE, 1=enabled, 0=disabled (Normal Pin5 operation)
				// 			<ACK/NAK> = ACK if all is good, NACK if packet is malformed on reception.
				HandleEnabledFeaturesSet(rxd_pkt, pkt_to_tx);
				BuildAckPacket(pkt_to_tx);
				break;

			// Set mode of operation of head array.
			case HA_HHP_CMD_MODE_OF_OPERATION_SET:
				// Received packet structure:
				// <LEN><ACTIVE_FEATURE_SET_CMD><ACTIVE_FEATURE><CHKSUM>
				//
				// Response packet structure:
				// <LEN><ACK/NAK><CHKSUM>
				// Where:	<ACTIVE_FEATURE_SET_CMD> = 0x3A
				// 			<ACTIVE_FEATURE> = May be one of the following values:
				// 				0x01: POWER ON/OFF (puts head array into idle/ready modes of operation)
				// 				0x02: BLUETOOTH
				// 				0x03: NEXT FUNCTION
				// 				0x04: NEXT PROFILE
                //              0x05: RNet DRIVING
                //              0x06: RNet SEATING.
				// 			<ACK/NAK> = ACK if all is good, NACK if packet is malformed on reception or profile disabled.
				//
				// WARN: If the values for the above functions ever change, then the values of FunctionalFeature_t
				// WARN: must be changed to match!
				HandleModeOfOpSet(rxd_pkt, pkt_to_tx);
				break;

			case HA_HHP_CMD_HEARTBEAT:
				// This is the "link status check" that is maintained by the master.
				//
				// Keeps the communications bus "alive". The master must know that this device
				// is alive, this "heartbeat" command is sent periodically to determine just that.
				//
				// Received packet structure:
				// <LEN><HEARTBEAT_CMD><VALUE><CHKSUM>
				// <LEN><VALUE><ACTIVE_FEATURE><STATUS><CHKSUM> 
				// or
				// <LEN><NACK><CHKSUM>
				// Where:	<HEARTBEAT_CMD> = 0x3B
				// 			<VALUE> = An increment byte value from 0 through 0xff. The response echoes the same <VALUE>.
				// 			<ACTIVE_FEATURE> = Current active feature responding with one of the following values:
				// 				0x01 for POWER ON/OFF (puts head array into idle/ready modes of operation)
				// 				0x02 for BLUETOOTH
				// 				0x03 for NEXT FUNCTION
				// 				0x04 for NEXT PROFILE
				// 			<STATUS> = System status of the head array that is a bit pattern as follows:
				// 				0x01: Head array is in idle/ready mode. 0=idle, 1=ready
				// 				0x02: left pad connected/disconnected status. 0=disconnected, 1=connected
				// 				0x04: right pad connected/disconnected status. 0=disconnected, 1=connected
				// 				0x08: center pad connected/disconnected status. 0=disconnected, 1=connected
				// 				0x10: Out of neutral failure (goes away once user stops pressing all pads for a 
				//  		          sufficient amount of time)
				BuildHeartBeatResponsePacket(rxd_pkt, pkt_to_tx);
				break;
            case HA_HHP_CMD_NEUTRAL_DAC_GET:
                //    Neutral DAC Get Command (0x3c
                //)
                //    <LEN><NEUTRAL_DAC_GET_CMD><CHKSUM>
                //    <LEN><NEUTRAL_CONSTANT><NEUTRAL_DAC_VALUE><DAC_RANGE><CHKSUM> 
                //    or
                //    <LEN><NACK><CHKSUM>
                //    Where: 	<NEUTRAL_DAC_GET_CMD> = 0x3c
                //              <NUETRAL_CONSTANT> = A value in the range [0,4096], typical 2048
                //              <NUETRAL_DAC_VALUE> = A value in the range [0,4096], typical 2048
                //              <DAC_RANGE> = A value in the range [0,4096], typical 410
                Buid_DAC_Get_Response(rxd_pkt, pkt_to_tx);
                break;

            case HA_HHP_CMD_NEUTRAL_DAC_SET:
                //    Neutral DAC Set Command (0x3d)
                //
                //    <LEN><NEUTRAL_DAC_SET_CMD><NEUTRAL_DAC_VALUE><CHKSUM>
                //    <LEN><NCK><CHKSUM>
                //    or
                //    <LEN><NACK><CHKSUM>
                //    Where: 	<NUETRAL_DAC_SET_CMD> = 0x3d
                //              <NUETRAL_DAC_VALUE> = A value in the range [0,4096], typical 2048
                Handle_DAC_SetCommand (rxd_pkt, pkt_to_tx);
                break;

            case HA_HHP_CMD_SAVE_PARAMETERS:
                // Save Parameter Command (0x3e)
                //
                //  <LEN><SAVE_PARAMETERS_CMD><CHKSUM>
                //  <LEN><ACK><CHKSUM>
                eepromFlush (true);      // save all parameters.
				BuildAckPacket(pkt_to_tx);
                break;

            case HA_HHP_CMD_RESET_PARAMETERS:
                // Reset Parameter Command (0x3f)
                //
                //  <LEN><RESET_PARAMETERS_CMD><CHKSUM>
                //  <LEN><ACK><CHKSUM>
                SetDefaultValues();
                eepromFlush (true);      // save all parameters.
				BuildAckPacket(pkt_to_tx);
                break;
                
            case HA_HHP_CMD_DRIVE_OFFSET_GET:
                // Get the Drive Offset Value
                //
                //  <LEN><DRIVE_OFFSET_GET_CMD><CHKSUM>
                //  <LEN><OFFSET_VALUE><CHKSUM>
                CreateGetOffsetResponse (pkt_to_tx);
                break;

            case HA_HHP_CMD_DRIVE_OFFSET_SET:
                // Set the Drive Offset Value
                //
                //  <LEN><DRIVE_OFFSET_SET_CMD><OFFSET_VALUE><CHKSUM>
                //  <LEN><ACK><CHKSUM>
				CreateSetOffsetResponse (rxd_pkt, pkt_to_tx);
                break;

			default:
                myData[0] = *rxd_pkt;
                myData[1] = *(rxd_pkt+1);
                myData[2] = *(rxd_pkt+2);
                myData[3] = *(rxd_pkt+3);
                myData[4] = *(rxd_pkt+4);
				BuildNackPacket(pkt_to_tx);
				break;
		}
	}
    else
    {
        // Packet is not constructed properly. Try to send a NACK back to the master device.
        BuildNackPacket(pkt_to_tx);
    }
	
	pkt_to_tx[pkt_to_tx[0] - 1] = CalcChecksum(pkt_to_tx, pkt_to_tx[0] - 1);
}

//-------------------------------
// Function: BuildAckPacket
//
// Description: Builds up a packet to send "ACK" as a response.
//
//-------------------------------
static void BuildAckPacket(uint8_t *pkt_to_tx)
{
	pkt_to_tx[0] = 3;
	pkt_to_tx[1] = HA_HHP_RESP_ACK;
}

//-------------------------------
// Function: BuildNackPacket
//
// Description: Builds up a packet to send "NACK" as a response.
//
//-------------------------------
static void BuildNackPacket(uint8_t *pkt_to_tx)
{
	pkt_to_tx[0] = 3;
	pkt_to_tx[1] = HA_HHP_RESP_NACK;
}

//-------------------------------
// Function: BuildVersionsResponsePacket
//
// Description: Builds up a packet to send as a response to a HA_HHP_CMD_HEARTBEAT command.
//
//-------------------------------
static void BuildVersionsResponsePacket(uint8_t *pkt_to_tx)
{
    pkt_to_tx[0] = 6;
    pkt_to_tx[1] = PROGRAM_VERSION_MAJOR;
    pkt_to_tx[2] = PROGRAM_VERSION_MINOR;
#ifdef SPECIAL_EEPROM_TO_DEFAULT_VALUES
    pkt_to_tx[3] = 99;
#else
    pkt_to_tx[3] = PROGRAM_VERSION_BUILD;
#endif
    pkt_to_tx[4] = EEPROM_DATA_STRUCTURE_VERSION;
}

//-------------------------------
// Function: HandlePadAssignmentGet
//
// Description: Digests and responds to a received HA_HHP_CMD_PAD_ASSIGNMENT_GET command.
//
//-------------------------------
static void HandlePadAssignmentGet(uint8_t *rxd_pkt, uint8_t *pkt_to_tx)
{
	HeadArrayInputType_t input_type;
	HeadArrayOutputFunction_t output_assignment;

	switch (rxd_pkt[2])
	{
		case COMMS_PAD_SEL_LEFT:
			input_type = (HeadArrayInputType_t)eepromEnumGet(EEPROM_STORED_ITEM_LEFT_PAD_INPUT_TYPE);
			output_assignment = (HeadArrayOutputFunction_t)eepromEnumGet(EEPROM_STORED_ITEM_LEFT_PAD_OUTPUT_MAP);
			break;
		
		case COMMS_PAD_SEL_RIGHT:
			input_type = (HeadArrayInputType_t)eepromEnumGet(EEPROM_STORED_ITEM_RIGHT_PAD_INPUT_TYPE);
			output_assignment = (HeadArrayOutputFunction_t)eepromEnumGet(EEPROM_STORED_ITEM_RIGHT_PAD_OUTPUT_MAP);
			break;
		
		case COMMS_PAD_SEL_CTR:
			input_type = (HeadArrayInputType_t)eepromEnumGet(EEPROM_STORED_ITEM_CTR_PAD_INPUT_TYPE);
			output_assignment = (HeadArrayOutputFunction_t)eepromEnumGet(EEPROM_STORED_ITEM_CTR_PAD_OUTPUT_MAP);
			break;
		
		default:
			// Should never be able to happen.  NACK on this anyhow.
			BuildNackPacket(pkt_to_tx);
			return; // Save some code space by doing this instead of using some "cleaner" method.
	}

	pkt_to_tx[0] = 4;
	pkt_to_tx[1] = TranslateInputToOutputMapValFromEnum(output_assignment);
	pkt_to_tx[2] = (input_type == HEAD_ARR_INPUT_DIGITAL) ? COMMS_PAD_TYPE_DIGITAL : COMMS_PAD_TYPE_PROPORTIONAL;
}

//-------------------------------
// Function: HandlePadAssignmentSet
//
// Description: Digests and responds to a received HA_HHP_CMD_PAD_ASSIGNMENT_SET command.
//
//-------------------------------
static void HandlePadAssignmentSet(uint8_t *rxd_pkt, uint8_t *pkt_to_tx)
{
    UNUSED(pkt_to_tx);
    
	if (rxd_pkt[2] == COMMS_PAD_SEL_LEFT)
	{
		eepromEnumSet(EEPROM_STORED_ITEM_LEFT_PAD_OUTPUT_MAP, (EepromStoredEnumType_t)TranslateInputToOutputMapValToEnum(rxd_pkt[3]));
		eepromEnumSet(EEPROM_STORED_ITEM_LEFT_PAD_INPUT_TYPE, (EepromStoredEnumType_t)TranslateInputTypeValToEnum(rxd_pkt[4]));
	}
	else if (rxd_pkt[2] == COMMS_PAD_SEL_RIGHT)
	{
		eepromEnumSet(EEPROM_STORED_ITEM_RIGHT_PAD_OUTPUT_MAP, (EepromStoredEnumType_t)TranslateInputToOutputMapValToEnum(rxd_pkt[3]));
		eepromEnumSet(EEPROM_STORED_ITEM_RIGHT_PAD_INPUT_TYPE, (EepromStoredEnumType_t)TranslateInputTypeValToEnum(rxd_pkt[4]));

	}
	else // COMMS_PAD_SEL_CTR
	{
		eepromEnumSet(EEPROM_STORED_ITEM_CTR_PAD_OUTPUT_MAP, (EepromStoredEnumType_t)TranslateInputToOutputMapValToEnum(rxd_pkt[3]));
		eepromEnumSet(EEPROM_STORED_ITEM_CTR_PAD_INPUT_TYPE, (EepromStoredEnumType_t)TranslateInputTypeValToEnum(rxd_pkt[4]));
	}
}

//-------------------------------
// Function: HandlePadCalibrationDataGet
//
// Description: Digests and responds to a received HA_HHP_CMD_CAL_RANGE_GET command.
//
//-------------------------------
static void HandlePadCalibrationDataGet(uint8_t *rxd_pkt, uint8_t *pkt_to_tx)
{
	TypeAccess16Bit_t t_val;
	pkt_to_tx[0] = 10;

	if (rxd_pkt[2] == COMMS_PAD_SEL_LEFT)
	{
		t_val.val = eeprom16bitGet(EEPROM_STORED_ITEM_LEFT_PAD_MIN_ADC_VAL);
		pkt_to_tx[1] = t_val.bytes[1];
		pkt_to_tx[2] = t_val.bytes[0];

		t_val.val = eeprom16bitGet(EEPROM_STORED_ITEM_LEFT_PAD_MAX_ADC_VAL);
		pkt_to_tx[3] = t_val.bytes[1];
		pkt_to_tx[4] = t_val.bytes[0];

		t_val.val = eeprom16bitGet(EEPROM_STORED_ITEM_LEFT_PAD_MIN_THRESH_PERC);
		pkt_to_tx[5] = t_val.bytes[1];
		pkt_to_tx[6] = t_val.bytes[0];

		t_val.val = eeprom16bitGet(EEPROM_STORED_ITEM_LEFT_PAD_MAX_THRESH_PERC);
		pkt_to_tx[7] = t_val.bytes[1];
		pkt_to_tx[8] = t_val.bytes[0];
	}
	else if (rxd_pkt[2] == COMMS_PAD_SEL_RIGHT)
	{
		t_val.val = eeprom16bitGet(EEPROM_STORED_ITEM_RIGHT_PAD_MIN_ADC_VAL);
		pkt_to_tx[1] = t_val.bytes[1];
		pkt_to_tx[2] = t_val.bytes[0];

		t_val.val = eeprom16bitGet(EEPROM_STORED_ITEM_RIGHT_PAD_MAX_ADC_VAL);
		pkt_to_tx[3] = t_val.bytes[1];
		pkt_to_tx[4] = t_val.bytes[0];

		t_val.val = eeprom16bitGet(EEPROM_STORED_ITEM_RIGHT_PAD_MIN_THRESH_PERC);
		pkt_to_tx[5] = t_val.bytes[1];
		pkt_to_tx[6] = t_val.bytes[0];

		t_val.val = eeprom16bitGet(EEPROM_STORED_ITEM_RIGHT_PAD_MAX_THRESH_PERC);
		pkt_to_tx[7] = t_val.bytes[1];
		pkt_to_tx[8] = t_val.bytes[0];
	}
	else // COMMS_PAD_SEL_CTR
	{
		t_val.val = eeprom16bitGet(EEPROM_STORED_ITEM_CTR_PAD_MIN_ADC_VAL);
		pkt_to_tx[1] = t_val.bytes[1];
		pkt_to_tx[2] = t_val.bytes[0];

		t_val.val = eeprom16bitGet(EEPROM_STORED_ITEM_CTR_PAD_MAX_ADC_VAL);
		pkt_to_tx[3] = t_val.bytes[1];
		pkt_to_tx[4] = t_val.bytes[0];

		t_val.val = eeprom16bitGet(EEPROM_STORED_ITEM_CTR_PAD_MIN_THRESH_PERC);
		pkt_to_tx[5] = t_val.bytes[1];
		pkt_to_tx[6] = t_val.bytes[0];

		t_val.val = eeprom16bitGet(EEPROM_STORED_ITEM_CTR_PAD_MAX_THRESH_PERC);
		pkt_to_tx[7] = t_val.bytes[1];
		pkt_to_tx[8] = t_val.bytes[0];
	}
}

//-------------------------------
// Function: HandlePadCalibrationDataSet
//
// Description: Digests and responds to a received HA_HHP_CMD_CAL_RANGE_SET command.
//
//-------------------------------
static void HandlePadCalibrationDataSet(uint8_t *rxd_pkt, uint8_t *pkt_to_tx)
{
	TypeAccess16Bit_t t_val;

	if (rxd_pkt[2] == COMMS_PAD_SEL_LEFT)
	{
		t_val.bytes[0] = rxd_pkt[4];
		t_val.bytes[1] = rxd_pkt[3];
		eeprom16bitSet(EEPROM_STORED_ITEM_LEFT_PAD_MIN_THRESH_PERC, t_val.val);
		
		t_val.bytes[00] = rxd_pkt[6];
		t_val.bytes[1] = rxd_pkt[5];
		eeprom16bitSet(EEPROM_STORED_ITEM_LEFT_PAD_MAX_THRESH_PERC, t_val.val);
	}
	else if (rxd_pkt[2] == COMMS_PAD_SEL_RIGHT)
	{
		t_val.bytes[0] = rxd_pkt[4];
		t_val.bytes[1] = rxd_pkt[3];
		eeprom16bitSet(EEPROM_STORED_ITEM_RIGHT_PAD_MIN_THRESH_PERC, t_val.val);
		
		t_val.bytes[0] = rxd_pkt[6];
		t_val.bytes[1] = rxd_pkt[5];
		eeprom16bitSet(EEPROM_STORED_ITEM_RIGHT_PAD_MAX_THRESH_PERC, t_val.val);
	}
	else // COMMS_PAD_SEL_CTR
	{
		t_val.bytes[0] = rxd_pkt[4];
		t_val.bytes[1] = rxd_pkt[3];
		eeprom16bitSet(EEPROM_STORED_ITEM_CTR_PAD_MIN_THRESH_PERC, t_val.val);
		
		t_val.bytes[0] = rxd_pkt[6];
		t_val.bytes[1] = rxd_pkt[5];
		eeprom16bitSet(EEPROM_STORED_ITEM_CTR_PAD_MAX_THRESH_PERC, t_val.val);
	}
}

//-------------------------------
// Function: HandlePadDataGet
//
// Description: Digests and responds to a received HA_HHP_CMD_PAD_DATA_GET command.
//
//-------------------------------
static void HandlePadDataGet(uint8_t *rxd_pkt, uint8_t *pkt_to_tx)
{
	HeadArrayInputType_t input_type;
	HeadArraySensor_t sensor_id;

    uint16_t my_Neutral_DAC_setting = eeprom16bitGet(EEPROM_STORED_ITEM_MM_NEUTRAL_DAC_SETTING);
    uint16_t my_Neutral_DAC_range = eeprom16bitGet(EEPROM_STORED_ITEM_MM_NEUTRAL_DAC_RANGE);
    
	switch (rxd_pkt[2])
	{
		case COMMS_PAD_SEL_LEFT:
			input_type = (HeadArrayInputType_t)eepromEnumGet(EEPROM_STORED_ITEM_LEFT_PAD_INPUT_TYPE);
			sensor_id = HEAD_ARRAY_SENSOR_LEFT;
			break;
		
		case COMMS_PAD_SEL_RIGHT:
			input_type = (HeadArrayInputType_t)eepromEnumGet(EEPROM_STORED_ITEM_RIGHT_PAD_INPUT_TYPE);
			sensor_id = HEAD_ARRAY_SENSOR_RIGHT;
			break;
		
		case COMMS_PAD_SEL_CTR:
			input_type = (HeadArrayInputType_t)eepromEnumGet(EEPROM_STORED_ITEM_CTR_PAD_INPUT_TYPE);
			sensor_id = HEAD_ARRAY_SENSOR_CENTER;
			break;
		
		default:
			// Should never be able to happen.  NACK on this anyhow.
			BuildNackPacket(pkt_to_tx);
			return; // Save some code space by doing this instead of using some "cleaner" method.
	}

	pkt_to_tx[0] = 6;

	if (input_type == HEAD_ARR_INPUT_DIGITAL)
	{
		pkt_to_tx[1] = 0x00;
		pkt_to_tx[2] = (uint8_t)headArrayDigitalInputValue(sensor_id);
		pkt_to_tx[3] = 0x00;
		pkt_to_tx[4] = (uint8_t)headArrayDigitalInputValue(sensor_id);
	}
	else
	{
		TypeAccess16Bit_t t_val;
		
		t_val.val = headArrayProportionalInputValueRaw(sensor_id);
		pkt_to_tx[1] = t_val.bytes[1];
		pkt_to_tx[2] = t_val.bytes[0];
		
        switch (sensor_id)
        {
            // Create a percentage based upon the DAC output.
            case HEAD_ARRAY_SENSOR_LEFT:
                // This number is <= neutral for LEFT commands
                t_val.val = headArrayOutputValue(HEAD_ARRAY_OUT_AXIS_LEFT_RIGHT);
                if (t_val.val > my_Neutral_DAC_setting)
                    t_val.val = 0;
                else
                    t_val.val = ((my_Neutral_DAC_setting - t_val.val) * 100) / my_Neutral_DAC_range;
                break;
            case HEAD_ARRAY_SENSOR_RIGHT:
                // This number is >= neutral for RIGHT commands 
                t_val.val = headArrayOutputValue(HEAD_ARRAY_OUT_AXIS_LEFT_RIGHT);
                if (t_val.val < my_Neutral_DAC_setting)
                    t_val.val = 0;
                else
                    t_val.val = ((t_val.val - my_Neutral_DAC_setting) * 100) / my_Neutral_DAC_range;
                break;
            case HEAD_ARRAY_SENSOR_CENTER:
                // This number is >= neutral
                t_val.val = headArrayOutputValue(HEAD_ARRAY_OUT_AXIS_FWD_REV);
                if (t_val.val < my_Neutral_DAC_setting)
                    t_val.val = 0;
                else
                    t_val.val = ((t_val.val - my_Neutral_DAC_setting) * 100) / my_Neutral_DAC_range;
                break;
            case HEAD_ARRAY_SENSOR_EOL: // Satisfies "not considered" compiler warning.
                break;
        } // end switch
		pkt_to_tx[3] = t_val.bytes[1];
		pkt_to_tx[4] = t_val.bytes[0];
	}
}

//-------------------------------
// Function: BuildEnabledFeaturesResponsePacket
//
// Description: This function crates the response to a GET FEATURES command
//-------------------------------

static void BuildEnabledFeaturesResponsePacket(uint8_t *pkt_to_tx)
{
    pkt_to_tx[0] = 5;
	pkt_to_tx[1] = eeprom8bitGet(EEPROM_STORED_ITEM_ENABLED_FEATURES);
	pkt_to_tx[2] = (uint8_t)(eeprom16bitGet(EEPROM_STORED_ITEM_USER_BTN_LONG_PRESS_ACT_TIME) / 100);
    pkt_to_tx[3] = eeprom8bitGet(EEPROM_STORED_ITEM_ENABLED_FEATURES_2);
}        

//-------------------------------
// Function: HandleEnabledFeaturesSet
//
// Description: Digests and responds to a received HA_HHP_CMD_ENABLED_FEATURES_SET command.
//
//-------------------------------
static void HandleEnabledFeaturesSet(uint8_t *rxd_pkt, uint8_t *pkt_to_tx)
{
    uint8_t item1, item2;
    
    item1 = rxd_pkt[2];
    item2 = rxd_pkt[4];
    
	eeprom8bitSet(EEPROM_STORED_ITEM_ENABLED_FEATURES, item1);
	eeprom16bitSet(EEPROM_STORED_ITEM_USER_BTN_LONG_PRESS_ACT_TIME, (uint16_t)rxd_pkt[3] * (uint16_t)100);
	eeprom8bitSet(EEPROM_STORED_ITEM_ENABLED_FEATURES_2, item2);

//	eeprom8bitSet(EEPROM_STORED_ITEM_ENABLED_FEATURES, rxd_pkt[2]);
//	eeprom16bitSet(EEPROM_STORED_ITEM_USER_BTN_LONG_PRESS_ACT_TIME, (uint16_t)rxd_pkt[3] * (uint16_t)100);
//	eeprom8bitSet(EEPROM_STORED_ITEM_ENABLED_FEATURES_2, rxd_pkt[4]);
    
}

//-------------------------------
// Function: HandleModeOfOpSet
//
// Description: Digests and responds to a received HA_HHP_CMD_MODE_OF_OPERATION_SET command.
//
//-------------------------------
static void HandleModeOfOpSet(uint8_t *rxd_pkt, uint8_t *pkt_to_tx)
{
	pkt_to_tx[0] = 3;

	// The current mapping for values received in rxd_pkt[2] and FunctionalFeature_t is a simple offset of 1.
	// Also, cannot change the current feature if device is not active (must "power on" device first)
	if (appCommonFeatureIsEnabled((FunctionalFeature_t)rxd_pkt[2] - 1))
	{
		eepromEnumSet(EEPROM_STORED_ITEM_CURRENT_ACTIVE_FEATURE, (EepromStoredEnumType_t)(rxd_pkt[2] - 1));
		pkt_to_tx[1] = HA_HHP_RESP_ACK;
	}
	else
	{
		pkt_to_tx[1] = HA_HHP_RESP_NACK;
	}
}

//-------------------------------
// Function: BuildHeartBeatResponsePacket
//
// Description: Builds up a packet to send as a response to a HA_HHP_CMD_VERSION_GET command.
//
// NOTE: The current mapping for values received in rxd_pkt[2] and FunctionalFeature_t is a simple offset of 1.
//
//-------------------------------
static void BuildHeartBeatResponsePacket(uint8_t *rxd_pkt, uint8_t *pkt_to_tx)
{
    FunctionalFeature_t currentFeature;
    uint8_t switchStatus;
    
	pkt_to_tx[0] = 5;
	pkt_to_tx[1] = rxd_pkt[2];
    currentFeature = eepromEnumGet(EEPROM_STORED_ITEM_CURRENT_ACTIVE_FEATURE);
	pkt_to_tx[2] = ++currentFeature;    // Make it base 1

	// Build up status byte
	pkt_to_tx[3] = 0x00;
	pkt_to_tx[3] |= AppCommonDeviceActiveGet() ? 0x01 : 0x00;
	pkt_to_tx[3] |= AppCommonCalibrationActiveGet() ? 0x02 : 0x00;
	pkt_to_tx[3] |= headArrayPadIsConnected(HEAD_ARRAY_SENSOR_LEFT) ? 0x04 : 0x00;
	pkt_to_tx[3] |= headArrayPadIsConnected(HEAD_ARRAY_SENSOR_RIGHT) ? 0x08 : 0x00;
	pkt_to_tx[3] |= headArrayPadIsConnected(HEAD_ARRAY_SENSOR_CENTER) ? 0x10 : 0x00;
	pkt_to_tx[3] |= headArrayNeutralTestFail() ? 0x20 : 0x00;
    switchStatus = GetSwitchStatus ();
    pkt_to_tx[3] |= (switchStatus & USER_SWITCH) ? 0x040 : 0x00;
    pkt_to_tx[3] |= (switchStatus & MODE_SWITCH) ? 0x080 : 0x00;
}

//-------------------------------
// Function: Buid_DAC_Get_Response
//
// Description: Builds up a packet to send as a response to a HA_HHP_CMD_NEUTRAL_DAC_GET command.
//
//-------------------------------
static void Buid_DAC_Get_Response(uint8_t *rxd_pkt, uint8_t *pkt_to_tx)
{
	// The response is 3 16-bit values.
    TypeAccess16Bit_t t_val;

	pkt_to_tx[0] = 8;
    // Get the 16-bit DAC Midpoint value into the transmit buffer
    t_val.val = eeprom16bitGet(EEPROM_STORED_ITEM_MM_NEUTRAL_DAC_COUNTS);
    pkt_to_tx[1] = t_val.bytes[1];
    pkt_to_tx[2] = t_val.bytes[0];
    // Get the Neutral DAC setting (variable) into the transmit buffer.
    t_val.val = eeprom16bitGet(EEPROM_STORED_ITEM_MM_NEUTRAL_DAC_SETTING);
    pkt_to_tx[3] = t_val.bytes[1];
    pkt_to_tx[4] = t_val.bytes[0];
    // Get the DAC range into the transmit buffer
    t_val.val = eeprom16bitGet(EEPROM_STORED_ITEM_MM_NEUTRAL_DAC_RANGE);
    pkt_to_tx[5] = t_val.bytes[1];
    pkt_to_tx[6] = t_val.bytes[0];
}

//-------------------------------
// Function: Handle_DAC_SetCommand
//
// Description: Digests and responds to a received HA_HHP_CMD_MODE_OF_OPERATION_SET command.
//
//-------------------------------
static void Handle_DAC_SetCommand(uint8_t *rxd_pkt, uint8_t *pkt_to_tx)
{
    TypeAccess16Bit_t t_val;
    int16_t my16Val, DAC_Constant, DAC_Range;

	t_val.bytes[0] = rxd_pkt[3];
	t_val.bytes[1] = rxd_pkt[2];
    my16Val = t_val.val;
    DAC_Constant = eeprom16bitGet(EEPROM_STORED_ITEM_MM_NEUTRAL_DAC_COUNTS);
    DAC_Range = eeprom16bitGet(EEPROM_STORED_ITEM_MM_NEUTRAL_DAC_RANGE);
    // Check the value for something valid.
    if ((my16Val <= (DAC_Constant + DAC_Range))
       && (my16Val >= (DAC_Constant - DAC_Range)))
    {
		eeprom16bitSet(EEPROM_STORED_ITEM_MM_NEUTRAL_DAC_SETTING, my16Val);
		pkt_to_tx[1] = HA_HHP_RESP_ACK;
	}
	else
	{
		pkt_to_tx[1] = HA_HHP_RESP_NACK;
	}

	pkt_to_tx[0] = 3;
}

//-------------------------------
// Function: CreateGetOffsetResponse
//
// Description: This function creates the response for Get Offset value
//      requested by the Hand Held Programmer.
//
//-------------------------------
static void CreateGetOffsetResponse (uint8_t *pkt_to_tx)
{
	pkt_to_tx[0] = 5;
    // Get the 8-bit DAC Midpoint value into the transmit buffer
    pkt_to_tx[1] = eeprom8bitGet(EEPROM_STORED_ITEM_MM_CENTER_PAD_MINIMUM_DRIVE_OFFSET);
    pkt_to_tx[2] = eeprom8bitGet(EEPROM_STORED_ITEM_MM_LEFT_PAD_MINIMUM_DRIVE_OFFSET);
    pkt_to_tx[3] = eeprom8bitGet(EEPROM_STORED_ITEM_MM_RIGHT_PAD_MINIMUM_DRIVE_OFFSET);
   
}

//-------------------------------
// Function: CreateSetOffsetResponse
//
// Description: This function creates the response for Set Offset value
//      requested by the Hand Held Programmer.
//      The response is either ACK or NAK depending on the range of the value.
//
// Version 5 (1.8.x) I changed the Drive Speed range from 0-30 to 0-60
//      ... No need to have such a tight tolerance here.)
//-------------------------------
static void CreateSetOffsetResponse (uint8_t *rxd_pkt, uint8_t *pkt_to_tx)
{
    if ((rxd_pkt[2] >= 0) && (rxd_pkt[2] <= 60) // Is the value a valid range for Center Pad
        && (rxd_pkt[3] >= 0) && (rxd_pkt[3] <= 60) // Is the value a valid range for Left Pad
        && (rxd_pkt[4] >= 0) && (rxd_pkt[4] <= 60)) // Is the value a valid range for Right Pad
    {
    	eeprom8bitSet(EEPROM_STORED_ITEM_MM_CENTER_PAD_MINIMUM_DRIVE_OFFSET, rxd_pkt[2]);
    	eeprom8bitSet(EEPROM_STORED_ITEM_MM_LEFT_PAD_MINIMUM_DRIVE_OFFSET, rxd_pkt[3]);
    	eeprom8bitSet(EEPROM_STORED_ITEM_MM_RIGHT_PAD_MINIMUM_DRIVE_OFFSET, rxd_pkt[4]);
		pkt_to_tx[1] = HA_HHP_RESP_ACK;
	}
	else
	{
		pkt_to_tx[1] = HA_HHP_RESP_NACK;
	}
    pkt_to_tx[0] = 3;   // Set msg length to 3 for NAK or ACK.
}

//-------------------------------
// Function: TranslateInputToOutputMapValFromEnum
//
// Description: Translates an enumerated input->output mapping value to an HA<->HHP comms interface value.
//
//-------------------------------
static uint8_t TranslateInputToOutputMapValFromEnum(HeadArrayOutputFunction_t output_assignment)
{
	switch (output_assignment)
	{
		case HEAD_ARRAY_OUT_FUNC_LEFT:
			return COMMS_OUTPUT_SEL_LEFT;
			
		case HEAD_ARRAY_OUT_FUNC_RIGHT:
			return COMMS_OUTPUT_SEL_RIGHT;
			
		case HEAD_ARRAY_OUT_FUNC_FWD:
			return COMMS_OUTPUT_SEL_FWD;
			
		case HEAD_ARRAY_OUT_FUNC_REV:
			return COMMS_OUTPUT_SEL_BACKWARDS;
			
		case HEAD_ARRAY_OUT_FUNC_NONE:
		default:
			return COMMS_OUTPUT_SEL_OFF;
	}
}

//-------------------------------
// Function: TranslateInputToOutputMapValToEnum
//
// Description: Translates HA<->HHP comms interface value to an enumerated input->output mapping value.
//
//-------------------------------
static HeadArrayOutputFunction_t TranslateInputToOutputMapValToEnum(uint8_t output_assignment)
{
	switch (output_assignment)
	{
		case COMMS_OUTPUT_SEL_LEFT:
			return HEAD_ARRAY_OUT_FUNC_LEFT;
			
		case COMMS_OUTPUT_SEL_RIGHT:
			return HEAD_ARRAY_OUT_FUNC_RIGHT;
			
		case COMMS_OUTPUT_SEL_FWD:
			return HEAD_ARRAY_OUT_FUNC_FWD;
			
		case COMMS_OUTPUT_SEL_BACKWARDS:
			return HEAD_ARRAY_OUT_FUNC_REV;
			
		case COMMS_OUTPUT_SEL_OFF:
		default:
			return HEAD_ARRAY_OUT_FUNC_NONE;
	}
}

//-------------------------------
// Function: TranslateInputToOutputMapValFromEnum
//
// Description: Translates HA<->HHP comms interface value to an enumerated input type value.
//
//-------------------------------
static HeadArrayInputType_t TranslateInputTypeValToEnum(uint8_t input_type)
{
	switch (input_type)
	{
		case COMMS_PAD_TYPE_DIGITAL:
			return HEAD_ARR_INPUT_DIGITAL;
			
		case COMMS_PAD_TYPE_PROPORTIONAL:
			return HEAD_ARR_INPUT_PROPORTIONAL;

		default:
			return HEAD_ARR_INPUT_EOL;
	}
}

//-------------------------------
// Function: CalcChecksum
//
// Description: Calculates the checksum for a packet.
//
//-------------------------------
static uint8_t CalcChecksum(uint8_t *packet, uint8_t len)
{
	uint8_t checksum = 0;

	for (unsigned int i = 0; i < len; i++)
	{
		checksum += packet[i];
	}

	return checksum;
}

#endif // #ifdef OK_TO_USE_HHP_CODE

// end of file.
//-------------------------------------------------------------------------
