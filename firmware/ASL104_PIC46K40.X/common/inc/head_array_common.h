//////////////////////////////////////////////////////////////////////////////
//
// Filename: head_array_common.h
//
// Description: Functionality common to components related to head array monitoring and control.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef HEAD_ARRAY_COMMON_H
#define HEAD_ARRAY_COMMON_H

/* ******************************   Types   ******************************* */

typedef enum
{
	HEAD_ARRAY_SENSOR_LEFT,
	HEAD_ARRAY_SENSOR_RIGHT,
	HEAD_ARRAY_SENSOR_CENTER,

	// Nothing else may be defined past this point!
	HEAD_ARRAY_SENSOR_EOL
} HeadArraySensor_t;

typedef enum
{
	DAC_SELECT_FORWARD_BACKWARD,
	DAC_SELECT_LEFT_RIGHT,

	// Nothing else may be defined past this point!
	DAC_SELECT_SENSOR_EOL
} DacSelect_t;

#endif // HEAD_ARRAY_COMMON_H

// end of file.
//-------------------------------------------------------------------------
