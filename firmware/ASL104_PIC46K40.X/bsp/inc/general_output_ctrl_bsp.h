/*
	MIT License

	Copyright (c) 2018 Trevor Parsh

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
 */

/**
 ********************************************************************************************************
 *
 * @brief       Defines the BSP level control API for general output control module.
 * @file        general_output_ctrl_bsp.h
 * @author      tparsh
 * @ingroup		USER_DRIVER
 *
 ********************************************************************************************************
 */

#ifndef GENERAL_OUTPUT_CTRL_BSP_H_
#define GENERAL_OUTPUT_CTRL_BSP_H_

/**
 * @addtogroup USER_DRIVER
 * @{
 */

/*
 ********************************************************************************************************
 *                                           INCLUDE FILES
 ********************************************************************************************************
 */
/********************************************** System *************************************************/
#include <stdbool.h>
/********************************************    User   ************************************************/
#include "general_output_ctrl_cfg.h"

/*
 ********************************************************************************************************
 *                                             FUNCTION PROTOTYPES
 ********************************************************************************************************
 */

bool GenOutCtrlBsp_Enable(GenOutCtrlId_t item_id);
bool GenOutCtrlBsp_Disable(GenOutCtrlId_t item_id);
bool GenOutCtrlBsp_SetActive(GenOutCtrlId_t item_id);
bool GenOutCtrlBsp_SetInactive(GenOutCtrlId_t item_id);
bool GenOutCtrlBsp_Toggle(GenOutCtrlId_t item_id);

// End of Doxygen grouping
/** @} */

#endif // End of GENERAL_OUTPUT_CTRL_BSP_H_

/**********************************************************************************************************************
---          End of File          ------          End of File          ------          End of File          ---
**********************************************************************************************************************/
