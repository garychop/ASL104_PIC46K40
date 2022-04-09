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
 * @brief       App level general output control
 * @file        general_output_ctrl_app.h
 * @author      tparsh
 * @ingroup     APP
 *
 ********************************************************************************************************
 */

/**
 * @addtogroup APP
 * @{
 */

#ifndef GENERAL_OUTPUT_CTRL_APP_H_
#define GENERAL_OUTPUT_CTRL_APP_H_

/*
 ********************************************************************************************************
 *                                           INCLUDE FILES
 ********************************************************************************************************
 */
/********************************************** System *************************************************/
#include <stdbool.h>

/********************************************    RTOS   ******************************************/
#include "cocoos.h"

/********************************************    User   ******************************************/
#include "general_output_ctrl_cfg.h"

/*
 ********************************************************************************************************
 *                                             FUNCTION PROTOTYPES
 ********************************************************************************************************
 */

bool GenOutCtrlApp_Init(void);
void GenOutCtrlApp_SetStateAll(GenOutState_t ctrlr_state);
void GenOutCtrlApp_SetState(GenOutCtrlId_t item_id, GenOutState_t ctrlr_state);
bool genOutCtrlAppNeedSendEvent(void);
Evt_t genOutCtrlAppWakeEvent(void);

// End of Doxygen grouping
/** @} */

#endif // End of GENERAL_OUTPUT_CTRL_APP_H_

/**********************************************************************************************************************
---          End of File          ------          End of File          ------          End of File          ---
**********************************************************************************************************************/
