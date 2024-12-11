/**
 * @file pmsm_foc_error_handling.h
 * @date 09 May, 2017
 *
 * @cond
 *********************************************************************************************************************
 * PMSM FOC Motor Control Library
 *
 * Copyright (c) 2015-2017, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes
 * with Infineon Technologies AG (dave@infineon.com).
 ***********************************************************************************************************************
 *
 * @file pmsm_foc_error_handling.h
 * @date 09 May, 2017
 * @version 1.0.1
 *
 * @brief Sensorless FOC with 3-shunt <br>
 *
 * <b>Detailed description of file</b> <br>
 *  IDE: Infineon DAVE 4, Version: 4.1.2, Installer build : 2016-09-30
 *  HW:  Infineon PMSM-LV-15W, or XMC 750 Watt Motor Control Application Kit.
 *  MCU: Infineon XMC1302.
 *
 * History
 *
 * @endcond
 *
 */

#ifndef PMSM_FOC_INTERRUPTS_PMSM_FOC_ERROR_HANDLING_H_
#define PMSM_FOC_INTERRUPTS_PMSM_FOC_ERROR_HANDLING_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

#include "..\ControlModules\pmsm_foc_functions.h"
#include "..\MCUInit\gpio.h"

#define PMSM_FOC_EID_CTRAP_POS (0U)
#define PMSM_FOC_EID_UNDER_VOLT_POS (1U)
#define PMSM_FOC_EID_OVER_VOLT_POS (2U)
#define PMSM_FOC_EID_STALL_POS (3U)
#define PMSM_FOC_EID_INVERTER_OVER_TEMP_POS (4U)
#define PMSM_FOC_EID_TORQUE_LIMIT_EXCEED_POS (5U)
#define PMSM_FOC_EID_OVER_CURRENT_POS (6U)
#define PMSM_FOC_EID_PLL_POS (7U)

typedef enum PMSM_FOC_EID
{
PMSM_FOC_EID_NO_ERROR = 0, /*!< Error ID 000 - NO ERROR */
PMSM_FOC_EID_CTRAP = (1U<<PMSM_FOC_EID_CTRAP_POS), /*!< Error ID 001 - CTRAP */
PMSM_FOC_EID_UNDER_VOLT = (1U<<PMSM_FOC_EID_UNDER_VOLT_POS), /*!< Error ID 002 - DC BUS UNDER VOLTAGE */
PMSM_FOC_EID_OVER_VOLT = (1U<<PMSM_FOC_EID_OVER_VOLT_POS), /*!< Error ID 004 - DC BUS OVER VOLTAGE */
PMSM_FOC_EID_STALL = (1U<<PMSM_FOC_EID_STALL_POS), /*!< Error ID 008 - ROTOR STALLED */
PMSM_FOC_EID_INVERTER_OVER_TEMP = (1U<<PMSM_FOC_EID_INVERTER_OVER_TEMP_POS), /*!< Error ID 016 - INVERTER OVER TEMPERATURE */
PMSM_FOC_EID_TORQUE_LIMIT_EXCEED = (1U<<PMSM_FOC_EID_TORQUE_LIMIT_EXCEED_POS), /*!< Error ID 032 - TORQUE LIMIT EXCEED */
PMSM_FOC_EID_OVER_CURRENT = (1U<<PMSM_FOC_EID_OVER_CURRENT_POS), /*!< Error ID 064 - OVER CURRENT */
PMSM_FOC_EID_PLL_FAULT = (1U<<PMSM_FOC_EID_PLL_POS), /*!< Error ID 128 - PLL Fault */

} PMSM_FOC_EID_t;

void pmsm_foc_error_handling (void);

#endif /* PMSM_FOC_INTERRUPTS_PMSM_FOC_ERROR_HANDLING_H_ */

