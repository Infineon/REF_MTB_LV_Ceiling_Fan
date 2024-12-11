/**
 * @file gpio.h
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
 * @file gpio.h
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

#ifndef PMSM_FOC_MCUINIT_GPIO_H_
#define PMSM_FOC_MCUINIT_GPIO_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

#include "..\Configuration\pmsm_foc_variables_scaling.h"

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#if(IR_Remote_Control == ENABLED)
/**********************************************************************************************************************
 * ENUMS
 **********************************************************************************************************************/

/**
* @ingroup DIGITAL_IO_enumerations
* @{
*/

/**
* @brief Initialization status of DIGITAL_IO APP.
*/
typedef enum DIGITAL_IO_STATUS
{
  DIGITAL_IO_STATUS_OK = 0U,/**< 0=Status OK */
  DIGITAL_IO_STATUS_FAILURE = 1U/**< 1=Status Failed */
} DIGITAL_IO_STATUS_t;

/**
* @}
*/

/**********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/
/**
* @ingroup DIGITAL_IO_datastructures
* @{
*/

/**
* @brief Initialization data structure of DIGITAL_IO APP
*/
typedef struct DIGITAL_IO
{
  XMC_GPIO_PORT_t *const gpio_port;             /**< port number */
  const XMC_GPIO_CONFIG_t gpio_config;          /**< mode, initial output level and pad driver strength / hysteresis */
  const uint8_t gpio_pin;                       /**< pin number */
  const XMC_GPIO_HWCTRL_t hwctrl;               /**< Hardware port control */
} DIGITAL_IO_t;


/**
* @}
*/
#endif /*#if(IR_Remote_Control == ENABLED)*/

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes GPIO pins used. <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void pmsm_foc_gpio_Init(void);

#if(IR_Remote_Control == ENABLED)

/**
*
* @brief Function to initialize the port pin as per UI settings.
* @param handler Pointer pointing to APP data structure. Refer @ref DIGITAL_IO_t for details.
* @return DIGITAL_IO_STATUS_t DIGITAL_IO APP status. Refer @ref DIGITAL_IO_STATUS_t structure for details.
*
* \par<b>Description:</b><br>
* This function initializes GPIO port registers IOCR,PDISC,OMR,PDR/PHCR to configure pin direction,initial output level,
* and pad driver strength/hysteresis.
*
* \par<b>Related APIs:</b><BR>
* None
**/

DIGITAL_IO_STATUS_t DIGITAL_IO_Init(const DIGITAL_IO_t *const handler);

/**
*
* @brief Function to set port pin high.
* @param handler Pointer pointing to APP data structure. Refer @ref DIGITAL_IO_t for details.
* @return None
*
* \par<b>Description:</b><br>
* This function configures port output modification register Pn_OMR, to make port pin to high level.
*
* \par<b>Related APIs:</b><BR>
*  DIGITAL_IO_SetOutputLow()
*
*/

__STATIC_INLINE void DIGITAL_IO_SetOutputHigh(const DIGITAL_IO_t *const handler)
{
  XMC_ASSERT("DIGITAL_IO_SetOutputHigh: handler null pointer", handler != NULL);
  XMC_GPIO_SetOutputHigh(handler->gpio_port, handler->gpio_pin);
}

/**
* @brief Function to reset port pin.
* @param handler Pointer pointing to APP data structure. Refer @ref DIGITAL_IO_t for details.
* @return None
*
* \par<b>Description:</b><br>
* This function configures port output modification register Pn_OMR, to make port pin to low level.
*
* \par<b>Related APIs:</b><BR>
* DIGITAL_IO_SetOutputHigh()
*
*/

__STATIC_INLINE void DIGITAL_IO_SetOutputLow(const DIGITAL_IO_t *const handler)
{
  XMC_ASSERT("DIGITAL_IO_SetOutputLow: handler null pointer", handler != NULL);
  XMC_GPIO_SetOutputLow(handler->gpio_port,handler->gpio_pin);
}

/**
* @brief Function to Toggle port pin.
* @param handler Pointer pointing to APP data structure. Refer @ref DIGITAL_IO_t for details.
* @return None
*
* \par<b>Description:</b><br>
* This function configures port output modification register Pn_OMR, to toggle port pin.
*
* \par<b>Related APIs:</b><BR>
* DIGITAL_IO_SetOutputLow(), DIGITAL_IO_SetOutputHigh()
*
*/

__STATIC_INLINE void DIGITAL_IO_ToggleOutput(const DIGITAL_IO_t *const handler)
{
  XMC_ASSERT("DIGITAL_IO_ToggleOutput: handler null pointer", handler != NULL);
  XMC_GPIO_ToggleOutput(handler->gpio_port, handler->gpio_pin);
}

/**
* @brief Function to read input level of port pin.
* @param handler Pointer pointing to APP data structure. Refer @ref DIGITAL_IO_t for details.
* @return uint32_t input logic level. Range:0-1
*
* \par<b>Description:</b><br>
* This function reads the Pn_IN register and returns the current logical value at the GPIO pin.
*
* \par<b>Related APIs:</b><BR>
*  None
*
*/

__STATIC_INLINE uint32_t DIGITAL_IO_GetInput(const DIGITAL_IO_t *const handler)
{
  XMC_ASSERT("DIGITAL_IO_GetInput: handler null pointer", handler != NULL);
  return XMC_GPIO_GetInput(handler->gpio_port, handler->gpio_pin);
}

/* Include APP extern file */
//#include "digital_io_extern.h"
extern const DIGITAL_IO_t recvpin;

#endif /*#if(IR_Remote_Control == ENABLED)*/

#endif /* MCUINIT_GPIO_H_ */

/**
 * @}
 */

/**
 * @}
 */
