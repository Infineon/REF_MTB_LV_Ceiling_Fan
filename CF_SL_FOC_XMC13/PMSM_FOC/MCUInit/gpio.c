/**
 * @file gpio.c
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
 * @file gpio.c
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

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "gpio.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/**
 *  Data Structure initialization - GPIO Configuration for Gate Driver enable pin .
 */
XMC_GPIO_CONFIG_t IO_PadConfig_Pushpull  =
{
  .mode            = (XMC_GPIO_MODE_t)XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
  .output_level    = (XMC_GPIO_OUTPUT_LEVEL_t)ENABLE_LEVEL,

  .input_hysteresis= XMC_GPIO_INPUT_HYSTERESIS_STANDARD

};

#if (Protection_Indicator ==  ENABLED)
XMC_GPIO_CONFIG_t LED_Protection_Config  =
{
  .mode            = (XMC_GPIO_MODE_t)XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
  .output_level    = (XMC_GPIO_OUTPUT_LEVEL_t)XMC_GPIO_OUTPUT_LEVEL_LOW,
#endif

};
#if(IR_Remote_Control == ENABLED)

const DIGITAL_IO_t recvpin =
{
  .gpio_port = XMC_GPIO_PORT0, /*Change the _port & _pin according to the assigned pin for IR recieve Pin*/
  .gpio_pin = 10U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_INPUT_TRISTATE,
    .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD
  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

DIGITAL_IO_STATUS_t DIGITAL_IO_Init(const DIGITAL_IO_t *const handler)
{
  XMC_ASSERT("DIGITAL_IO_Init: handler null pointer", handler != NULL);

  /* Initializes input / output characteristics */
  XMC_GPIO_Init(handler->gpio_port, handler->gpio_pin, &handler->gpio_config);

  /*Configure hardware port control*/
  XMC_GPIO_SetHardwareControl(handler->gpio_port, handler->gpio_pin, handler->hwctrl);

  return (DIGITAL_IO_STATUS_OK);

}

XMC_GPIO_CONFIG_t IR_testing  =
{
  .mode            = (XMC_GPIO_MODE_t)XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
  .output_level    = (XMC_GPIO_OUTPUT_LEVEL_t)XMC_GPIO_OUTPUT_LEVEL_LOW,

};
#endif
/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

/* API to initialize GPIO pins used */
void pmsm_foc_gpio_Init(void)
{
//	/* P0.11 as gate driver enable pin */
//	XMC_GPIO_Init(INVERTER_EN_PIN, &IO_PadConfig_Pushpull); /*NOT used in Design*/

	/* P0.0	ALT5 CCU80.OUT00 */
	XMC_GPIO_SetMode(PHASE_U_HS_PIN, PHASE_U_HS_ALT_SELECT);

	/* P0.1 ALT5 CCU80.OUT01 */
	XMC_GPIO_SetMode(PHASE_U_LS_PIN, PHASE_U_LS_ALT_SELECT);

	/* P0.8 ALT7 CCU80.OUT10 */
	XMC_GPIO_SetMode(PHASE_V_HS_PIN, PHASE_V_HS_ALT_SELECT);

	/* P0.9 ALT7 CCU80.OUT11 */
	XMC_GPIO_SetMode(PHASE_V_LS_PIN, PHASE_V_LS_ALT_SELECT);

	/* P0.7 ALT5 CCU80.OUT20 */
	XMC_GPIO_SetMode(PHASE_W_HS_PIN, PHASE_W_HS_ALT_SELECT);

	/* P0.6 ALT5 CCU80.OUT21 */
	XMC_GPIO_SetMode(PHASE_W_LS_PIN, PHASE_W_LS_ALT_SELECT);

	/* P0.12 as CCU80 Trap input, internal pull-up */
//	XMC_GPIO_SetMode(TRAP_PIN, XMC_GPIO_MODE_INPUT_PULL_UP); /*Replaced with ACMP-->ERU-->CCU8_TRAP*/

#if (Test_point == ENABLED)
	XMC_GPIO_SetMode (TEST_PIN,XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
	XMC_GPIO_SetOutputLow(TEST_PIN);
#endif

#if (Protection_Indicator ==  ENABLED)
	XMC_GPIO_Init (LED_protection, &LED_Protection_Config);
#endif

#if (MCU_Load_time_test == ENABLED)
	XMC_GPIO_SetMode (mcu_load,XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
	XMC_GPIO_SetOutputLow(mcu_load);
#endif

#if(Remote_Control == ENABLED)

	if(User_Para[58] == 1) /*1 - For RF remote*/
	{
	  /*GPIO Setup for Remote Pin*/
	  /*rm: page 1223 | data sheet: page 39*/
	  /*Set as pull down. if remote is not use, it should be LOW*/
	//  XMC_GPIO_SetMode (button_D, XMC_GPIO_MODE_INPUT_PULL_UP);    /*empty*/
	  XMC_GPIO_SetMode (button_C, XMC_GPIO_MODE_INPUT_PULL_DOWN);    /*For button increase speed*/
	  XMC_GPIO_SetMode (button_B, XMC_GPIO_MODE_INPUT_PULL_DOWN);    /*For button decrease speed*/
	  XMC_GPIO_SetMode (button_A, XMC_GPIO_MODE_INPUT_PULL_DOWN);    /*For start stop function*/
	}

#endif

#if(IR_Remote_Control == ENABLED)
//	  XMC_GPIO_Init (IR_LED_Testing, &IR_testing);
//	  XMC_GPIO_SetOutputLow(IR_LED_Testing);
//	  XMC_GPIO_Init (IR_LED_Testing2, &IR_testing);
//	  XMC_GPIO_SetOutputLow(IR_LED_Testing2);
//	  XMC_GPIO_Init (IR_LED_Testing3, &IR_testing);
//	  XMC_GPIO_SetOutputLow(IR_LED_Testing3);
#endif

#if(MCU_Sleep_Mode == ENABLED)
	XMC_GPIO_SetMode (ERU_sleepmode,XMC_GPIO_MODE_INPUT_TRISTATE); /*Pin2_9*/
	XMC_GPIO_EnableDigitalInput(ERU_sleepmode);

//  XMC_GPIO_SetMode (ERU_sleepmode,XMC_GPIO_MODE_INPUT_PULL_UP); /*Pin2_9*/
//  XMC_GPIO_EnableDigitalInput(ERU_sleepmode);
#endif
}
