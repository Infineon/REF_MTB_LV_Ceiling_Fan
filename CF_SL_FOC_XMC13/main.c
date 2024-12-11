/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the XMC13-14 Drive Card PMSM FOC SL Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
 Copyright (c) 2015-2017, Infineon Technologies AG                            **
 All rights reserved.                                                         **
                                                                              **
 Redistribution and use in source and binary forms, with or without           **
 modification,are permitted provided that the following conditions are met:   **
                                                                              **
 *Redistributions of source code must retain the above copyright notice,      **
 this list of conditions and the following disclaimer.                        **
 *Redistributions in binary form must reproduce the above copyright notice,   **
 this list of conditions and the following disclaimer in the documentation    **
 and/or other materials provided with the distribution.                       **
 *Neither the name of the copyright holders nor the names of its contributors **
 may be used to endorse or promote products derived from this software without**
 specific prior written permission.                                           **
                                                                              **
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"  **
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE    **
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE   **
 ARE  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE   **
 LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR         **
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF         **
 SUBSTITUTE GOODS OR  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS    **
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN      **
 CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)       **
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE   **
 POSSIBILITY OF SUCH DAMAGE.                                                  **
                                                                              **
 To improve the quality of the software, users are encouraged to share        **
 modifications, enhancements or bug fixes with Infineon Technologies AG       **
 dave@infineon.com).                                                          **
                                                                              **
********************************************************************************
**                                                                            **
**                                                                            **
** PLATFORM : Infineon XMC1300 Series                                         **
**                                                                            **
** AUTHOR : Motor Control  R&D Team                                           **
**                                                                            **
** version 1.5.0                                                              **
**                                                                            **
** Modified date: 2019-01-10                                                  **
**                                                                            **
*********************************************************************************************************************
* HEADER FILES
*********************************************************************************************************************/

/* SFR declarations of the selected device */
#include "PMSM_FOC/Configuration/pmsm_foc_variables_scaling.h"
#include "PMSM_FOC/ControlModules/pmsm_foc_interface.h"


bool motor_request_start = false; //true = direct start; false = Micro Inspector Pro start (default)
bool motor_off = true;
float Vdc_link;
int32_t Motor_actual_speed;
int32_t Motor_target_set = 1000;


#if(uCPROBE_GUI_OSCILLOSCOPE == ENABLED)
#include "ProbeScope/probe_scope.h"
#endif


/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
extern void pmsm_foc_motor_start(void);
extern void pmsm_foc_init (void);
#if defined ( __CC_ARM )
extern char Image$$RW_CODE$$Base ;
extern char Image$$RW_CODE$$Length ;
extern char Load$$RW_CODE$$Base ;

int load_ramcode( void )
{
  return (int)memcpy( &Image$$RW_CODE$$Base,
          &Load$$RW_CODE$$Base,
          ( size_t )&Image$$RW_CODE$$Length) ;
}
#endif


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* Initialization of MCU and motor control peripherals (CCU, Math unit, GPIOs, ADCs)
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/

/* SFR declarations of the selected device */
#include "PMSM_FOC\Configuration\pmsm_foc_variables_scaling.h"

#if(uCPROBE_GUI_OSCILLOSCOPE == ENABLED)
#include "ProbeScope\probe_scope.h"
#endif

#if(UART_INTERFACE == ENABLED)
#include <stdlib.h>               /*For strtol | strtof*/
#include "PMSM_FOC\MIDSys\pmsm_foc_uart.h"
#include "PMSM_FOC\MIDSys\serial.h"
#include "PMSM_FOC\MIDSys\shell.h"
#include "xmc1_flash.h"           /*For flash feature*/
#include <stdio.h>

uint32_t User_Para[70];          /*Flash Array size - depend how many parameter needed to be stored in flash*/
#else
/* uC Probe Variables */
#include "xmc1_flash.h"  /* For uC_Probe */
uint32_t User_Para[70];
#endif/*#if(UART_INTERFACE == ENABLED)*/

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
extern void pmsm_foc_motorstart(int32_t user_set_speed);
extern void pmsm_foc_init (void);
extern void variables_init_main (void);
extern void user_para_init(void);

#if((Remote_Control == ENABLED) || (IR_Remote_Control == ENABLED))
extern void remote_control_variables_init (void);
#endif

#if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)
extern void start_cfr_motor(void);
#endif

uint32_t nANGLE_TO_SPEED_CONV_FACTOR_SCALE;
uint32_t nANGLE_TO_SPEED_CONV_FACTOR;

int main(void)
{
	nANGLE_TO_SPEED_CONV_FACTOR_SCALE = ANGLE_TO_SPEED_CONV_FACTOR_SCALE;
	nANGLE_TO_SPEED_CONV_FACTOR = ANGLE_TO_SPEED_CONV_FACTOR;

#if(UART_INTERFACE == ENABLED)
    uart_config_init();
#else
    user_para_init();
#endif/*#if(UART_INTERFACE == ENABLED)*/

#if(uCPROBE_GUI_OSCILLOSCOPE == ENABLED)
  ProbeScope_Init(USER_CCU8_PWM_FREQ_HZ);
#endif

  /* Init MCU and motor control peripherals */
  pmsm_foc_init ();

#if((Remote_Control == ENABLED) || (IR_Remote_Control == ENABLED))
  remote_control_variables_init();  /*Remote Control Initialize*/
#endif
  variables_init_main();    /*Fault Reset / motor last speed*/

#if(SETTING_TARGET_SPEED == BY_POT_ONLY && uCPROBE_GUI_no_UART== DISABLED)
  pmsm_foc_motorstart();
#endif

	while (1)
	 /* MCU main loop. Actually only require the processor to run when an interrupt occurs. */
	{
      /* Handle periodic CCU80 Period Match Interrupt CCU80_0_IRQHandler () in FOC_Functions.c */
      /* and ADC Interrupt VADC0_G1_1_IRQHandler () in ADC.c */
      /* Placeholder for user application code. */
	}

	 return 0;
}
/* [] END OF FILE */

