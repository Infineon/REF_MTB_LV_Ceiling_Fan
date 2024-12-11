/**
 * @file ccu4.h
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
 * @file ccu4.h
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

#ifndef PMSM_FOC_MCUINIT_CCU4_H_
#define PMSM_FOC_MCUINIT_CCU4_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

#include "..\Configuration\pmsm_foc_variables_scaling.h"

/**
 * @addtogroup
 * @{
 */

/**
 * @addtogroup
 * @{
 */

#if(IR_Remote_Control == ENABLED)
/**********************************************************************************************************************
 * ENUMS
 **********************************************************************************************************************/
/**
 * @ingroup GLOBAL_CCU4_enumerations
 * @{
 */
/**
 * @brief Return status of the GLOBAL_CCU4 APP
 */
typedef enum GLOBAL_CCU4_STATUS
{
  GLOBAL_CCU4_STATUS_SUCCESS = 0U, /**< Status success */
  GLOBAL_CCU4_STATUS_FAILURE /**< Status failure */
} GLOBAL_CCU4_STATUS_t;
/**
* @}
*/
/***********************************************************************************************************************
* DATA STRUCTURES
***********************************************************************************************************************/

/**
 * @ingroup GLOBAL_CCU4_datastructures
 * @{
 */

/**
 * This saves the context of the GLOBAL_CCU4 APP.
 */
typedef struct GLOBAL_CCU4
{
  const uint32_t module_frequency; /**< fccu frequency */
  const XMC_SCU_CCU_TRIGGER_t syncstart_trigger_msk; /**< Mask to start the timers synchronously */
  XMC_CCU4_MODULE_t* const module_ptr;   /**< reference to module handle */
  XMC_CCU4_SLICE_MCMS_ACTION_t const mcs_action; /**< Shadow transfer of selected values in multi-channel mode */
  bool  is_initialized; /**< Indicates initialized state of particular instance of the APP */
} GLOBAL_CCU4_t;

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
 * Initializes the CCU4 module for debugging. Variable to be monitored are outputs to P1. 0, P0.4, P1.2, P1.3. <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void pmsm_foc_ccu4_init(void);

#if(IR_Remote_Control == ENABLED)
/**
 * @brief Initializes a GLOBAL_CCU4 with generated configuration.
 *
 * @param handle pointer to the GLOBAL_CCU4 APP handle structure.
 * @return GLOBAL_CCU4_STATUS_t\n  GLOBAL_CCU4_STATUS_SUCCESS : if initialization is successful\n
 *                                 GLOBAL_CCU4_STATUS_FAILURE : if initialization is failed\n
 * <BR>
 * \par<b>Description:</b><br>
 * <ul>
 * <li>Enable the module.</li>
 * <li>Start the prescaler.</li>
 * </ul>
 *
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *  DAVE_STATUS_t init_status;
 *  init_status = DAVE_Init();  // GLOBAL_CCU4_Init(&GLOBAL_CCU4_0) will be called from DAVE_Init()
 *
 *  while(1)
 *  {
 *  }
 *  return 1;
 * }
 * @endcode<BR>
 */
GLOBAL_CCU4_STATUS_t GLOBAL_CCU4_Init(GLOBAL_CCU4_t* handle);

/**
 * @brief Start all the timers which are configured to start externally on positive edge.<br>
 * @param ccucon_msk mask for which kernels sync start has to be applied.
 * \par<b>Note:</b><br>
 * This mask has been generated in the APP handle and as a macro in global_ccu4_conf.h file.
 * 1. The variable from the APP handle is useful while starting the specific kernel/s
 * 2. GLOBAL_CCU4_CCUCON_Msk Macro from global_ccu4_conf.h file can be used to start all the selected kernels at a time.
 * @retval none
 *
 * \par<b>Description:</b><br>
 * The top level APPs have to be enabled, to start the timer externally with positive trigger edge.
 *
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *   DAVE_STATUS_t status;
 *
 *   status = DAVE_Init();  // GLOBAL_CCU4_Init() is called from DAVE_Init()
 *
 *  // Below can be used to start the specific kernels, by generating two instance of APP
 *  // GLOBAL_CCU4_SyncStartTriggerHigh((uint32_t)(GLOBAL_CCU4_0.syncstart_trigger_msk | GLOBAL_CCU4_1.syncstart_trigger_msk));
 *  // Below can be used to start all the kernels simultaneously
 *   GLOBAL_CCU4_SyncStartTriggerHigh(GLOBAL_CCU4_CCUCON_Msk);
 *
 *   while(1)
 *   {
 *   }
 *
 *   return 1;
 * }
 * @endcode <BR> </p>
 */
__STATIC_INLINE void GLOBAL_CCU4_SyncStartTriggerHigh(uint32_t ccucon_msk)
{
  XMC_SCU_SetCcuTriggerHigh(ccucon_msk);
}

/**
 * @brief Start all the timers which are configured to start externally on negative edge.<br>
 * @param ccucon_msk mask for which kernels sync start has to be applied.
 * \par<b>Note:</b><br>
 * This mask has been generated in the APP handle and a macro in global_ccu4_conf.h file.
 * 1. The variable from the APP handle is useful while starting the specific kernel/s
 * 2. GLOBAL_CCU4_CCUCON_Msk Macro from global_ccu4_conf.h file can be used to start all the selected kernels at a time.
 * @retval none
 *
 * \par<b>Description:</b><br>
 * The top level APPs have to be enabled, to start the timer externally with negative trigger edge.
 *
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *   DAVE_STATUS_t status;
 *
 *   status = DAVE_Init();  // GLOBAL_CCU4_Init() is called from DAVE_Init()
 *
 *  // Below can be used to start the specific kernels, by generating two instance of APP
 *  // GLOBAL_CCU4_SyncStartTriggerLow((uint32_t)(GLOBAL_CCU4_0.syncstart_trigger_msk | GLOBAL_CCU4_1.syncstart_trigger_msk));
 *  // Below can be used to start all the kernels simultaneously
 *   GLOBAL_CCU4_SyncStartTriggerLow(GLOBAL_CCU4_CCUCON_Msk);
 *
 *   while(1)
 *   {
 *   }
 *
 *   return 1;
 * }
 * @endcode <BR> </p>
 */
__STATIC_INLINE void GLOBAL_CCU4_SyncStartTriggerLow(uint32_t ccucon_msk)
{
  XMC_SCU_SetCcuTriggerLow(ccucon_msk);
}

/**
 * @}
 */


//#include "global_ccu4_extern.h"
extern GLOBAL_CCU4_t GLOBAL_CCU4_0; /**< APP handle for handle GLOBAL_CCU4_0*/
#endif /*#if(IR_Remote_Control == ENABLED)*/

#endif /* MCUINIT_CCU4_H_ */
/**
 * @}
 */

/**
 * @}
 */
