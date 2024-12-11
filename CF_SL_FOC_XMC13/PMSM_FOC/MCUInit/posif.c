/**
 * @file pmsm_foc_posif.c
 * @date 09 May, 2019
 *
 * @cond
 *********************************************************************************************************************
 * PMSM FOC motor Control Library
 *
 * Copyright (c) 2015-2020, Infineon Technologies AG
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
 * @endcond
 ***********************************************************************************************************************/
/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "../MCUInit/posif.h"
#include "../Configuration/pmsm_foc_variables_scaling.h"
/***********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/

#if (USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)
/**
 * POSIF configured with mult-ichannel mode
 */
const XMC_POSIF_CONFIG_t PMSM_FOC_POSIF_CONFIG =
{
  .mode   = XMC_POSIF_MODE_MCM                     /**< POSIF Operational mode */
};

/**
 * POSIF multi-channel configurations
 * Multi-channel pattern update signal is connected to CCU4 blanking signal period match or
 * CCU4 fast synchronization period match
 */
//const XMC_POSIF_MCM_CONFIG_t PMSM_FOC_POSIF_MCM_Config =
//{
//  .pattern_sw_update      = 0U,
//  .pattern_update_trigger = MOTOR0_BLDC_SCALAR_POSIF_PATTERN_UPDATE_SEL,
//  .pattern_trigger_edge   = XMC_POSIF_HSC_TRIGGER_EDGE_RISING,
//  .pwm_sync               = MOTOR0_BLDC_SCALAR_POSIF_PWM_SYNC_SIGNAL_SEL
//};



/**********************************************************************************************************************
* API IMPLEMENTATION
**********************************************************************************************************************/
/*
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * This function initializes the POSIF peripheral in multichannel mode.
 * Enables below Multi-channel pattern shadow transfer event and connects to interrupt node
 */
void PMSM_FOC_POSIF_Init()
{
  XMC_POSIF_Init(PMSM_FOC_POSIF_MODULE, &PMSM_FOC_POSIF_CONFIG);
}

#endif //End of #if (USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)
