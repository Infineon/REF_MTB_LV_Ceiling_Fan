/**
 * @file pmsm_foc_stall_detection.c
 * @date 28 Feb, 2022
 *
 * @cond
 *********************************************************************************************************************
 * PMSM FOC Motor Control Library
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

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/


#include "pmsm_foc_stall_detection.h"
#include "..\Configuration\pmsm_foc_variables_scaling.h"


PMSM_FOC_STALL_DETECTION_t PMSM_FOC_STALL_DETECTION;

/*************************************************************************************************/
/**
 * @brief
 *
 * @param None
 *
 * @retval  None
 */
void PMSM_FOC_Stall_Detection_Init(void)
{
  PMSM_FOC_STALL_DETECTION.Stall_cnt = 0;
  PMSM_FOC_STALL_DETECTION.Coeff_N_iq = 3;
  PMSM_FOC_STALL_DETECTION.iq_flted = 0;
  PMSM_FOC_STALL_DETECTION.Stall_cnt_threshold = 100;
  /* In case the stall torque is not initialized, set a default value */
  if(PMSM_FOC_STALL_DETECTION.Stall_torque_Q15 == 0)
  {
    PMSM_FOC_STALL_DETECTION.Stall_torque_Q15 = (int32_t)(STALL_TORQUE_NM * (1<<15));   // 1638 = 0.05Nm * 32768
  }
  /* i = torque/KT = torque * (1/KT) */
  PMSM_FOC_STALL_DETECTION.Stall_iq_threshold_Q15 =  STALL_CURRENT; //(int32_t)((int32_t)(((int32_t)(PMSM_FOC_STALL_DETECTION.Stall_torque_Q15 * KT_CONST_INV_Q12) >> 12) * I_NORM_Q15) >> 15);

  /* Clear the flag at initialization */
  PMSM_FOC_STALL_DETECTION.Stall_detected = 0;
  PMSM_FOC_STALL_DETECTION.Blanking_cnt = 0;
  PMSM_FOC_STALL_DETECTION.Stall_detection_enabled = 1; // Enable stall detection by default
}


/**
 * @brief
 *
 * @param None
 *
 * @retval  None
 */
__attribute__((section(".ram_code"))) void PMSM_FOC_Stall_Detection(void)
{
  /* Do the fitering for the torque current */
  PMSM_FOC_STALL_DETECTION.iq_flted += \
    (int32_t)((FOCOutput.I_q - PMSM_FOC_STALL_DETECTION.iq_flted) >> PMSM_FOC_STALL_DETECTION.Coeff_N_iq);

  /* Start a blanking counter after stable status. This is to avoid startup torque trigger the stall detaction */
  if(Motor.State == FOC_CLOSED_LOOP)
  {
    if(PMSM_FOC_STALL_DETECTION.Blanking_cnt <= (STALL_DETECTION_BLANKING +1))
    {
      PMSM_FOC_STALL_DETECTION.Blanking_cnt++;
    }
  }

  if ((Motor.State == FOC_CLOSED_LOOP)  && (PMSM_FOC_STALL_DETECTION.Blanking_cnt >= STALL_DETECTION_BLANKING))
  {
    /* if the current is bigger than the threshold or
        the motor cannot transist to stable status */
    if((PMSM_FOC_STALL_DETECTION.iq_flted >= PMSM_FOC_STALL_DETECTION.Stall_iq_threshold_Q15))
    {
      /* Start the counter for the stall determination */
      PMSM_FOC_STALL_DETECTION.Stall_cnt ++;
      if(PMSM_FOC_STALL_DETECTION.Stall_cnt >= PMSM_FOC_STALL_DETECTION.Stall_cnt_threshold)
      {
        PMSM_FOC_STALL_DETECTION.Stall_detected = TRUE;
        PMSM_FOC_STALL_DETECTION.Stall_cnt = 0;
        //motor_stop();
        Motor.motorstartstop = 0;
      }
    }
    else
    {
      PMSM_FOC_STALL_DETECTION.Stall_cnt = 0;
    }
  }
}


