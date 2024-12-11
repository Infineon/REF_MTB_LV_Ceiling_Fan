/**
 * @file pmsm_foc_rotor_ipd.c
 * @date 09 May, 2019
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
/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "../MCUInit/posif.h"
#include "../Configuration/pmsm_foc_variables_scaling.h"
#include "../MIDSys/pmsm_foc_rotor_ipd.h"
#include "../MCUInit/ccu8.h"
#include "../ControlModules/pmsm_foc_functions.h"

#if(IPD_TEST_FLAG == ENABLED)
int32_t ipd_data[IPD_TEST_CNT];
uint16_t ipd_cnt = 0;
#endif

#if (USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)
/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * EXTERN
 **********************************************************************************************************************/

 extern XMC_VADC_QUEUE_ENTRY_t VADC_QueueEntry_IPD;
 extern FOCOutputType FOCOutput;                 /* Output for FOC LIB. */
 extern PMSM_FOC_PLL_ESTIMATOR_t PMSM_FOC_PLL_ESTIMATOR;
 extern MotorControlType Motor; /* Motor control information */

/***********************************************************************************************************************
 * GLOBAL DATA
 **********************************************************************************************************************/

 PMSM_FOC_ROTOR_IPD_t PMSM_FOC_Rotor_IPD =
{
  .status = PMSM_FOC_ROTOR_IPD_STATUS_UNKNOWN,
  .ind_sense_pat = {
		  	  	  	  PMSM_FOC_ROTOR_IPD_SENSE_MC_PAT_A,
		  	  	  	  PMSM_FOC_ROTOR_IPD_SENSE_MC_PAT_B,
		  	  	  	  PMSM_FOC_ROTOR_IPD_SENSE_MC_PAT_C,
		  	  	  	  PMSM_FOC_ROTOR_IPD_SENSE_MC_PAT_D,
		  	  	  	  PMSM_FOC_ROTOR_IPD_SENSE_MC_PAT_E,
		  	  	  	  PMSM_FOC_ROTOR_IPD_SENSE_MC_PAT_F
                    },
  .ind_sense_pulse_width = (uint32_t)IPD_SENSE_CCU8_PULSE_WIDTH_COUNT,
  .ind_sense_current_decay = (uint32_t)IPD_SENSE_CCU8_CURRENT_DECAY_COUNT,
  .ind_sense_adc_result = {0,0,0,0,0,0},
  /* phase current */
  .ind_sense_adc_reg_num = {
  	  	  	  	  	  	  	  VADC_IV_RESULT_REG, VADC_IU_RESULT_REG, VADC_IW_RESULT_REG,
							                VADC_IV_RESULT_REG, VADC_IU_RESULT_REG, VADC_IW_RESULT_REG

  },
  .ind_sense_adc_grp_ptr = {
		                          VADC_IV_GROUP,  VADC_IU_GROUP,   VADC_IW_GROUP,
							                VADC_IV_GROUP,  VADC_IU_GROUP,   VADC_IW_GROUP
                           },

  .ind_sense_pat_index = (uint8_t)0,
  .read_current_enable = (uint8_t)0
};

/* Request the LLD to insert the channel */
XMC_VADC_QUEUE_ENTRY_t VADC_QueueEntry_IPD =
{
  .channel_num        = VADC_IU_CHANNEL,
  .external_trigger   = true,
  .generate_interrupt = false,
  .refill_needed      = false
};

/************************END : Inductive Sensing  ************************************/

/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * API IMPLEMENTATION
 **********************************************************************************************************************/
#if (CURRENT_SENSING ==  USER_SINGLE_SHUNT_CONV)
/* This function setups CCU8 for inductance sensing. */
void PMSM_FOC_Rotor_IPD_Init(void)
{
  /* Pull CCUCON signal to low to stop PWM */
  XMC_SCU_SetCcuTriggerLow(PMSM_FOC_CCU8_CCU4_SYNC_START);

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  /* XMC_VADC_GROUP_QueueClearReqSrcEvent(VADC_ISS_GROUP); */  /*does this function only clear SR for SR0?? */
  VADC_ISS_GROUP->SEFCLR = (uint32_t)VADC_G_SEFCLR_SEV1_Msk;
  #if (VADC_ISS_GROUP_NO == 0)
  NVIC_ClearPendingIRQ(VADC0_G0_1_IRQn);
  #else
  NVIC_ClearPendingIRQ(VADC0_G1_1_IRQn);
  #endif
#endif

  /*POSIF initialization for MCM */
  PMSM_FOC_POSIF_Init();

  /* CCU8 initialization for inductive sensing */
  PMSM_FOC_CCU8_InductiveSense_Init();

  PMSM_FOC_VADC_IPD_Init();

  /* Change the rotor initial position identification status to in progress. */
  PMSM_FOC_Rotor_IPD.status = PMSM_FOC_ROTOR_IPD_STATUS_IN_PROGRESS;

//  /* Start PWM Timers */
  XMC_SCU_SetCcuTriggerHigh(PMSM_FOC_CCU8_SYNC_START);
}
#else
/* This function setups CCU8 for inductance sensing. */
void PMSM_FOC_Rotor_IPD_Init(void)
{
  /* Pull CCUCON signal to low to stop PWM */
  XMC_SCU_SetCcuTriggerLow(PMSM_FOC_CCU8_CCU4_SYNC_START);

  /*POSIF initialization for MCM */
  PMSM_FOC_POSIF_Init();

  /* CCU8 initialization for inductive sensing */
  PMSM_FOC_CCU8_InductiveSense_Init();

  PMSM_FOC_VADC_IPD_Init();

  /* Insert queue entry for IPD sensing */
  VADC_QueueEntry_IPD.channel_num = PMSM_FOC_Rotor_IPD.ind_sense_adc_reg_num[PMSM_FOC_Rotor_IPD.ind_sense_pat_index];
  XMC_VADC_GROUP_QueueInsertChannel(PMSM_FOC_Rotor_IPD.ind_sense_adc_grp_ptr[PMSM_FOC_Rotor_IPD.ind_sense_pat_index], VADC_QueueEntry_IPD);

  /* Change the rotor initial position identification status to in progress. */
  PMSM_FOC_Rotor_IPD.status = PMSM_FOC_ROTOR_IPD_STATUS_IN_PROGRESS;
  /* Start PWM Timers */
  XMC_SCU_SetCcuTriggerHigh(PMSM_FOC_CCU8_SYNC_START);
}
#endif

void PMSM_FOC_Rotor_Init_Pos_Detect(void)
{
  uint32_t result_temp = 0U;
  int32_t adc_res = 0;

  if(PMSM_FOC_Rotor_IPD.ind_sense_pat_index < PMSM_FOC_ROTOR_IPD_SENSING_PAT_NUM)
  {
    /* Stop timers (just clear the start bit so they can restart on the rising edge) */
    XMC_SCU_SetCcuTriggerLow((uint32_t) PMSM_FOC_CCU8_SYNC_START);

    /* High side/Low side SW operated by same ST signal */
    CCU8_MODULE_PHASE_U->CHC |= 0x100;
    CCU8_MODULE_PHASE_V->CHC |= 0x100;
    CCU8_MODULE_PHASE_W->CHC |= 0x100;

    XMC_POSIF_MCM_SetMultiChannelPattern(PMSM_FOC_POSIF_MODULE,
        (uint16_t) PMSM_FOC_Rotor_IPD.ind_sense_pat[PMSM_FOC_Rotor_IPD.ind_sense_pat_index]);
    XMC_POSIF_MCM_UpdateMultiChannelPattern(PMSM_FOC_POSIF_MODULE);

    if(PMSM_FOC_Rotor_IPD.read_current_enable == 1U)
    {
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      while ((result_temp & VADC_G_RES_VF_Msk) == 0U)
      {
        result_temp = XMC_VADC_GROUP_GetDetailedResult((XMC_VADC_GROUP_t *)VADC_ISS_GROUP, VADC_IDC_SS_RESULT_REG);
      }
      /* in single-shunt, all three ADC.adc_bias_ix are the same */
      adc_res = (int32_t)((result_temp & 0x0000FFFFU) - ADC.ADC_Bias_Iu);
//      adc_res = (int32_t)((result_temp & 0x0000FFFFU));

#else
      /* Wait for ADC result */
      while ((result_temp & VADC_G_RES_VF_Msk) == 0U)
      {
        result_temp = XMC_VADC_GROUP_GetDetailedResult((XMC_VADC_GROUP_t *)PMSM_FOC_Rotor_IPD.ind_sense_adc_grp_ptr[PMSM_FOC_Rotor_IPD.ind_sense_pat_index],
                                                    PMSM_FOC_Rotor_IPD.ind_sense_adc_reg_num[PMSM_FOC_Rotor_IPD.ind_sense_pat_index]);
      }
      if((PMSM_FOC_Rotor_IPD.ind_sense_adc_grp_ptr[PMSM_FOC_Rotor_IPD.ind_sense_pat_index] == VADC_IU_GROUP) &&
                (PMSM_FOC_Rotor_IPD.ind_sense_adc_reg_num[PMSM_FOC_Rotor_IPD.ind_sense_pat_index] == VADC_IU_CHANNEL))
      {
      /* Subtract the ADC offset */
        adc_res = (int32_t)((result_temp & 0x0000FFFFU) - ADC.ADC_Bias_Iu);
      }
      else if((PMSM_FOC_Rotor_IPD.ind_sense_adc_grp_ptr[PMSM_FOC_Rotor_IPD.ind_sense_pat_index] == VADC_IV_GROUP) &&
                (PMSM_FOC_Rotor_IPD.ind_sense_adc_reg_num[PMSM_FOC_Rotor_IPD.ind_sense_pat_index] == VADC_IV_CHANNEL))
      {
      /* Subtract the ADC offset */
        adc_res = (int32_t)((result_temp & 0x0000FFFFU) - ADC.ADC_Bias_Iv);
      }
      else
      {
        /* Subtract the ADC offset */
        adc_res = (int32_t)((result_temp & 0x0000FFFFU) - ADC.ADC_Bias_Iw);
      }
#endif

      PMSM_FOC_Rotor_IPD.ind_sense_adc_result[PMSM_FOC_Rotor_IPD.ind_sense_pat_index++] = (int16_t)(adc_res);
      PMSM_FOC_Rotor_IPD.read_current_enable = 0U;

      /* Turn off all switches */
      XMC_POSIF_MCM_SetMultiChannelPattern(PMSM_FOC_POSIF_MODULE,ALL_OFF);
      XMC_POSIF_MCM_UpdateMultiChannelPattern(PMSM_FOC_POSIF_MODULE);

      /* Update period value to produce configured delay */
      XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU8_MODULE_PHASE_U, (uint16_t)PMSM_FOC_Rotor_IPD.ind_sense_current_decay);

      /* Enable CCU8 shadow transfer */
      CCU8_MODULE->GCSS |= (uint32_t)(XMC_CCU8_SHADOW_TRANSFER_SLICE_0|XMC_CCU8_SHADOW_TRANSFER_SLICE_1|XMC_CCU8_SHADOW_TRANSFER_SLICE_2|XMC_CCU8_SHADOW_TRANSFER_SLICE_3);
    }
    else
    {
#if(BEMF_PIN_AS_TEST == ENABLED)
//XMC_GPIO_ToggleOutput(BEMF_W_PIN);
//XMC_GPIO_PORT2->OMR = 0x10001U;       /* P2.0 */
#endif
      PMSM_FOC_Rotor_IPD.read_current_enable = 1U;
      /* Update period value to produce configured delay */
      XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU8_MODULE_PHASE_U, (uint16_t)PMSM_FOC_Rotor_IPD.ind_sense_pulse_width);
      /*Enable CCU8 shadow transfer*/
      CCU8_MODULE->GCSS |= (uint32_t)(XMC_CCU8_SHADOW_TRANSFER_SLICE_0|XMC_CCU8_SHADOW_TRANSFER_SLICE_1|XMC_CCU8_SHADOW_TRANSFER_SLICE_2|XMC_CCU8_SHADOW_TRANSFER_SLICE_3);

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      VADC_ISS_GROUP->VFR = (uint32_t)(1<<VADC_IDC_SS_RESULT_REG);   /* XMC1400 UM P984 */		/*xmc1300 um p706*/
#else
      /* Clear the ADC Valid flag for instantaneous phase current measurement.*/
      PMSM_FOC_Rotor_IPD.ind_sense_adc_grp_ptr[PMSM_FOC_Rotor_IPD.ind_sense_pat_index]->VFR = (uint32_t)(1<<PMSM_FOC_Rotor_IPD.ind_sense_adc_reg_num[PMSM_FOC_Rotor_IPD.ind_sense_pat_index]);

      /* Flush the queue for next iteration */
      XMC_VADC_GROUP_QueueFlushEntries(VADC_G0);
      XMC_VADC_GROUP_QueueFlushEntries(VADC_G1);

      /* Insert the queue entry for next iteration */
      VADC_QueueEntry_IPD.channel_num = PMSM_FOC_Rotor_IPD.ind_sense_adc_reg_num[PMSM_FOC_Rotor_IPD.ind_sense_pat_index];
      XMC_VADC_GROUP_QueueInsertChannel(PMSM_FOC_Rotor_IPD.ind_sense_adc_grp_ptr[PMSM_FOC_Rotor_IPD.ind_sense_pat_index], VADC_QueueEntry_IPD);
#endif

      result_temp = 0U;
    }   /* end of if(PMSM_FOC_Rotor_IPD.read_current_enable == 1U) */

    /* Start Timers */
    XMC_SCU_SetCcuTriggerHigh((uint32_t)PMSM_FOC_CCU8_SYNC_START);
  } /* end of (PMSM_FOC_Rotor_IPD.ind_sense_pat_index < PMSM_FOC_ROTOR_IPD_SENSING_PAT_NUM) */
  else
  {
    /* Stop timers (just clear the start bit so they can restart on the rising edge) */
    XMC_SCU_SetCcuTriggerLow((uint32_t)PMSM_FOC_CCU8_SYNC_START);

    /* Turn off all switches */
    XMC_POSIF_MCM_SetMultiChannelPattern(PMSM_FOC_POSIF_MODULE,ALL_OFF);
    XMC_POSIF_MCM_UpdateMultiChannelPattern(PMSM_FOC_POSIF_MODULE);

    /* Inductive sensing completed, updates the rotor detected position and voltage vector to be applied based upon the motor direction configured */
    PMSM_FOC_Rotor_IPD.status = PMSM_FOC_CalcIndSenPos();

  /* Start Timers */
    XMC_SCU_SetCcuTriggerHigh((uint32_t)PMSM_FOC_CCU8_SYNC_START);
  }
}

/*
 * Finds the rotor position based upon the current sensed from six applied patterns during inductive sensing.
 * Determines the patterns to be applied for rotating motor in intended direction.
 */
PMSM_FOC_ROTOR_IPD_STATUS_t PMSM_FOC_CalcIndSenPos(void)
 {
   int16_t delta[3U];
   PMSM_FOC_ROTOR_IPD_STATUS_t status;
   uint8_t count;
   volatile int32_t rotor_pos;
   int16_t *curr = &PMSM_FOC_Rotor_IPD.ind_sense_adc_result[0U];

   /* Calculate the deltas between measured current readings */
   for (count = 0U; count < 6U; count += 2U)
   {
     delta[(uint8_t)(count >>1U) ] = curr[count] - curr[count + 1U];
   }

   /* Check if there is no input range error */

   /* Finds rotor position and determines voltage vector that will cause proper rotation */
   if ( (delta[0U] >= delta[1U]) && ((delta[0U] >= delta[2U])) )
   {
       /* delta[0] the biggest */
       if (delta[1U] >= delta[2U])
       {
           /* delta[0] > delta[1] > delta[2] */
           /* Angle between 330 degree and 30 degree */
            rotor_pos = PMSM_FOC_ANGLE_000_DEGREE_Q31;
       }
       else
       {
           /* delta[0] > delta[2] > delta[1] */
           /* Angle between 270 degree and 330 degree */
            rotor_pos = PMSM_FOC_ANGLE_300_DEGREE_Q31;
       }
   }
   else if ( (delta[1U] >= delta[0U]) && ((delta[1U] >= delta[2U])) )
   {
       /* delta[1] the biggest */
       if (delta[0U] >= delta[2U])
       {
           /* delta[1] > delta[0] > delta[2] */
           /* Angle between 30 degree and 90 degree */
            rotor_pos = PMSM_FOC_ANGLE_060_DEGREE_Q31;
       }
       else
       {
           /* delta[1] > delta[2] > delta[0] */
           /* Angle between 90 degree and 150 degree */
            rotor_pos = PMSM_FOC_ANGLE_120_DEGREE_Q31;
       }
   }
   else
   {
       /* delta[2] is the biggest */
       if (delta[0U] >= delta[1U])
       {
           /* delta[2] > delta[0] > delta[1] */
           /* Angle between 210 degree and 270 degree */
            rotor_pos = PMSM_FOC_ANGLE_240_DEGREE_Q31;
       }
       else
       {
           /* delta[2] > delta[1] > delta[0] */
           /* Angle between 150 degree and 210 degree */
            rotor_pos = PMSM_FOC_ANGLE_180_DEGREE_Q31;
       }
   }
 /* Initialize PLL estimator with identified rotor position */
 PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31 = rotor_pos;
 status = PMSM_FOC_ROTOR_IPD_STATUS_COMPLETED;


   return status;
 }


#endif //end of #if (USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)
