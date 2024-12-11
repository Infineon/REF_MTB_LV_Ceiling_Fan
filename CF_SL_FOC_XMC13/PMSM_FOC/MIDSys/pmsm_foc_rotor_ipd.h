/**
 * @file pmsm_foc_rotor_ipd.h
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
#ifndef PMSM_FOC_ROTOR_IPD_H
#define PMSM_FOC_ROTOR_IPD_H

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup MIDSys
 * @{
 */
/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "../MCUInit/posif.h"
#include "../Configuration/pmsm_foc_variables_scaling.h"

/***********************************************************************************************************************
 * MACRO
 **********************************************************************************************************************/

#if(IPD_TEST_FLAG == ENABLED)
#define IPD_TEST_CNT      2
extern int32_t ipd_data[IPD_TEST_CNT];
extern uint16_t ipd_cnt;
#endif

#define PMSM_FOC_ROTOR_IPD_SENSING_PAT_NUM (6U)

#define   PMSM_FOC_ROTOR_IPD_SENSE_MC_PAT_A            (WOFF_VL_UH)  /*!< Phase pattern corresponding to U+,V- */
#define   PMSM_FOC_ROTOR_IPD_SENSE_MC_PAT_B            (WOFF_VH_UL)  /*!< Phase pattern corresponding to U-,V+ */
#define   PMSM_FOC_ROTOR_IPD_SENSE_MC_PAT_C            (WL_VH_UOFF)  /*!< Phase pattern corresponding to V+,W- */
#define   PMSM_FOC_ROTOR_IPD_SENSE_MC_PAT_D            (WH_VL_UOFF)  /*!< Phase pattern corresponding to V-,W+ */
#define   PMSM_FOC_ROTOR_IPD_SENSE_MC_PAT_E            (WH_VOFF_UL)  /*!< Phase pattern corresponding to W+,U- */
#define   PMSM_FOC_ROTOR_IPD_SENSE_MC_PAT_F            (WL_VOFF_UH)  /*!< Phase pattern corresponding to W-,U+ */


/***********************************************************************************************************************
 * ENUMS
 **********************************************************************************************************************/
/**
 * This enumerates the inductive sensing initial rotor identification status
 */
typedef enum PMSM_FOC_ROTOR_IPD_STATUS
{
  PMSM_FOC_ROTOR_IPD_STATUS_UNKNOWN        = 0U,       /*!< Inductive sensing not executed */
  PMSM_FOC_ROTOR_IPD_STATUS_IN_PROGRESS    = 1U,       /*!< Inductive sensing in progress */
  PMSM_FOC_ROTOR_IPD_STATUS_COMPLETED      = 2U        /*!< Inductive sensing completed */
} PMSM_FOC_ROTOR_IPD_STATUS_t;

/***********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/
/**  @brief structure inductive sensing */
typedef struct PMSM_FOC_ROTOR_IPD
{
  PMSM_FOC_ROTOR_IPD_STATUS_t status;                                   /*!< Inductive sensing status information */
  uint32_t ind_sense_pat[PMSM_FOC_ROTOR_IPD_SENSING_PAT_NUM];           /*!< During inductive sensing this patterns are applied */
  uint32_t ind_sense_pulse_width;                                       /*!< Inductive sense applied MCM pulse width */
  uint32_t ind_sense_current_decay;                                     /*!< After inductive sense pattern applied wait for current to decay */
  VADC_G_TypeDef *ind_sense_adc_grp_ptr[PMSM_FOC_ROTOR_IPD_SENSING_PAT_NUM];   /*!< Phase sensing ADC group number */
  int16_t ind_sense_adc_result[PMSM_FOC_ROTOR_IPD_SENSING_PAT_NUM];     /*!< Captured VADC results after each inductive sense pattern applied */
  uint8_t ind_sense_adc_reg_num[PMSM_FOC_ROTOR_IPD_SENSING_PAT_NUM];    /*!< Phase sensing ADC result register number */
  uint8_t ind_sense_pat_index;                                          /*!< Inductive sensing applied pattern index */
  uint8_t read_current_enable;                                          /*!< Flag to indicate to read the current */
} PMSM_FOC_ROTOR_IPD_t;

/***********************************************************************************************************************
 * EXTERN
 **********************************************************************************************************************/
extern PMSM_FOC_ROTOR_IPD_t PMSM_FOC_Rotor_IPD;
/***********************************************************************************************************************
 * API Prototypes
 **********************************************************************************************************************/
#if (USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)

 /* This function setups CCU8 for inductance sensing. */
 void PMSM_FOC_Rotor_IPD_Init(void);

/*
 * Finds the rotor position based upon the current sensed from six applied patterns during inductive sensing.
 * Determines the patterns to be applied for rotating motor in intended direction.
 */
 PMSM_FOC_ROTOR_IPD_STATUS_t PMSM_FOC_CalcIndSenPos(void);

 void PMSM_FOC_Rotor_Init_Pos_Detect(void);


#endif /*end of #if (USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING) */

 /**
  * @}
  */

 /**
  * @}
  */

 #endif /* End of #ifndef PMSM_FOC_ROTOR_IPD_H */

