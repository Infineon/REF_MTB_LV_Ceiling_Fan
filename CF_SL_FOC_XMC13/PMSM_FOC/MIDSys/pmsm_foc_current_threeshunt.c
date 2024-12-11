/**
 * @file pmsm_foc_current_threeshunt.c
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
 * @file pmsm_foc_current_threeshunt.c
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
#include "pmsm_foc_current_threeshunt.h"

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/


/***********************************************************************************************************************
 * GLOBAL DATA
***********************************************************************************************************************/
extern ADCType ADC;

/* Motor current and current space vector */
extern CurrentType Current;

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

/* API to initialize value of 12-bit VADC bias, external amplifiers bias at about 2.5V */
void pmsm_foc_get_current_bias(void)
{
	/* Init ADC bias */
	ADC.ADC_Bias = IU_ADC_BIAS;

	ADC.ADC_Bias_Iu = IU_ADC_BIAS;
	ADC.ADC_Bias_Iv = IV_ADC_BIAS;
	ADC.ADC_Bias_Iw = IW_ADC_BIAS;

	ADC.ADC_Iu = IU_ADC_BIAS;
	ADC.ADC_Iv = IV_ADC_BIAS;
	ADC.ADC_Iw = IW_ADC_BIAS;
}

#if (CURRENT_SENSING ==  USER_THREE_SHUNT_SYNC_CONV)
/* API to read ADC result of the 3 shunt current */
__attribute__((section(".ram_code"))) void pmsm_foc_get_adcphasecurrent(uint16_t Previous_SVM_SectorNo, uint16_t New_SVM_SectorNo,
                                       ADCType* const HandlePtr)
{
  uint16_t I1;
  uint16_t I2;
  uint16_t I3;

  /* Read current ADC (ADC synchronous conversion) */
  I1 = VADC_I1_GROUP->RES[VADC_I1_RESULT_REG];
  I2 = VADC_I2_GROUP->RES[VADC_I2_RESULT_REG];
  I3 = VADC_I3_GROUP->RES[VADC_I3_RESULT_REG];


  /* 3-phase current reconstruction */
  switch (Previous_SVM_SectorNo)
  {
    case 0:
    case 5:
      /* Sectors A and F. ADC sequences - Iw -> Iv -> Iu */
      HandlePtr->ADC_Iu = I3;
      HandlePtr->ADC_Iv = I2;
      HandlePtr->ADC_Iw = I1;
      break;
    case 1:
    case 2:
      /* Sectors B and C. ADC sequences - Iw -> Iu -> Iv */
      HandlePtr->ADC_Iu = I2;
      HandlePtr->ADC_Iv = I3;
      HandlePtr->ADC_Iw = I1;
      break;

    default:
      /* Process for all other cases, Sectors D and E. ADC sequences - Iu -> Iv -> Iw */
      HandlePtr->ADC_Iu = I1;
      HandlePtr->ADC_Iv = I2;
      HandlePtr->ADC_Iw = I3;
      break;
  }

  /* If SVM sector changed */
  if (New_SVM_SectorNo != Previous_SVM_SectorNo)
  {
    /* Rotating ADC alias */
    switch (New_SVM_SectorNo)
    {
      case 0:
      case 5:
         /* Sectors A and F. ADC sequences - Iw -> Iv -> Iu */
        VADC_G1->ALIAS = (((uint32_t)VADC_IU_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G1_CHANNEL);
        VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL);
        break;

      case 1:
      case 2:
        /*  Sectors B and C. ADC sequences - Iw -> Iu -> Iv */
        VADC_G1->ALIAS = (((uint32_t)VADC_IV_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G1_CHANNEL);
        VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G0_CHANNEL);
        break;

      default:
        /* Process for all other cases, Sectors D and E. ADC sequences - Iu -> Iv -> Iw */
        VADC_G1->ALIAS = (((uint32_t)VADC_IW_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G1_CHANNEL);
        VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL);
        break;
    }
  }
}

#else
void pmsm_foc_get_adcphasecurrent(uint16_t Previous_SVM_SectorNo, uint16_t New_SVM_SectorNo, ADCType* const HandlePtr)
{
  HandlePtr->ADC_Iu = XMC_VADC_GROUP_GetResult(VADC_IU_GROUP,VADC_IU_RESULT_REG);
  HandlePtr->ADC_Iv = XMC_VADC_GROUP_GetResult(VADC_IV_GROUP,VADC_IV_RESULT_REG);
  HandlePtr->ADC_Iw = XMC_VADC_GROUP_GetResult(VADC_IW_GROUP,VADC_IW_RESULT_REG);
}
#endif


