/**
 * @file pmsm_foc_vadc_isr.c
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
 * @file pmsm_foc_vadc_isr.c
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
#include <XMC1300.h>    /* SFR declarations of the selected device */
#include "..\MCUInit\adc.h"

/***********************************************************************************************************************
 * GLOBAL DATA
***********************************************************************************************************************/
extern ADCType ADC;                             /* ADC results, trigger positions. */

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

void pmsm_foc_vadc_source_irqhandler(void)
{
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  if(ADC.Result_Flag == RESULTS_ADCTZ12)
  {
    ADC.ADC_ResultTz1 = (uint16_t)VADC_ISS_GROUP->RES[VADC_IDC_SS_RESULT_REG_FIFO1];
    ADC.ADC_ResultTz2 = (uint16_t)VADC_ISS_GROUP->RES[VADC_IDC_SS_RESULT_REG_FIFO1];
    ADC.Result_Flag = RESULTS_ADC34;
  }
  else if(ADC.Result_Flag == RESULTS_ADC34)
  {
    ADC.ADC_Result3 = (uint16_t)VADC_ISS_GROUP->RES[VADC_IDC_SS_RESULT_REG_FIFO1];
    ADC.ADC_Result4 = (uint16_t)VADC_ISS_GROUP->RES[VADC_IDC_SS_RESULT_REG_FIFO1];
    ADC.Result_Flag = RESULTS_ADCTZ12;

    ADC.ADC_Result1 = (ADC.ADC_Result3 - ADC.ADC_ResultTz1) << 3;
    ADC.ADC_Result2 = (ADC.ADC_Result4 - ADC.ADC_ResultTz2) << 3;
  }
  else
  {
    ADC.ADC_Result1 = (uint16_t)VADC_ISS_GROUP->RES[VADC_IDC_SS_RESULT_REG_FIFO1] - ADC.ADC_Bias;
    ADC.ADC_Result2 = (uint16_t)VADC_ISS_GROUP->RES[VADC_IDC_SS_RESULT_REG_FIFO1] - ADC.ADC_Bias;
    ADC.ADC_Result1 <<= 4;
    ADC.ADC_Result2 <<= 4;
  }
#endif
}


