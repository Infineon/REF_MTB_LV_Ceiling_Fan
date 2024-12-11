/**
 * @file adc.h
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
 * @file adc.h
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
#ifndef PMSM_FOC_MCUINIT_ADC_H_
#define PMSM_FOC_MCUINIT_ADC_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

#include "..\Configuration\pmsm_foc_variables_scaling.h"

/* To use ADC calibration. From IFX example project "ADC_Cal_BG_Source_Example_XMC13". */

/*********************************************************************************************************************
 * MACRO
 ********************************************************************************************************************/
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
#define VADC_IDC_SS_RESULT_REG        15U
#define VADC_IDC_SS_RESULT_REG_FIFO1  14U
#define RESULTS_ADCTZ12       0 /* ADC results are ADCTz1 / ADCTz2 of SVM with Pseudo Zero Vectors (PZV) */
#define RESULTS_ADC34         1 /* To indicate the ADC results are ADC3 / ADC4 */
#define RESULTS_STANDARD_SVM  2 /* ADC results are for standard SVM (3, 4 or 7-segment switching sequences) */
#endif

/*********************************************************************************************************************
 * ENUMS
 ********************************************************************************************************************/
typedef enum SHS_GAIN_FACTOR
{
  SHS_GAIN_FACTOR_1 = 0,   /**< Select gain factor 1 */
  SHS_GAIN_FACTOR_3,       /**< Select gain factor 3 */
  SHS_GAIN_FACTOR_6,       /**< Select gain factor 6 */
  SHS_GAIN_FACTOR_12       /**< Select gain factor 12 */
}SHS_GAIN_FACTOR_t;


/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
typedef struct ADCType
{
  uint16_t ADC_Iu;					 /* For 3-shunt phase current sensing */
  uint16_t ADC_Iv;
  uint16_t ADC_Iw;

  uint32_t ADC_Bias_Iu;			/* Bias of ADC Iu. */
  uint32_t ADC_Bias_Iv;			/* Bias of ADC Iv. */
  uint32_t ADC_Bias_Iw;			/* Bias of ADC Iw. */

	int32_t ADC_Pos_Iu;				/* +Iu. For 2-shunt phase current sensing, no external Op-Amps */
	int32_t ADC_Pos_Iv;				/* +Iv */

	uint16_t ADCTrig_Point;		/* ADC trigger position for 2or3-shunt current sensing */

	int32_t ADC_POT;				  /* ADC Value of potentiometer (POT) */
	int32_t ADC_DCLink;				/* ADC Value of inverter DC link voltage Vdc */
  int32_t ADC_IDCLink;      /* ADC Value of inverter DC link current Idc*/
	int32_t ADC_Bias;				  /* ADC for bias of dc-link current amplifier, or on-chip gain */
	uint32_t ADC_ResultTz1;			/* ADC value (12-bit) of first motor phase current ADC sampling */
	uint32_t ADC_ResultTz2;			/* For single-shunt current sensing. */
	uint32_t ADC_Result3;
	uint32_t ADC_Result4;

	int32_t ADC_Result1;			/* Dedicated RAM for single-shunt 3-Phase Current Reconstruction */
	int32_t ADC_Result2;

	uint16_t ADC3Trig_Point;	/* Trigger position for ADC3, of single-shunt current sensing */
	uint16_t ADC4Trig_Point;	/* Trigger position for ADC4 */

  #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
	  // For single-shunt current sensing:
	  uint16_t Result_Flag;     // Flag to indicate ADC results Tz1/2 (if Flag=RESULTS_ADCTZ12) or 3/4 (if Flag=RESULTS_ADC34)
	#endif


	  uint16_t BEMF_U;        // BEMF for phase U, V and W.
	  uint16_t BEMF_V;
	  uint16_t BEMF_W;
	  uint16_t BEMF_Max;        // Maximum value of BEMF_U and BEMF_V.
	  uint16_t BEMF_UV_Threshold;   // Threshold, above it BEMF_U and BEMF_V are considered high.

#if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)
	  int32_t adc_res_bemf_u;
	  int32_t adc_res_bemf_v;
	  int32_t adc_res_bemf_w;
#endif


}ADCType;

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC module with the associated configuration structure for 3-shunt phase current sensing. <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void pmsm_phasecurrent_init(void);


void pmsm_adc_module_init(void);
/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC channel for potentiameter voltage sensing.  <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void pmsm_adc_pot_init(void);

/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC channel for DC Link voltage sensing.  <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void pmsm_adc_dclink_init(void);

/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC channel for Catch Free Running BEMF voltage sensing.  <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void ADC_CFR_Init(void);
void PMSM_FOC_VADC_BEMF_Init(void);

#if(USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)
void PMSM_FOC_VADC_IPD_Init(void);
#endif

#endif /* MCUINIT_ADC_H_ */

/**
 * @}
 */

/**
 * @}
 */
