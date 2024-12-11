/**
 * @file pmsm_foc_ip.h
 * @date 2016-09-30
 *
 * @cond
 *********************************************************************************************************************
 * PMSM FOC Motor Control Library
 *
 * Copyright (c) 2015-2016, Infineon Technologies AG
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
 * @file pmsm_foc_ip.h
 * @date 30 Sep, 2016
 * @version 1.0.0
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

#ifndef PMSM_FOC_FOCLIB_PMSM_FOC_PLL_ESTIMATOR_H_
#define PMSM_FOC_FOCLIB_PMSM_FOC_PLL_ESTIMATOR_H_

#include <XMC1300.h>											// SFR declarations of the selected device
#include "..\PMSM_FOC\ControlModules\pmsm_foc_functions.h"

/**********************************************************************************************************************
* EXTERN
**********************************************************************************************************************/
extern PMSM_FOC_PLL_PI_t PMSM_FOC_PLL_PI;
extern PMSM_FOC_PLL_ESTIMATOR_t PMSM_FOC_PLL_ESTIMATOR;

/**
 * This function calculates magnitude of stator current from alpha,beta current component.
 * It uses the CORDIC to calculate magnitude. Function loads alpha beta to CORDIC register
 * and initiates the calculations.
 * @param Vref_AngleQ31
 * @param I_Alpha_1q31
 * @param I_Beta_1q31
 * @return None
 */
__attribute__((section(".ram_code"))) void PMSM_FOC_PLL_Imag(int32_t Vref_AngleQ31, int32_t i_alpha_1q31, int32_t i_beta_1q31);
/**
 * This function reads the stator current magnitude result from CORDIC result register.
 * @param PLL Estimator structure pointer
 * @return None
 */
__attribute__((section(".ram_code"))) void PMSM_FOC_PLL_ImagGetResult(PMSM_FOC_PLL_ESTIMATOR_t* const handle_ptr);
/**
 * This function calculates the PLL observer internal signal value required for the estimator.
 * @param vref_32 Stator voltage magnitude
 * @param PMSM_FOC_PLL_ESTIMATOR_t handle pointer
 * @return None
 */
__attribute__((section(".ram_code"))) void PMSM_FOC_PLL_Vref (uint32_t vref_32, PMSM_FOC_PLL_ESTIMATOR_t* const handle_ptr);
/**
 * This function reads the calculated PLL observer internal signal value required for the estimator.
 * @param PLL Estimator structure pointer
 * @return None
 */
__attribute__((section(".ram_code"))) void PMSM_FOC_PLL_VrefGetResult(PMSM_FOC_PLL_ESTIMATOR_t* const handle_ptr);
/**
 * This function estimates the motor speed and rotor angle.
 * @param PLL Estimator structure pointer
 * @param PLL PI controller structure pointer
 * @return None
 */
__attribute__((section(".ram_code"))) void PMSM_FOC_PLL_GetPosSpeed(PMSM_FOC_PLL_ESTIMATOR_t* const handle_ptr,PMSM_FOC_PLL_PI_t* const pll_pi_handle_ptr);

#endif /* PMSM_FOC_FOCLIB_PMSM_FOC_PLL_ESTIMATOR_H_ */
