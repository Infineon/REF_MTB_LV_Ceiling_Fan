/**
 * @file pmsm_foc_ip.c
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

#include "pmsm_foc_ip.h"

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#define CORDIC_CIRCULAR_VECTORING_MODE		(0x62)			/*!< CORDIC: Circular Vectoring Mode (default). MPS: Divide by 2 (default). */
#define CORDIC_CIRCULAR_ROTATION_MODE		(0x6A)			/*!< CORDIC: Circular Rotation Mode. MPS: Divide by 2 (default). */

#define CORDIC_CIRCULAR_MPS_BY_K_SCALED     (311)           /*!< CORDIC MPS/K ->(2/1.64676)* 2^CORDIC_MPS_BY_K_SCALE */
#define CORDIC_MPS_BY_K_SCALE               (8)             /*!< CORDIC MPS/K scaling factor */
#define IS_CORDIC_BUSY                      (MATH->STATC & 0x01) /*!< Returns 1 if CORDIC is busy */

#define ROTOR_ANGLE_005_DEGREE_Q31          (59652324)      /*!< 5° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define ROTOR_ANGLE_010_DEGREE_Q31          (119304647)     /*!< 10° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define ROTOR_ANGLE_090_DEGREE_Q31          (1073741824)    /*!< 90° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */

#define MIN_MAX_LIMIT(Buffer,LimitH,LimitL) ((Buffer) > (LimitH)) ? (LimitH) : (((Buffer) < (LimitL))? (LimitL): (Buffer))
#define MIN(a, b)                           (((a) < (b)) ? (a) : (b))   /*!< macro returning smallest input */
#define MAX(a, b)                           (((a) > (b)) ? (a) : (b))   /*!< macro returning biggest input */

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
PMSM_FOC_PLL_ESTIMATOR_t PMSM_FOC_PLL_ESTIMATOR;

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/**
 * @brief	I_Mag (i.e.: |I|) and (γ-θ)angle for PLL Observer
 * 			I_Mag = K/MPS * sqrt(I_Alpha^2+I_Beta^2)		* Xfinal = K/MPS * sqrt(X^2+Y^2), where K = 1.646760258121.
 * 			γ = atan(I_Beta/I_Alpha)						* Zfinal = Z + atan(Y/X)					(Yfinal = 0).
 *
 * @param	Current.I_Beta_1Q31
 * 			Current.I_Alpha_1Q31
 *
 *@retval	PMSM_FOC_PLL_ESTIMATOR.current_i_mag
 * 			PMSM_FOC_PLL_ESTIMATOR.Delta_IV
 */
__attribute__((section(".ram_code"))) void PMSM_FOC_PLL_Imag(int32_t Vref_AngleQ31, int32_t I_Alpha_1q31, int32_t I_Beta_1q31)
{
  /*  General control of CORDIC Control Register */
  MATH->CON = CORDIC_CIRCULAR_VECTORING_MODE;

 /* Z = -θ¸ of last PWM cycle, to get γ-θ¸ directly by CORDIC */
  MATH->CORDZ = -Vref_AngleQ31;

  /* Y = I_Beta */
  MATH->CORDY = I_Beta_1q31;

  /* X = I_Alpha. Input CORDX data, and auto start of CORDIC calculation */
  MATH->CORDX = I_Alpha_1q31;
}

/**
 * @brief	 Results of CORDIC #2 - |I| and γ-θ
 *
 * @param	MATH->CORRX
 * 			MATH->CORRZ
 *
 *@retval	Current_I_Mag
 *			Delta_IV
 */

__attribute__((section(".ram_code"))) void PMSM_FOC_PLL_ImagGetResult(PMSM_FOC_PLL_ESTIMATOR_t* const handle_ptr)
{
  int32_t iq_temp ;
  /* Read CORDIC result: |I|- 32-bit unsigned */
  handle_ptr->current_i_mag = MATH->CORRX;

  /* Shift to get real result |I| (1Q15) of last CORDIC calculation with overflow protection */
  iq_temp = (int32_t)(((int32_t)(handle_ptr->current_i_mag >> CORDIC_MPS_BY_K_SCALE)) * CORDIC_CIRCULAR_MPS_BY_K_SCALED);
  handle_ptr->current_i_mag = (int16_t)(iq_temp >> CORDIC_SHIFT);

  /* γ-θ¸ by CORDIC directly */
  handle_ptr->delta_vi_angle_q31 = MATH->CORRZ;
}

/**
 * @brief	Vref x sin(γ-θ) for PLL Observer
 *			Vref x cos(γ-θ)=K[|Vref| cos(γ-θ)- 0 X sin(γ-θ)]/MPS		* Xfinal = K[X cos(Z) - Y sin(Z)] / MPS, where K = 1.646760258121.
 * 			Vref x sin(γ-θ)=K[0 X cos(γ-θ) + |Vref|sin(γ-θ)]/MPS		* Yfinal = K[Y cos(Z) + X sin(Z)] / MPS. Select Y= 0. (Zfinal = 0).
 *
 * @param	Delta_IV
 *          PMSM_FOC_PLL_PI.Uk
 * 			PMSM_FOC_INPUT.Vref32
 *
 *@retval	MATH->CORDZ
 *			MATH->CORDX
 *			MATH->CORDY
 */
__attribute__((section(".ram_code"))) void PMSM_FOC_PLL_Vref (uint32_t Vref_32, PMSM_FOC_PLL_ESTIMATOR_t* const handle_ptr)
{
  int32_t wli;
  int32_t temp_wi = 0;

  /* General control of CORDIC Control Register */
  MATH->CON = CORDIC_CIRCULAR_ROTATION_MODE;

  /* Z = Angle difference of I and Vref */
  MATH->CORDZ = handle_ptr->delta_vi_angle_q31;

  /* Y = zero */
  MATH->CORDY = 0;

  /* X = |Vref| of last PWM cycle. Input CORDX data, and auto start of CORDIC calculation */
  MATH->CORDX = Vref_32;

  /* wli calculation with overflow protection */
  temp_wi = (PMSM_FOC_PLL_PI.uk)*((int32_t)(handle_ptr->current_i_mag));
  if (temp_wi > 32767)
  {
    wli = ((((int32_t)(temp_wi >> 15))*handle_ptr->phase_inductance_Ls) >> handle_ptr->phase_inductance_scale);
  }
  else
  {
    wli = (((temp_wi*handle_ptr->phase_inductance_Ls) >> handle_ptr->phase_inductance_scale) >> 15);
  }

  /* Low pass filtered wLI */
  handle_ptr->bemf_2 += (int32_t)((wli - handle_ptr->bemf_2)>>handle_ptr->lpf_n_bemf);
}

/**
 * @brief	Results of CORDIC #3 - Vrefxsin(γ-θ) and Vrefxcos(γ-θ)
 *
 *@param	MATH->CORRY
 *
 *@retval	VrefxSinDelta
 */
__attribute__((section(".ram_code"))) void PMSM_FOC_PLL_VrefGetResult(PMSM_FOC_PLL_ESTIMATOR_t* const handle_ptr)
{
  while(IS_CORDIC_BUSY);
  /*  Read CORDIC result |Vref|sin(γ-θ) - 32-bit. FOC doesn't need above information of |Vref|cos(γ-θ) */
  handle_ptr->vref_x_sin_delta = MATH->CORRY;
  handle_ptr->vref_x_cos_delta = MATH->CORRX;
  
  /* Shift to get real result (16-bit).*/
  handle_ptr->vref_x_sin_delta = (int32_t)((handle_ptr->vref_x_sin_delta >> CORDIC_SHIFT) * CORDIC_CIRCULAR_MPS_BY_K_SCALED)>>CORDIC_MPS_BY_K_SCALE;
}

/**
 * @brief Estimates rotor position and speed.
 *
 *@param
 *
 *@retval	PMSM_FOC_OUTPUT.RotorSpeed
 *			PMSM_FOC_OUTPUT.RotorAngleQ31
 */
__attribute__((section(".ram_code"))) void PMSM_FOC_PLL_GetPosSpeed(PMSM_FOC_PLL_ESTIMATOR_t* const handle_ptr,PMSM_FOC_PLL_PI_t* const pi_pll_handle_ptr)
{
  int32_t tmp_Ik_Uk;

  // |Vref|sin(Î³-Î¸) with LPF.
  handle_ptr->bemf_1 += (int32_t)((handle_ptr->vref_x_sin_delta - handle_ptr->bemf_1)>> handle_ptr->lpf_n_bemf);

  /*&&&& PI Controller #4 - PLL Observer PI controller &&&&*/
  //	PI_controller(0, (FOCInput_HandlePtr->BEMF2 + FOCInput_HandlePtr->BEMF1), &PMSM_FOC_PLL_PI);

  pi_pll_handle_ptr->error = 0 - (handle_ptr->bemf_1 + handle_ptr->bemf_2);

  /* Integral output I[k] = I[k-1] + Ki * error[k] */
  tmp_Ik_Uk = ((int32_t)pi_pll_handle_ptr->ki * pi_pll_handle_ptr->error) + pi_pll_handle_ptr->ik;

  /* Check I[k] limit */
  pi_pll_handle_ptr->ik = MIN_MAX_LIMIT(tmp_Ik_Uk, pi_pll_handle_ptr->ik_limit_max, pi_pll_handle_ptr->ik_limit_min);

  /* PI output U[k] = Kp * error[k] + I[k] */
  tmp_Ik_Uk = (int32_t)((((int32_t)pi_pll_handle_ptr->kp * pi_pll_handle_ptr->error) + pi_pll_handle_ptr->ik) >> pi_pll_handle_ptr->scale_kp_ki);

  /* Check U[k] output limit */
  pi_pll_handle_ptr->uk = MIN_MAX_LIMIT(tmp_Ik_Uk, pi_pll_handle_ptr->uk_limit_max, pi_pll_handle_ptr->uk_limit_min);

  /* speed angle conversion factor in place */
  handle_ptr->rotor_angle_q31 += (int32_t)((pi_pll_handle_ptr->uk*(int32_t)handle_ptr->speed_angle_conversion_factor)>>handle_ptr->speed_angle_conversion_factor_scale);     // Estimate latest rotor angle (1Q31). φ[k] = φ[k-1] + ωr[k].

  /* PLL Observer rotor speed with LPF. */
  handle_ptr->rotor_speed += (int32_t)((pi_pll_handle_ptr->uk - handle_ptr->rotor_speed)>> handle_ptr->lpf_n_speed);

}
