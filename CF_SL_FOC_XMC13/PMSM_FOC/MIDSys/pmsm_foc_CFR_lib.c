/**
 * @file pmsm_foc_CFR_lib.c
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


#include "pmsm_foc_CFR_lib.h"
#include "..\Configuration\pmsm_foc_variables_scaling.h"

#define DIV_3_Q14                                   (5461U)                 /* ((int16_t)((1/3) * (1<<SCALE_DIV_3))) */
#define DIV_SQRT3_Q14                               (9459U)                 /* (1/√3)*2^14 */
#define CORDIC_CIRCULAR_VECTORING_MODE              (0x62)                  /* CORDIC: Circular Vectoring Mode (default). MPS: Divide by 2 (default).*/
//#define CORDIC_SHIFT                                (14)             /* 8 ~ 16. Shift for CORDIC input / output registers, whose [7:0] are 0x00. Normally no need change.*/
#define CORDIC_CIRCULAR_MPS_BY_K_SCALED             (311)                   /* CORDIC MPS/K ->(2/1.64676)* 2^CORDIC_MPS_BY_K_SCALE */
#define CORDIC_MPS_BY_K_SCALE                       (8)                     /* CORDIC MPS/K scaling factor */
#define PMSM_FOC_ANGLE_090_DEGREE_Q31               (1073741824)            /* 90° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */

static uint32_t catch_time_counter;
static uint32_t sufficient_bemf_counter;
static int32_t voltage_bemf_alpha;
static int32_t voltage_bemf_beta;
static int32_t bemf_angle_q31_previous;
static int32_t measured_rotor_speed;
static AMCLIB_CATCH_FREE_RUNNING_t * amclib_cfr_ptr;

/*  Catch free running initialization  */
AMCLIB_CFR_INIT_STATUS_t AMCLIB_CFR_Init(AMCLIB_CATCH_FREE_RUNNING_t * const cfr_ptr)
{
  AMCLIB_CFR_INIT_STATUS_t status;

  /* Reset internal variables */
  catch_time_counter = 0;
  sufficient_bemf_counter = 0;
  voltage_bemf_alpha = 0;
  voltage_bemf_beta = 0;
  bemf_angle_q31_previous = 0;
  measured_rotor_speed = 0;

  /* Assign valid function & structure pointers */
  amclib_cfr_ptr = cfr_ptr;

  /* Initialization is successful */
  status = AMCLIB_CFR_INIT_STATUS_SUCCESS;

  return status;
}

volatile int32_t bemf_voltage_ll_beta;
volatile int32_t measured_bemf_mag;
volatile int32_t bemf_voltage_ll_u;
volatile int32_t bemf_voltage_ll_v;
volatile int32_t bemf_voltage_ll_w;
//int32_t data[4][70];
/* Calculate the rotor position, speed & direction */
__attribute__((section(".ram_code"))) void AMCLIB_CFR_CalcPosSpeed (int32_t adc_res_bemf_u,int32_t adc_res_bemf_v,int32_t adc_res_bemf_w)
{
//  int32_t bemf_voltage_ll_u;
//  int32_t bemf_voltage_ll_v;
//  int32_t bemf_voltage_ll_w;

  int32_t bemf_voltage_ll_alpha;
//  int32_t bemf_voltage_ll_beta;

  if (catch_time_counter <= amclib_cfr_ptr->cfr_total_catch_time)
  {
    /* Calculate BEMF line to line */
    bemf_voltage_ll_u = (int32_t)(adc_res_bemf_u << 1U) - adc_res_bemf_v - adc_res_bemf_w;
    bemf_voltage_ll_v = (int32_t)(adc_res_bemf_v << 1U) - adc_res_bemf_u - adc_res_bemf_w;
    bemf_voltage_ll_w = (int32_t)(adc_res_bemf_w << 1U) - adc_res_bemf_v - adc_res_bemf_u;

    /* Clarke Transform */
    /* BEMF_Alpha = (2 * BEMF_U - (BEMF_V + BEMF_W))/3 */
    bemf_voltage_ll_alpha = (int32_t)(((bemf_voltage_ll_u << 1) - (bemf_voltage_ll_v + bemf_voltage_ll_w)) * (DIV_3_Q14 << (CORDIC_SHIFT - 14)));
    /* BEMF_Beta = (BEMF_V - BEMF_W)/√3 in 1Q31 */
    bemf_voltage_ll_beta = (int32_t)((bemf_voltage_ll_v - bemf_voltage_ll_w) * (DIV_SQRT3_Q14 << (CORDIC_SHIFT - 14U)));

    /*%%%% CORDIC #0 - Cartesian to Polar %%%%
     *###* Execution time: ?us (O3 - Optimize most), using CORDIC Coprocessor.
     * |V_BEMF| = K/MPS * sqrt(BEMFα^2+BEMFβ^2)   * Xfinal = K/MPS * sqrt(X^2+Y^2), where K = 1.646760258121.
     * θ = atan(BEMFβ/BEMFα)            * Zfinal = Z + atan(Y/X)          (Yfinal = 0).
     * -------------------------------------------------------------------------------------------------------*/
    MATH->CON = CORDIC_CIRCULAR_VECTORING_MODE;               // General control of CORDIC Control Register.
    MATH->CORDZ = 0;                    // Z = zero.
    MATH->CORDY = bemf_voltage_ll_beta;
    MATH->CORDX = bemf_voltage_ll_alpha; // Input CORDX data, and auto start of CORDIC calculation.

    /*%%%% Results of CORDIC #0 - Cartesian to Polar %%%%*/
    while (MATH->STATC & 0x01);             // Wait if CORDIC is still running calculation.

    amclib_cfr_ptr->measured_bemf_angle_q31 = MATH->CORRZ;          // BEMF angle θ.
    amclib_cfr_ptr->measured_bemf_mag = MATH->CORRX;             // Read CORDIC result - Magnitude. X- Unsigned.
    amclib_cfr_ptr->measured_bemf_mag  = (uint32_t)(amclib_cfr_ptr->measured_bemf_mag >> CORDIC_SHIFT);
    /*BEMF scaling - compensate clarke transform and (MPS/K) CORDIC scaling factor */

     amclib_cfr_ptr->measured_bemf_mag = (uint16_t)((amclib_cfr_ptr->measured_bemf_mag * CORDIC_CIRCULAR_MPS_BY_K_SCALED) >> CORDIC_MPS_BY_K_SCALE); // x MPS/K.

    /* For the first time to catch free-running, don't (cannot) calculate rotor speed.
     * Calculate only if sufficient BEMF is available */
    if((amclib_cfr_ptr->measured_bemf_mag > amclib_cfr_ptr->cfr_bemf_thrshold) && (catch_time_counter > 0U))
    {
      sufficient_bemf_counter ++;
      //measured_rotor_speed =  (int32_t)((amclib_cfr_ptr->measured_bemf_angle_q31 - bemf_angle_q31_previous) >> (16U - amclib_cfr_ptr->rotor_speed_res_inc));
      measured_rotor_speed =  (int32_t)(((amclib_cfr_ptr->measured_bemf_angle_q31 - bemf_angle_q31_previous)*ANGLE_TO_SPEED_CONV_FACTOR) >> ANGLE_TO_SPEED_CONV_FACTOR_SCALE);

      /* Low pass filtered speed output */
      amclib_cfr_ptr->estimated_rotor_speed_filtered += (int32_t)((measured_rotor_speed - amclib_cfr_ptr->estimated_rotor_speed_filtered) >> amclib_cfr_ptr->cfr_speed_LPF);
    }

#if(0)
    amclib_cfr_ptr->estimated_rotor_angle_q31 = amclib_cfr_ptr->measured_bemf_angle_q31 - PMSM_FOC_ANGLE_090_DEGREE_Q31 + (measured_rotor_speed << (16U - amclib_cfr_ptr->rotor_speed_res_inc));
#endif

    bemf_angle_q31_previous = amclib_cfr_ptr->measured_bemf_angle_q31;
//    data[0][catch_time_counter] = adc_res_bemf_u;
//    data[1][catch_time_counter] = bemf_voltage_ll_u;
//    data[2][catch_time_counter] = bemf_angle_q31_previous;
//    data[3][catch_time_counter] = amclib_cfr_ptr->estimated_rotor_speed_filtered;
  }
  else
  {
    /* Check the Motor direction based upon measured speed */
    if (amclib_cfr_ptr->estimated_rotor_speed_filtered > 0)
    {
      /* Motor rotating in +ve direction: for TTi: drive-out direction */
      amclib_cfr_ptr->motor_identified_dir = AMCLIB_MOTOR_DIR_POSITIVE;
      //amclib_cfr_ptr->estimated_rotor_angle_q31 = amclib_cfr_ptr->measured_bemf_angle_q31 - PMSM_FOC_ANGLE_090_DEGREE_Q31 + (measured_rotor_speed << (16U - amclib_cfr_ptr->rotor_speed_res_inc));
      amclib_cfr_ptr->estimated_rotor_angle_q31 = amclib_cfr_ptr->measured_bemf_angle_q31 - PMSM_FOC_ANGLE_090_DEGREE_Q31 + (int32_t)((measured_rotor_speed*SPEED_TO_ANGLE_CONV_FACTOR) >> SPEED_TO_ANGLE_CONV_FACTOR_SCALE);
    }
    else
    {
      /* Motor rotating in -ve direction: for TTi: drive-in direction */
      amclib_cfr_ptr->motor_identified_dir = AMCLIB_MOTOR_DIR_NEGATIVE;
      //amclib_cfr_ptr->estimated_rotor_angle_q31 = amclib_cfr_ptr->measured_bemf_angle_q31 + PMSM_FOC_ANGLE_090_DEGREE_Q31 + (measured_rotor_speed << (16U - amclib_cfr_ptr->rotor_speed_res_inc));
      amclib_cfr_ptr->estimated_rotor_angle_q31 = amclib_cfr_ptr->measured_bemf_angle_q31 - PMSM_FOC_ANGLE_090_DEGREE_Q31 + (int32_t)((measured_rotor_speed*SPEED_TO_ANGLE_CONV_FACTOR) >> SPEED_TO_ANGLE_CONV_FACTOR_SCALE);
    }

    /* Calculate the rotor angle from BEMF */
    /* If more than half the times say can catch a free-running Motor
     * and BEMF is sufficient now as well then switch to catch free transition */
    if((sufficient_bemf_counter > (amclib_cfr_ptr->cfr_total_catch_time >> 1U)) && (amclib_cfr_ptr->measured_bemf_mag > amclib_cfr_ptr->cfr_bemf_thrshold))
    {
      amclib_cfr_ptr->cfr_startup_flag = AMCLIB_CFR_STARTUP_CFR;
    }
    else
    {
      /* Go to normal Motor startup, bootstrap / brake Motor -> Direct FOC. */
      amclib_cfr_ptr->cfr_startup_flag = AMCLIB_CFR_STARTUP_NORMAL;

    }

    /* Scale BEMF as per the voltage scaling of FOC: voltage divider, 1/3, Q12 to Q15 */
    amclib_cfr_ptr->measured_bemf_mag  = (uint16_t)((amclib_cfr_ptr->bemf_v_scaling_factor * amclib_cfr_ptr->measured_bemf_mag) >> amclib_cfr_ptr->bemf_v_scaling_factor_scale);

    /* reset flag & counter for next iteration */
    catch_time_counter = 0;
    sufficient_bemf_counter = 0;

    amclib_cfr_ptr->cfr_status = AMCLIB_CFR_STATUS_COMPLETED;
  }

  catch_time_counter ++;
}

