/**
 * @file pmsm_foc_CFR_lib.h
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
#ifndef PMSM_FOC_CFR_LIB_H_
#define PMSM_FOC_CFR_LIB_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <xmc_common.h>

/***********************************************************************************************************************
 * ENUMS
 **********************************************************************************************************************/
typedef enum AMCLIB_MOTOR_DIR
{
  AMCLIB_MOTOR_DIR_POSITIVE   = 1,                 /*!< Motor direction - forward */
  AMCLIB_MOTOR_DIR_NEGATIVE   = -1                 /*!< Motor direction - reverse */

}AMCLIB_MOTOR_DIR_t;

typedef enum AMCLIB_CFR_STARTUP
{
  AMCLIB_CFR_STARTUP_CFR,                         /*!< Catchfree startup */
  AMCLIB_CFR_STARTUP_NORMAL,                       /*!< Normal startup from zero speed */

}AMCLIB_CFR_STARTUP_t;

typedef enum AMCLIB_CFR_STATUS
{
  AMCLIB_CFR_STATUS_IN_PROGRESS,                  /*!< CFR is in progress */
  AMCLIB_CFR_STATUS_COMPLETED                     /*!< CFR detection is completed */

}AMCLIB_CFR_STATUS_t;

typedef enum AMCLIB_CFR_INIT_STATUS
{
  AMCLIB_CFR_INIT_STATUS_FAIL     = -1,                /*!< Initialization failed */
  AMCLIB_CFR_INIT_STATUS_SUCCESS  =  1                 /*!< Initialization successful */

}AMCLIB_CFR_INIT_STATUS_t;
/***********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/
/* Catch free-running configuration & output structure */
typedef struct AMCLIB_CATCH_FREE_RUNNING
{
  AMCLIB_MOTOR_DIR_t 	motor_identified_dir;            /*!< Rotation direction of motor (Positive/Forward - (+1), or Negative/Backward - (-1)) */
  AMCLIB_CFR_STARTUP_t 	cfr_startup_flag;                /*!< Flag to indicate CFR startup or normal startup */
  AMCLIB_CFR_STATUS_t 	cfr_status;                      /*!< CFR completion status */

  uint32_t measured_bemf_mag;                            /*!< Measured BEMF amplitude */
  int32_t measured_bemf_angle_q31;                       /*!< Measured BEMF angle  */

  uint32_t bemf_v_scaling_factor;                        /*!< BEMF adc count to FOC voltage scaling conversion factor * 2^scale */

  int32_t estimated_rotor_angle_q31;                     /*!< estimated rotor angle from from BEMF */
  int32_t estimated_rotor_speed_filtered;               /*!< estimated rotor speed(filtered) from from BEMF */

  uint32_t cfr_bemf_thrshold;                            /*!< BEMF threshold above this CFR start or normal start */
  uint32_t cfr_total_catch_time;                         /*!< CFR algo will be allowed to determine the speed & position for this duration(PWM cycles) */

  uint16_t cfr_speed_LPF;                                /*!< Low pass filter coefficient used for speed filtering */

  uint8_t rotor_speed_res_inc;                           /*!< rotor speed resolution increment if used for FOC,Range:[0-7] */
  uint8_t init_flag;                                     /*!< CFR related peripheral initialization flag */
  uint8_t bemf_v_scaling_factor_scale;                   /*!< bemf_v_scaling_factor scale information */

}AMCLIB_CATCH_FREE_RUNNING_t;

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
/***********************************************************************************************************************
 * API Prototypes
 **********************************************************************************************************************/
AMCLIB_CFR_INIT_STATUS_t AMCLIB_CFR_Init(AMCLIB_CATCH_FREE_RUNNING_t * const cfr_ptr);
__attribute__((section(".ram_code"))) void AMCLIB_CFR_CalcPosSpeed (int32_t adc_res_bemf_u,int32_t adc_res_bemf_v,int32_t adc_res_bemf_w);

#endif  // End of #ifndef PMSM_FOC_MIDSYS_PMSM_FOC_CATCH_FREE_RUNNING_H_

