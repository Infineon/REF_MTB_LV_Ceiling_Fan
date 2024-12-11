/**
 * @file pmsm_foc_stall_detection.h
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
#ifndef PMSM_FOC_STALL_DETECTION_H_
#define PMSM_FOC_STALL_DETECTION_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <xmc_common.h>
#include "../PMSM_FOC/Configuration/pmsm_foc_user_config.h"
#include "../ControlModules/pmsm_foc_functions.h"
#include "../Interrupts/pmsm_foc_statemachine.h"
#include "../Interrupts/pmsm_foc_error_handling.h"
#include "../Configuration/pmsm_foc_variables_scaling.h"

/**
 *  @brief Motor STALL detection
 */
typedef struct AMCLIB_STALL_DETECT
{
  uint32_t phase_resistance;                   /*!< Motor phase resistance scaled value */
  int32_t phase_inductance_Lq;                /*!< Motor inductance Lq scaled up value  */
  int32_t stall_current;                      /*!< user configured stall current */
  uint32_t stall_coefficient;                   /*!< user configured stall coefficient in scaled format */
  uint32_t threshold_stall_time;              /*!< stall condition continue to see above this user configured time then stall detected */
  uint16_t motor_stall_status;                /*!< motor STALL condition detection status */
  uint16_t phase_inductance_scale;             /*!< motor inductance scale  */
  uint16_t phase_resistance_scale;             /*!< Motor phase resistance scale */
  uint16_t stall_ratio_scale;                  /*!< Stall ratio scale */
  uint8_t LPF_stall;                          /*!< Low pass filter coefficient used in STALL detection */

}PMSM_FOC_STALL_DETECT_t;




/**  @brief Stall detection structure */
typedef struct
{
  int32_t Coeff_N_iq; // Coefficiency for the filter
  int32_t iq_flted;   // The filtered value of i_mag
  int32_t Stall_torque_Q15; // The stall torque in Q15
  int32_t Stall_iq_threshold_Q15; // The threshold for the stall protection
  uint32_t Stall_cnt;     // The counter for stall fault judgement
  uint32_t Blanking_cnt;  // Blanking at startup
  uint32_t Stall_cnt_threshold; // The threshold for the stall counter
  uint16_t Stall_detected;  // The flag for stall fault detected
  uint16_t Stall_detection_enabled; // The flag of enabling stall detection
}PMSM_FOC_STALL_DETECTION_t;


extern PMSM_FOC_STALL_DETECTION_t PMSM_FOC_STALL_DETECTION;
extern MotorControlType Motor;                  /* Motor control information */

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
#if(IP_STALL)
__attribute__((section(".ram_code"))) void PMSM_FOC_StallDetection(int32_t torque_iq, int32_t torque_vq, int32_t current_mag);
void PMSM_FOC_STALL_InitVariables (PMSM_FOC_STALL_DETECT_t * const stall_ptr);
#else
void PMSM_FOC_Stall_Detection_Init(void);
__attribute__((section(".ram_code"))) void PMSM_FOC_Stall_Detection_Init(void);
__attribute__((section(".ram_code"))) void PMSM_FOC_Stall_Detection(void);
#endif

#endif
