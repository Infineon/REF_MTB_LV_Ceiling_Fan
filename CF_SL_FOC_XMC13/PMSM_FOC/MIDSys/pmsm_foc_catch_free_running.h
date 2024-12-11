/**
 * @file pmsm_foc_catch_free_running.h
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
#ifndef PMSM_FOC_CATCH_FREE_RUNNING_H_
#define PMSM_FOC_CATCH_FREE_RUNNING_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "../ControlModules/pmsm_foc_pi.h"
#include "../ControlModules/pmsm_foc_functions.h"
#include "pmsm_foc_CFR_lib.h"

#if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)

extern AMCLIB_CATCH_FREE_RUNNING_t CatchFreeRunning;
extern PMSM_FOC_PLL_ESTIMATOR_t PMSM_FOC_PLL_ESTIMATOR;

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
void PMSM_FOC_CFR_Variables_Init(void);

void PMSM_FOC_MSM_CFR_Func();

void PMSM_FOC_CFR_NormalStartup(void);
void PMSM_FOC_CFR_Startup(AMCLIB_CATCH_FREE_RUNNING_t * const HandlePtr);

void PMSM_FOC_MSM_MOTOR_COASTING_Func(void);
void PMSM_FOC_MSM_PRE_CHARGE_Func();


#endif // End of #if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)

#endif  // End of #ifndef PMSM_FOC_MIDSYS_PMSM_FOC_CATCH_FREE_RUNNING_H_

