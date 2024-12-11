/**
 * @file pmsm_foc_transitions.c
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
 * @file pmsm_foc_transitions.c
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
#include "pmsm_foc_transitions.h"
#include "../FOCLib/pmsm_foc_ip.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/* Global variables:*/
extern ClarkeTransformType Clarke_Transform;
extern Car2PolType Car2Polar;
extern PMSM_FOC_PLL_ESTIMATOR_t PMSM_FOC_PLL_ESTIMATOR;
/* Angle ? (1Q23 << 8) of current space vector, from last PWM cycle */
extern int32_t I_AngleQ31;

/* ADC results, trigger positions. */
extern ADCType ADC;
/* Motor control information */
extern MotorControlType Motor;
/* Motor current and current space vector. */
extern CurrentType Current;

/* Parameters input for FOC LIB. */
extern FOCInputType FOCInput;
/* PLL rotor speed PI controller. */
extern PMSM_FOC_PLL_PI_t PMSM_FOC_PLL_PI;

/* Indicates a running CORDIC calculation if MATH->STATC[0] (i.e.: BSY) = 1. */
#define CORDIC_IS_BSY (MATH->STATC & 0x01)

/** To calculate |Vref|sin(?-?), wL|I|, and e=|Vref|sin(?-?)+wL|I|, for MET (Maximum Efficiency Tracking) .
** Execution time: 5.9us - 6.3us (O3 - Optimize most).
* ----------------------------------------------------------------------------------------------------------*/
void pmsm_foc_init_smooth_transition_to_foc (uint32_t Omega_Speed)
{

    #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
    // Get_ADC_SingleShuntCurrent(&ADC);
    pmsm_foc_current_reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);

    #else
    pmsm_foc_get_adcphasecurrent(SVM.PreviousSectorNo, SVM.CurrentSectorNo, &ADC);

    pmsm_foc_current_reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
    #endif

    pmsm_foc_clarketransform(Current.I_U, Current.I_V, Current.I_W, &Clarke_Transform);

    PMSM_FOC_PLL_Imag(Car2Polar.Vref_AngleQ31,Clarke_Transform.I_Alpha_1Q31,Clarke_Transform.I_Beta_1Q31);

    /* Wait if CORDIC is still running calculation. Omit if CCU4 outputs debug information.*/
    while (CORDIC_IS_BSY)
    {
      continue;
    }
    PMSM_FOC_PLL_ImagGetResult(&PMSM_FOC_PLL_ESTIMATOR);

    PMSM_FOC_PLL_Vref(Car2Polar.Vref32, &PMSM_FOC_PLL_ESTIMATOR);

    /* Results of CORDIC #7 - Vrefxsin(γ-θ) and Vrefxcos(?-?) */
    /* Wait if CORDIC is still running calculation.*/
    while (CORDIC_IS_BSY)
    {
      continue;
    }
    PMSM_FOC_PLL_VrefGetResult(&PMSM_FOC_PLL_ESTIMATOR);

}

#define MET_VREF_STEP     (1U)

/* ~0.5s, max time that MET control takes before considered stable, x PWM period. */
#define TIME_OF_MET     (7500U)

/** Miscellaneous works in MET, such as ramp up, speed adjustment, transition from MET to FOC, etc
* ---------------------------------------------------------------------------------------------------*/
void pmsm_foc_misc_works_of_met (void)
{
    if (Motor.Status != MOTOR_TRANSITION)
    {         // Motor in transition mode.
      pmsm_foc_transition_foc();                // Transition from MET to FOC, using 3-step motor start-up V/f->MET->FOC.
    }

    if ((SYSTEM_BE_IDLE) && (Motor.Speed <= Motor.Speed_by_POT_PWM))
    {
      /* If PWM duty cycle or POT ADC too low, and speed reached PWM set speed */
      Motor.Counter = 0;
      Motor.Ramp_Counter = 0;
      /* Set low motor speed, so motor stop and resume can be fast.*/
      Motor.Speed = SPEED_LOW_LIMIT >> 5;
       /* Next, go to Motor Stop. */
      Motor.State = STOP_MOTOR;
  }

} /* End of pmsm_foc_misc_works_of_met () */

/*
 * Init variables for transition to FOC, e.g. from MET to FOC: MET -> FOC
 * Execution time: ?us (O3 - Optimize most)
 * 3-step motor start-up: V/f open-loop -> MET closed-loop -> FOC closed-loop.
 */
void pmsm_foc_transition_foc(void)
{
  Motor.Counter = TIME_OF_MET + 1; /* Go to FOC immediately once it find |ε| <= ε_Th. */

  if (Motor.Counter > TIME_OF_MET)
  {
    Motor.State = FOC_CLOSED_LOOP; /* Next, go to FOC closed-loop. */

    pmsm_foc_init_foc_rotorangle(); /* Init rotor angle for first FOC PWM cycle, Lag/lead current angle γ by a 90° angle. */
    pmsm_foc_init_foc_pi_iks(); /* To init PI controllers' Ik for first FOC PWM cycle. */

    PMSM_FOC_PLL_PI.uk = Motor.Speed; /*
                              * Init FOC rotor speed ωr = PMSM_FOC_PLL_PI.Uk, needed for ωL|I|, ωLId, ωLIq,
                              * and FG frequency calculation.
                              */
    Motor.Ref_Speed = Motor.Speed; /* Motor reference speed of FOC. */

    pmsm_foc_systemparameters_init_onceonly(); /* Init parameters of FOC LIB. Init once only. */

    Motor.Status = MOTOR_TRANSITION; /* Motor in transition mode. */
    Motor.Counter = 0; /* Clear counters. */
    Motor.Ramp_Counter = 0;
    Motor.Ramp_Up_Rate = RAMPUP_RATE << USER_RATIO_S; /* Slower ramp up and ramp down for S-curve profile. */
  }
} /* End of pmsm_foc_transition_foc () */

/* API to Shift times for unity gain LPF: Y[n] = Y[n-1] + (X[n]-Y[n-1]) */
__attribute__((section(".ram_code")))  int32_t pmsm_foc_unity_gain_lpf(int32_t filter_input, uint8_t gain)
{
  static int32_t temp_output;

  temp_output = (temp_output * ((1 << gain)-1) + filter_input) >> gain;

  return temp_output;
}






