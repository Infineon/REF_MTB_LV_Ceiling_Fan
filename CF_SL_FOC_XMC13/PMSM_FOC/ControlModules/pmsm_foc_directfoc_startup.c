/**
 * @file pmsm_foc_directfoc_startup.c
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
 * @file pmsm_foc_directfoc_startup.c
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
#include "pmsm_foc_directfoc_startup.h"

#if (USER_ROTOR_IPD_METHOD == ROTOR_IPD_PRE_ALIGNMENT)
/*********************************************************************************************************************
 * MACROS
 *********************************************************************************************************************/
#if(UART_INTERFACE == DISABLED)
#define VREF_INC_STEP   (1U)                        /* Step that voltage increases. Default-4*/
#endif/*#if(UART_INTERFACE == DISABLED)*/

#define PRE_POSIT_VOLT ((VQ_VF_OFFSET * 2U) >> 1)   /* Voltage for rotor preposition/alignment. */
#define PI_SPEED_IK_DEFAULT (1U << 11)
/*********************************************************************************************************************
 * GLOBAL DATA
**********************************************************************************************************************/
extern ADCType ADC;                             /* ADC results, trigger positions. */
extern SVMType SVM;                             /* SVM information, such as sector 0 ~ 5 (A ~ F) in SVM space vector hexagon. */
extern CurrentType Current;                     /* Motor current and current space vector. */
extern MotorControlType Motor;                  /* Motor control information */
extern FOCOutputType FOCOutput;                 /* Output for FOC LIB. */
extern ClarkeTransformType Clarke_Transform;
extern Car2PolType Car2Polar;

extern PI_Coefs_Type PI_Speed;                  /* Speed PI controller. */
extern PI_Coefs_Type PI_Torque;                 /* Torque / Iq PI controller. */
extern PI_Coefs_Type PI_Flux;                   /* Flux /Id PI controller. */
extern PMSM_FOC_PLL_PI_t PMSM_FOC_PLL_PI;                    /* PLL rotor speed PI controller. */


/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/* API to set the Rotor initial preposition/alignment */

void pmsm_foc_directfocrotor_pre_positioning(void)
{
  uint32_t Ik_unsigned;

  /* Single-shunt or 2or3-shunt 3-phase current reconstruction, to get Iu and Iv. */
  #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  pmsm_foc_current_reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);
  #else
  pmsm_foc_current_reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
  #endif
  pmsm_foc_clarketransform(Current.I_U, Current.I_V, Current.I_W, &Clarke_Transform);

  /* Rotor preposition/alignment counter ++. */
  Motor.Alignment_Counter++;
  if (Motor.Alignment_Counter < ALIGNMENT_TIME)
  {
    if (Car2Polar.SVM_Vref16 < PRE_POSIT_VOLT)
    {
      /* Vref increases gradually. */
      Car2Polar.SVM_Vref16 += VREF_INC_STEP;
    }
  }
  else
  {
    /* Next, directly to FOC closed-loop for faster startup. */
//    XMC_GPIO_SetOutputLow(TEST_PIN);
    Motor.State = (uint32_t) FOC_CLOSED_LOOP;
    /* Motor in transition mode. */
    //Motor.Status = MOTOR_TRANSITION;
    Motor.Status = MOTOR_STABLE;

    /* Clear counter. */
    Motor.Alignment_Counter = 0U;
    /* Init FOC rotor speed ωr = PMSM_FOC_PLL_PI.Uk, needed for ωL|I|, ωLId, ωLIq,
     *  * and FG frequency calculation. */
    PMSM_FOC_PLL_PI.uk = (int32_t) Motor.Speed;
    /* Motor reference speed of FOC. */
//    Motor.Ref_Speed = (int32_t) Motor.Speed;      //comment off testing
    Ik_unsigned = (uint32_t) Car2Polar.SVM_Vref16 << PI_TORQUE_SCALE_KPKI;
    PI_Torque.Ik = (int32_t) Ik_unsigned;
    /* Init Vd's Ik = 0 of flux / Id PI controller. */
    PI_Flux.Ik = 0;
    #if(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC)
    /* no V/f or MET. */
    Motor.Ramp_Up_Rate = (RAMPUP_RATE << 2U);
    /* Slower ramp up for S-curve profile of FOC. */
    PI_Speed.Ik = PI_SPEED_IK_DEFAULT << PI_SPEED_SCALE_KPKI;
    #endif
  }

  /* Update SVM PWM. */
  pmsm_foc_svpwm_update(Car2Polar.SVM_Vref16, Car2Polar.SVM_Angle16);
  /* Record SVM reference vector magnitude (32-bit) of last PWM cycle. */
  Car2Polar.Vref32_Previous = Car2Polar.Vref32;
  Car2Polar.Vref32 = (uint32_t) Car2Polar.SVM_Vref16 << CORDIC_SHIFT;

  /* Init for smooth transition from V/f to FOC closed-loop. */
  pmsm_foc_init_smooth_transition_to_foc(Motor.Speed);

  if (Motor.State == (uint32_t) FOC_CLOSED_LOOP)
  {
    /* Init parameters of FOC LIB. Init once only. */
    pmsm_foc_systemparameters_init_onceonly();
  }

}
#endif
/* End of pmsm_foc_directfocrotor_pre_positioning () */

