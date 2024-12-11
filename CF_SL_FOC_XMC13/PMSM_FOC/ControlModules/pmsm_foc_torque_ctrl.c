/**
 * @file pmsm_foc_torque_ctrl.c
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
 * @file pmsm_foc_torque_ctrl.c
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
#include "pmsm_foc_torque_ctrl.h"
#include "../FOCLib/pmsm_foc_ip.h"

/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/
/* ADC results, trigger positions. */
extern ADCType ADC;
/* Motor control information */
extern MotorControlType Motor;
/* SVM information, such as sector 0 ~ 5 (A ~ F) in SVM space vector hexagon. */
extern SVMType SVM;
/* Speed PI controller.*/
extern PI_Coefs_Type PI_Speed;
/*Torque / Iq PI controller.*/
extern PI_Coefs_Type PI_Torque;
/* Flux /Id PI controller.*/
extern PI_Coefs_Type PI_Flux;
extern uint32_t * NEW_SHS0_CALOC1;
extern Car2PolType Car2Polar;
extern PMSM_FOC_PLL_ESTIMATOR_t PMSM_FOC_PLL_ESTIMATOR;
extern ClarkeTransformType Clarke_Transform;
extern ParkTransformType Park_Transform;
/* PLL rotor speed PI controller. */
extern PMSM_FOC_PLL_PI_t PMSM_FOC_PLL_PI;
/*  Parameters input for FOC LIB.*/
extern FOCInputType FOCInput;
/* Output for FOC LIB.*/
extern FOCOutputType FOCOutput;
/* Motor current and current space vector.*/
extern CurrentType Current;

extern uint32_t Current_I_Mag;
extern int32_t Delta_IV;
extern volatile int32_t current_i_mag_filtered;

#if(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
//#if((MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC) || (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC))
 __attribute__((section(".ram_code")))  void pmsm_foc_torque_controller (void)
{
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
 // Get_ADC_SingleShuntCurrent(&ADC);
   pmsm_foc_current_reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);

#else
   pmsm_foc_get_adcphasecurrent(FOCOutput.Previous_SVM_SectorNo, FOCOutput.New_SVM_SectorNo, &ADC);

  pmsm_foc_current_reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
#endif
  Motor.Speed = FOCOutput.Speed_by_Estimator;

  /* Motor reference speed */
  FOCInput.Ref_Speed = Motor.Ref_Speed;

  pmsm_foc_clarketransform(Current.I_U, Current.I_V, Current.I_W, &Clarke_Transform);

  pmsm_foc_parktransform(Clarke_Transform.I_Alpha_1Q31, Clarke_Transform.I_Beta_1Q31, PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31);

  /* Record previous SVM sector number */
  FOCOutput.Previous_SVM_SectorNo = FOCOutput.New_SVM_SectorNo;

  pmsm_foc_parktransform_getresult(&Park_Transform);

  #if(MOTOR_STALL_DETECTION == ENABLED)
  /* CORDIC - I_mag = sqrt(i_d^2 + i_q^2) */
  PMSM_FOC_CircularMag(Park_Transform.Id, Park_Transform.Iq,0);
  FOCOutput.current_i_mag = PMSM_FOC_CircularMag_GetResult();
  /* stall detection related modifications */
  #define LPF_IMAG_FILT_COEFF (4U) /* Low pass filter to remove high frequency current noise */
  FOCOutput.current_i_mag_filtered += (FOCOutput.current_i_mag - FOCOutput.current_i_mag_filtered) >> LPF_IMAG_FILT_COEFF;
  #if(uCPROBE_GUI == ENABLED)
  current_i_mag_filtered = FOCOutput.current_i_mag_filtered;
  #endif
  #endif

  PMSM_FOC_PLL_Imag(Car2Polar.Vref_AngleQ31, Clarke_Transform.I_Alpha_1Q31,Clarke_Transform.I_Beta_1Q31);

  /* PI Controller #2 - Torque / Iq PI controller of FOC */
//  FOCInput.Ref_Iq = 400U;
  pmsm_foc_pi_controller(FOCInput.Ref_Iq, Park_Transform.Iq, &PI_Torque);

  Car2Polar.Torque_Vq = PI_Torque.Uk;

  PMSM_FOC_PLL_ImagGetResult(&PMSM_FOC_PLL_ESTIMATOR);

  PMSM_FOC_PLL_Vref(Car2Polar.Vref32, &PMSM_FOC_PLL_ESTIMATOR);

  /* PI Controller #3 - Flux / Id PI controller of FOC */
  pmsm_foc_pi_controller(FOCInput.Ref_Id, Park_Transform.Id, &PI_Flux);

  Car2Polar.Flux_Vd = PI_Flux.Uk;

#if(DQ_DECOUPLING == ENABLED)
  int32_t wL_dq_Decoupling;

  /* Use lower resolution speed ω = PMSM_FOC_PLL_PI.Uk to prevent overflow in multiplications of ωLId and ωLIq. */
  /* Temp variable for ωL, L scaled up by (MPS/K)^2. */
  wL_dq_Decoupling = (PMSM_FOC_PLL_PI.uk) * ((uint32_t)L_OMEGALI);

  Car2Polar.Torque_Vq += ((wL_dq_Decoupling * Park_Transform.Id) >> SCALE_L);
  Car2Polar.Flux_Vd -= ((wL_dq_Decoupling * Park_Transform.Iq) >> SCALE_L);

  Car2Polar.Torque_Vq = MIN_MAX_LIMIT(Car2Polar.Torque_Vq,PI_TORQUE_UK_LIMIT_MAX, PI_TORQUE_UK_LIMIT_MIN);
  Car2Polar.Flux_Vd = MIN_MAX_LIMIT(Car2Polar.Flux_Vd,PI_FLUX_UK_LIMIT_MAX, PI_FLUX_UK_LIMIT_MIN);
#endif


  PMSM_FOC_PLL_VrefGetResult(&PMSM_FOC_PLL_ESTIMATOR);

  pmsm_foc_cart2polar(Car2Polar.Torque_Vq, Car2Polar.Flux_Vd,PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31);

  PMSM_FOC_PLL_GetPosSpeed(&PMSM_FOC_PLL_ESTIMATOR, &PMSM_FOC_PLL_PI);

  pmsm_foc_car2pol_getresult(&Car2Polar);

  uint32_t SVM_Vref16;
  SVM_Vref16 = Car2Polar.Vref32 >>CORDIC_SHIFT;

  SVM_Vref16 = (SVM_Vref16 * 311) >> 8;
  Car2Polar.Vref32 = SVM_Vref16 << CORDIC_SHIFT;

  FOCOutput.Speed_by_Estimator = PMSM_FOC_PLL_ESTIMATOR.rotor_speed;
  FOCOutput.Rotor_PositionQ31 = PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31;

}

__attribute__((section(".ram_code"))) void pmsm_foc_linear_torque_ramp_generator(int32_t current_set, int32_t inc_step, int32_t dec_step, FOCInputType* const HandlePtr)
{
    static uint32_t Iq_counter = 0;

    Iq_counter++;
    if(Iq_counter >= USER_IQ_RAMP_SLEWRATE)
    {
        Iq_counter = 0;
        if( HandlePtr->Ref_Iq < current_set)
        {
          HandlePtr->Ref_Iq += inc_step;
        }
        else if(HandlePtr->Ref_Iq > current_set)
        {
          if((HandlePtr->Ref_Iq >= dec_step )&& (ADC.ADC_DCLink < VDC_MAX_LIMIT))
          {
            HandlePtr->Ref_Iq -=  dec_step;
          }

        }
    }

    /* Limit protection for ref_iq, the max value is capped up to 1Q15*/
    if(HandlePtr->Ref_Iq > USER_IQ_REF_HIGH_LIMIT)
      HandlePtr->Ref_Iq = USER_IQ_REF_HIGH_LIMIT - 1;

}
#endif
























