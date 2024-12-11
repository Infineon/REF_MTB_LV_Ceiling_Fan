/**
 * @file pmsm_foc_functions.c
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
 * @file pmsm_foc_functions.c
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

#include <XMC1300.h>                      /* SFR declarations of the selected device */
#include "pmsm_foc_functions.h"
#include "..\MIDSys\pmsm_foc_stall_detection.h"
#include "../FOCLib/pmsm_foc_ip.h"

//#include "pmsm_foc_interface.c"   /*For T2S*/

ClarkeTransformType Clarke_Transform;
ParkTransformType Park_Transform;
//PMSM_FOC_PLL_PI_t PMSM_FOC_PLL_PI; /* PLL Observer PI Controller */
uint16_t speed_ctrl_first_time_flag = 0;

Car2PolType Car2Polar;
/* ADC results, trigger positions. */
extern ADCType ADC;
/* Motor current and current space vector. */
CurrentType Current;
/* Motor control information */
extern MotorControlType Motor;

extern PMSM_FOC_PLL_ESTIMATOR_t PMSM_FOC_PLL_ESTIMATOR;

/* For trip / over-current detection, and protection. */
TripType Trip;
/* For motor startup lock / fail / stall detection, and protection. */
StallType Stall;
/* For over/under-voltage detection, and protection. */
OverUnderVoltType OverUnderVolt;
/* Angle γ (1Q23 << 8) of current space vector, from last PWM cycle */
int32_t I_AngleQ31;


HallType Hall;
/* Parameters input for FOC LIB. */
extern FOCInputType FOCInput;
/* Sine LUT used for SVM calculations, array size 256 or 1024.*/
extern const uint16_t Sinus60_tab[];
/* Speed PI controller.*/
extern PI_Coefs_Type PI_Speed;
/*  Torque / Iq PI controller.*/
extern PI_Coefs_Type PI_Torque;
/*  Flux /Id PI controller.*/
extern PI_Coefs_Type PI_Flux;
/* PLL rotor speed PI controller.*/
extern PMSM_FOC_PLL_PI_t PMSM_FOC_PLL_PI;
extern PMSM_FOC_PLL_ESTIMATOR_t PMSM_FOC_PLL_ESTIMATOR;
extern int32_t VrefxSinDelta;
/* |I|, magnitude of current space vector */
extern uint32_t Current_I_Mag;
extern int32_t Delta_IV;
volatile int32_t current_i_mag_filtered;

extern uint32_t Real_Speed_in_rpm;   /*using extern for global.*/

#define SHIFT_TO_1Q15		(3U)
#define MET_VREF_STEP		(1U)

/**
  * @brief	3-Shunt 3-Phase Current Reconstruction, ADC values are from last PWM cycle
  * 		ADCs of 2or3-Shunt are triggered by CCU83 CR1S
  *
  * @param	VADC_G1_RES_0
  * 		VADC_G0_RES_0
  * 		VADC_G1_RES_1
  *
  *@retval 	Current.I_U
  * 		Current.I_V
  * 		Current.I_W
  */

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
__attribute__((section(".ram_code"))) void pmsm_foc_current_reconstruction (uint16_t Previous_SVM_SectorNo,int32_t ADC_result1, int32_t ADC_result2, CurrentType * const HandlePtr)
{
   switch (Previous_SVM_SectorNo)
  {
    case 0:  /* Sector A */
      HandlePtr->I_U = ADC_result1;
      HandlePtr->I_V = ADC_result2 - ADC_result1;
      break;

    case 1:  /* Sector B */
      HandlePtr->I_U = ADC_result2 - ADC_result1;
      HandlePtr->I_V = ADC_result1;
      break;

    case 2:  /* Sector C */
      HandlePtr->I_U = -ADC_result2;
      HandlePtr->I_V = ADC_result1;
      break;

    case 3:  /* Sector D */
      HandlePtr->I_U = -ADC_result2;
      HandlePtr->I_V = ADC_result2 - ADC_result1;
      break;

    case 4:  /* Sector E */
      HandlePtr->I_U = ADC_result2 - ADC_result1;
      HandlePtr->I_V = -ADC_result2;
      break;

    default:  /* Sector F */
      HandlePtr->I_U = ADC_result1;
      HandlePtr->I_V = -ADC_result2;
      break;
  }

   HandlePtr->I_W = -HandlePtr->I_U - HandlePtr->I_V; /*Calculating I_W (I_W + I_V + I_U = 0)*/


   pmsm_foc_ccu4_debug3output (HandlePtr->I_U, 1,13,HandlePtr->I_V ,1,13);
}
#else
__attribute__((section(".ram_code"))) void pmsm_foc_current_reconstruction (int32_t ADC_Iu, int32_t ADC_Iv, int32_t ADC_Iw, CurrentType * const HandlePtr)
{

	/* Motor phase current, Iu, Iv, Iw*/
	HandlePtr->I_U = ((int16_t)(ADC.ADC_Bias_Iu - ADC_Iu)) << 4;
	HandlePtr->I_V = ((int16_t)(ADC.ADC_Bias_Iv - ADC_Iv)) << 4;
	HandlePtr->I_W = ((int16_t)(ADC.ADC_Bias_Iw - ADC_Iw)) << 4;


}
#endif

	/*###* FOC controller LIB, calculated once in each PWM cycle ####
		 * ------------------------------------------------------------*/
__attribute__((section(".ram_code"))) void pmsm_foc_speed_controller (void)
{
  #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  pmsm_foc_current_reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);
  #else
  pmsm_foc_get_adcphasecurrent(FOCOutput.Previous_SVM_SectorNo, FOCOutput.New_SVM_SectorNo, &ADC);
	pmsm_foc_current_reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
  #endif
	Motor.Speed = FOCOutput.Speed_by_Estimator;

  #if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC) ||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC))
  #if(OVERCURRENT_PROTECTION == ENABLED)
	pmsm_foc_over_current_protection_check(ADC.ADC_IDCLink, Park_Transform.Iq, &FOCInput.overcurrent_factor);
  FOCInput.Ref_Speed = (Motor.Ref_Speed * FOCInput.overcurrent_factor) >> 12;        // Motor reference speed.
  #else
  FOCInput.Ref_Speed = Motor.Ref_Speed;        // Motor reference speed.
  #endif
  #else
  FOCInput.Ref_Speed = Motor.Ref_Speed;        // Motor reference speed.
  #endif
  pmsm_foc_clarketransform(Current.I_U, Current.I_V, Current.I_W, &Clarke_Transform);

  pmsm_foc_parktransform(Clarke_Transform.I_Alpha_1Q31, Clarke_Transform.I_Beta_1Q31, PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31);

  FOCOutput.Previous_SVM_SectorNo = FOCOutput.New_SVM_SectorNo;	// Record previous SVM sector number.

#if(MAX_POWER_LIMIT_EN_DIS == ENABLED)
  /*Power Limit Iq calculation*/
  pmsm_foc_divide(POWER_RAW_SCALE_UP, PMSM_FOC_PLL_ESTIMATOR.rotor_speed); /* Iq_power_max_int = Power_raw_scale_up / rotor_speed  -- 104320/ */
  pmsm_foc_divide_getresult(&Motor.power_limit_iq);

  /*Power limit check after having certain speed. Rotor speed zero result in undefined/data type max value for power limit iq.*/
  if(Real_Speed_in_rpm <= Advanced_stop_exit_speed)
  {
	  Motor.power_limit_iq = LIMIT_MAX_IS; /*Default 32767*/
  }

  if(Motor.power_limit_iq >= LIMIT_MAX_IS)
  {
	  Motor.power_limit_iq = LIMIT_MAX_IS; /*Default 32767*/
  }

  /*Power Limit Ramp down - Speed Control*/
  if((FOCOutput.current_i_mag_filtered > Motor.power_limit_iq) && (FOCOutput.current_i_mag_filtered > 0)) /*I_mag might go negative*/
  {
	  FOCInput.Ref_Speed = FOCInput.Ref_Speed - RAMPDOWN_RATE;
	  if(FOCInput.Ref_Speed <= 0)
	  {
		  FOCInput.Ref_Speed = 0;
	  }
	  Motor.power_limit_flag = 1;
//	  XMC_GPIO_SetOutputHigh(TEST_PIN);
	  XMC_GPIO_PORT0->OMR = (uint32_t)0x1U << 13;
  }
  else
  {
	  Motor.power_limit_flag = 0;
//	  XMC_GPIO_SetOutputLow(TEST_PIN);
	  XMC_GPIO_PORT0->OMR = 0x10000U << 13;
  }
#endif


  #if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC) ||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC))
	/* PI Controller #1 - Speed / Speed PI controller of FOC */
	pmsm_foc_pi_controller_anti_windup(FOCInput.Ref_Speed,PMSM_FOC_PLL_ESTIMATOR.rotor_speed, &PI_Speed);
	#endif /*#if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC) ||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC))*/

	pmsm_foc_parktransform_getresult(&Park_Transform);

  #if(MOTOR_STALL_DETECTION == ENABLED)
  /* CORDIC - I_mag = sqrt(i_d^2 + i_q^2) */
  PMSM_FOC_CircularMag(Park_Transform.Id, Park_Transform.Iq,0);
  FOCOutput.I_q = Park_Transform.Iq;
  FOCOutput.current_i_mag = PMSM_FOC_CircularMag_GetResult();
  /* stall detection related modifications */
  #define LPF_IMAG_FILT_COEFF (10U) /* Low pass filter to remove high frequency current noise */
  FOCOutput.current_i_mag_filtered += (FOCOutput.current_i_mag - FOCOutput.current_i_mag_filtered) >> LPF_IMAG_FILT_COEFF;
  #if(uCPROBE_GUI_no_UART == ENABLED)
  current_i_mag_filtered = FOCOutput.current_i_mag_filtered;
  #endif
  #endif

  PMSM_FOC_PLL_Imag(Car2Polar.Vref_AngleQ31, Clarke_Transform.I_Alpha_1Q31,Clarke_Transform.I_Beta_1Q31);
  #if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC) ||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC))
  FOCInput.Ref_Iq = PI_Speed.Uk;
	/* PI Controller #2 - Torque / Iq PI controller of FOC */
	pmsm_foc_pi_controller(FOCInput.Ref_Iq, Park_Transform.Iq, &PI_Torque);
  #elif(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
  /* PI Controller #2 - Torque / Iq PI controller of FOC */
	pmsm_foc_pi_controller(FOCInput.Ref_Iq, Park_Transform.Iq, &PI_Torque);
  #endif /*#if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC) ||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC))*/

	if(PI_Torque.Uk < FIRST_KICK_VALUE && speed_ctrl_first_time_flag == 0)
	  Car2Polar.Torque_Vq = FIRST_KICK_VALUE;
	else
	{
	  Car2Polar.Torque_Vq = PI_Torque.Uk;
	  speed_ctrl_first_time_flag = 1;
	}

	PMSM_FOC_PLL_ImagGetResult(&PMSM_FOC_PLL_ESTIMATOR);
	PMSM_FOC_PLL_Vref(Car2Polar.Vref32, &PMSM_FOC_PLL_ESTIMATOR);
	/* PI Controller #3 - Flux / Id PI controller of FOC */
	pmsm_foc_pi_controller(FOCInput.Ref_Id, Park_Transform.Id, &PI_Flux);
	Car2Polar.Flux_Vd = PI_Flux.Uk;

  #if(DQ_DECOUPLING == ENABLED)
  int32_t wL_dq_Decoupling;

  // Use lower resolution speed ω = PMSM_FOC_PLL_PI.Uk to prevent overflow in multiplications of ωLId and ωLIq.
  // Temp variable for ωL, L scaled up by (MPS/K)^2.
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




/* To init rotor angle 1Q31 for first FOC PWM cycle, Lag/lead current angle Î³ by a 90Â° angle */
void pmsm_foc_init_foc_rotorangle(void)
{
  PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31 = (I_AngleQ31 - DEGREE_90) + (FOCInput.Ref_Speed << (16U - FOCInput.Res_Inc));

} /* End of pmsm_foc_init_foc_rotorangle () */

/* To init PI controllers' integral terms (Ik) for first FOC PWM cycle */
void pmsm_foc_init_foc_pi_iks(void)
{
  PI_Speed.Ik = PMSM_FOC_PLL_ESTIMATOR.current_i_mag; /* Init PI integral terms for smooth transition to FOC. */
  PI_Torque.Ik = (PMSM_FOC_PLL_ESTIMATOR.vref_x_cos_delta * 256) >> 8; /*
                                              * Init Vq of torque / Iq PI controller,
                                              * |Vref|cos(Î³-Î¸) = |Vref|cos(Î¸-Î³).
                                              */
  PI_Flux.Ik = (PMSM_FOC_PLL_ESTIMATOR.vref_x_sin_delta * 256) >> 8; /* Init Vd of flux / Id PI controller, |Vref|sin(Î³-Î¸) < 0 typically. */

  PMSM_FOC_PLL_PI.ik = FOCInput.Ref_Speed; /* Init rotor speed Ï‰r of PLL Observer PI controller. */

  PI_Speed.Ik <<= PI_Speed.Scale_KpKi; /* All PI integral terms left shift by PI_data->Scale_KpKi. */
  PI_Torque.Ik <<= PI_Torque.Scale_KpKi;
  PI_Flux.Ik <<= PI_Flux.Scale_KpKi;
  PMSM_FOC_PLL_PI.ik <<= PMSM_FOC_PLL_PI.scale_kp_ki;

	}	// End of pmsm_foc_init_foc_pi_iks ()



/*
 * To update angle θ (16-bit) of SVM reference vector Vref
 * Digital implementation θ[k] = θ[k-1] + ω[k]
 */
void pmsm_foc_update_vref_angle (int32_t Speed)
{
	Car2Polar.Vref_AngleQ31_Previous =  Car2Polar.Vref_AngleQ31;		// Record Vref angle θ of last PWM cycle.

  if (Motor.Rotation_Dir == DIRECTION_INC)
  {
	  /* If motor rotation direction - angle increasing. θ[k] = θ[k-1] + ω[k]. */
	  Car2Polar.Vref_AngleQ31 += (int32_t)((Speed * SPEED_TO_ANGLE_CONV_FACTOR)>>SPEED_TO_ANGLE_CONV_FACTOR_SCALE);
  }
  else
  {
	  Car2Polar.Vref_AngleQ31 -= (int32_t)((Speed * SPEED_TO_ANGLE_CONV_FACTOR)>>SPEED_TO_ANGLE_CONV_FACTOR_SCALE);
  }

	Car2Polar.SVM_Angle16 = Car2Polar.Vref_AngleQ31 >> 16U;			// SVM Vref angle θ (16-bit).
}	// End of pmsm_foc_update_vref_angle ()

/* Direct FOC startup. Motor startup to FOC closed-loop directly, no V/f or MET.*/
#if(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC)
/* Initial ramp up rate in FOC transition mode.*/
  #define RAMP_RATE_INIT_FOC    (RAMPUP_RATE << 0U)
#else
  #define RAMP_RATE_INIT_FOC    (RAMPUP_RATE << 0U)
#endif
/* (SPEED_LOW_LIMIT), (DEFAULT_SPEED_REFERENCE). Minimum startup speed for FOC. */
#define MIN_STARTUP_SPEED_FOC   (SPEED_LOW_LIMIT << 1)
/* Threshold speed to exit FOC. */
#define FOC_EXIT_SPEED  (SPEED_LOW_LIMIT * 3)



	/** Miscellaneous works in FOC, such as ramp up, speed adjustment, stop motor, etc
	 ** Do NOT add any CORDIC calculations in this function.
		 * -----------------------------------------------------------------------------------*/
__attribute__((section(".ram_code"))) void pmsm_foc_misc_works_of_foc (void)
{
	  uint16_t DCLink_adc_result;
      uint16_t IDCLink_adc_result;

      /* DC link ADC LPF. Read RES5 for ADC result (Previous ADC result) */
	  DCLink_adc_result = VADC_VDC_GROUP->RES[VADC_VDC_RESULT_REG];
		ADC.ADC_DCLink = (ADC.ADC_DCLink * ((1<<ADCLPF)-1) + DCLink_adc_result) >> ADCLPF;

		/* IDC link ADC LPF.  */
	   IDCLink_adc_result = VADC_IDC_GROUP->RES[VADC_IDC_RESULT_REG];
	   ADC.ADC_IDCLink = (ADC.ADC_IDCLink * ((1 << ADCLPF) - 1) + IDCLink_adc_result) >> ADCLPF;

#if(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
	   pmsm_foc_linear_torque_ramp_generator(Motor.Speed_by_POT_PWM,USER_IQ_RAMPUP, USER_IQ_RAMPDOWN, &FOCInput);

#if(ADVANCE_CONDITIONAL_MOTOR_STOP == ENABLED)
		if((SYSTEM_BE_IDLE) && (FOCInput.Ref_Iq <= Motor.Speed_by_POT_PWM) && Motor.Speed < FOC_EXIT_SPEED)
		{
		  /* Clear counters.*/
      Motor.Counter = 0;
      /* Next, go to Motor Stop (may brake motor and cause vibration).*/
      Motor.State = STOP_MOTOR;
		}
#elif(ADVANCE_CONDITIONAL_MOTOR_STOP == DISABLED)
    if(SYSTEM_BE_IDLE)
    {
      Motor.Counter = 0;
      Motor.State = STOP_MOTOR;
    }
#endif
#endif

#if(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)

		pmsm_foc_linear_vq_ramp_generator(Motor.Speed_by_POT_PWM ,USER_VQ_RAMPUP, USER_VQ_RAMPDOWN, &FOCInput);

#if(ADVANCE_CONDITIONAL_MOTOR_STOP == ENABLED)
//		if((SYSTEM_BE_IDLE) && (FOCInput.Vq <= Motor.Speed_by_POT_PWM) && Motor.Speed < FOC_EXIT_SPEED)
		if((SYSTEM_BE_IDLE) && (FOCInput.Vq <= Motor.Speed_by_POT_PWM))
		{
		  Motor.Counter = 0;
		  Motor.State = STOP_MOTOR;
		}
#elif(ADVANCE_CONDITIONAL_MOTOR_STOP == DISABLED)
    if(SYSTEM_BE_IDLE)
    {
      Motor.Counter = 0;
      Motor.State = STOP_MOTOR;
    }
#endif
#endif

		if (Motor.Status == MOTOR_TRANSITION)
		{
		  /* Motor in transition mode. Motor goes to higher speed first, important for startup. */
        #if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC))
//		  pmsm_foc_linear_ramp_generator(MIN_STARTUP_SPEED_FOC,RAMP_RATE_INIT_FOC,RAMPDOWN_RATE,SPEEDRAMPSTEP,&Motor.Ref_Speed); /*Comment off - using a start ref speed*/
			if(Motor.Ref_Speed<MIN_STARTUP_SPEED_FOC)
			{
				pmsm_foc_linear_ramp_generator(MIN_STARTUP_SPEED_FOC,RAMP_RATE_INIT_FOC,RAMPDOWN_RATE,SPEEDRAMPUPSTEP,&Motor.Ref_Speed); /*Comment off - using a start ref speed*/
			}
			else
			{
				pmsm_foc_linear_ramp_generator(MIN_STARTUP_SPEED_FOC,RAMP_RATE_INIT_FOC,RAMPDOWN_RATE,SPEEDRAMPDOWNSTEP,&Motor.Ref_Speed); /*Comment off - using a start ref speed*/
			}
        #endif
		}
		else   /* else, FOC is in stable mode.*/
		{
		  /*Prevously Over/under voltage protection in here, shifted to interface.c(inside irq function)
		   *
		   * */

        #if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC))
//			pmsm_foc_scurve_ramp_generator(Motor.Speed_by_POT_PWM, RAMPUP_RATE, RAMPDOWN_RATE, SPEEDRAMPSTEP,&Motor.Ref_Speed);
#if(MAX_POWER_LIMIT_EN_DIS == ENABLED)
			if((Motor.power_limit_flag == 0) || (Motor.motorstartstop == 0)) /*Did not reach limit, default ramp up/down*/
			{
				if(Motor.Ref_Speed<Motor.Speed_by_POT_PWM)
				{
					pmsm_foc_scurve_ramp_generator(Motor.Speed_by_POT_PWM, RAMPUP_RATE, RAMPDOWN_RATE, SPEEDRAMPUPSTEP,&Motor.Ref_Speed);
				}
				else
				{
					pmsm_foc_scurve_ramp_generator(Motor.Speed_by_POT_PWM, RAMPUP_RATE, RAMPDOWN_RATE, SPEEDRAMPDOWNSTEP,&Motor.Ref_Speed);
				}
			}
#else
			if(Motor.Ref_Speed<Motor.Speed_by_POT_PWM)
			{
				pmsm_foc_scurve_ramp_generator(Motor.Speed_by_POT_PWM, RAMPUP_RATE, RAMPDOWN_RATE, SPEEDRAMPUPSTEP,&Motor.Ref_Speed);
			}
			else
			{
				pmsm_foc_scurve_ramp_generator(Motor.Speed_by_POT_PWM, RAMPUP_RATE, RAMPDOWN_RATE, SPEEDRAMPDOWNSTEP,&Motor.Ref_Speed);
			}
#endif/*#if(MAX_POWER_LIMIT_EN_DIS == ENABLED)*/

          pmsm_foc_adjust_foc_parameters ();							// Adjust parameters, e.g.: for PI controllers. //temp comment off

          #if(ADVANCE_CONDITIONAL_MOTOR_STOP == ENABLED)
          #if((IR_Remote_Control == ENABLED) || (Remote_Control == ENABLED))
          	  if ((SYSTEM_BE_IDLE) && (Real_Speed_in_rpm<= Advanced_stop_exit_speed))
//            if ((SYSTEM_BE_IDLE) && (Motor.Ref_Speed <= Motor.Speed_by_POT_PWM) && (Motor.gradual_feedback_status == 0) && (Real_Speed_in_rpm<= Advanced_stop_exit_speed))
//            if ((SYSTEM_BE_IDLE) && (Motor.gradual_feedback_status == 0) && (Real_Speed_in_rpm<= Advanced_stop_exit_speed))
          #else
            if ((SYSTEM_BE_IDLE) && (Motor.Ref_Speed <= Motor.Speed_by_POT_PWM) && (Motor.Speed < FOC_EXIT_SPEED))
          #endif
            {	// If PWM duty cycle or POT ADC too low, and motor ref speed reached PWM set speed,
                Motor.Counter = 0;								  /* Clear counters. */
                Motor.Ramp_Counter = 0;
                Motor.State = STOP_MOTOR;						/* Next, go to Motor Stop (may brake motor and cause vibration). */
            }
          #elif(ADVANCE_CONDITIONAL_MOTOR_STOP == DISABLED)
            if (SYSTEM_BE_IDLE)
             { // If PWM duty cycle or POT ADC too low, and motor ref speed reached PWM set speed,
                 Motor.Counter = 0;                  /* Clear counters. */
                 Motor.Ramp_Counter = 0;
                 Motor.State = STOP_MOTOR;           /* Next, go to Motor Stop (may brake motor and cause vibration). */
             }
          #endif
        #endif
		}
}


#define PI_SPEED_IK_LIMIT_FINAL		((1<<15) <<PI_SPEED_SCALE_KPKI)
/* Step for parameter Ik change. */
#define SPEED_IK_ADJUST_STEP		(1<<7)

/** Adjust parameters, e.g.: for PI controllers, in FOC stable state
** Scheduling - using different parameters in different operating regions.
** Execution time: ?us (O3 - Optimize most).
 * ----------------------------------------------------------------------*/
__attribute__((section(".ram_code"))) void pmsm_foc_adjust_foc_parameters (void)
{
     /* Parameter adjustment not finished yet. */
    if (Motor.Adjust_Para_Flag != ADJUST_DONE)
    {
      /* 1). Ik limit scheduling for Speed PI controller:*/
      if (PI_Speed.Ik_limit_max < PI_SPEED_IK_LIMIT_FINAL)
      {
        /* Parameter adjusted gradually and regularly every PWM cycle. */
        PI_Speed.Ik_limit_max += SPEED_IK_ADJUST_STEP;
        PI_Speed.Ik_limit_min = - PI_Speed.Ik_limit_max;
      }
      else
      {
        /* To indicate that adjustment of this parameter is done. */
        Motor.Adjust_Para_Flag = ADJUST_DONE;
      }
    }
}




/* Init parameters of LIB. Init once only */

void pmsm_foc_systemparameters_init_onceonly (void)
{

  /* Init below once only before go to FOC: */
  FOCInput.Phase_L = L_OMEGALI;
  FOCInput.Phase_L_Scale = SCALE_L;

  FOCInput.LPF_N_BEMF = SHIFT_MET_PLL;

  FOCInput.CCU8_Period = (uint32_t) PERIOD_REG;

  FOCInput.Ref_Id = 0;

  FOCInput.Vq_Flag = 0; /* FOC Vq from Iq PI controller. */

  FOCInput.Iq_PI_Flag = 0; /* Reference of Iq PI controller from speed PI output. */

  FOCInput.RotorSpeed_In = 0;

  FOCInput.SVM_5_Segment_Flag = 0; /* 7-segment SVM. For 3-shunt current sensing only. */

  FOCInput.Flag_State = 0;

    #if(OVERCURRENT_PROTECTION == ENABLED)
     FOCInput.overcurrent_factor = 4096;                /* */
    #endif
  PMSM_FOC_PLL_ESTIMATOR.rotor_speed = Motor.Speed;
  FOCInput.Ref_Speed = Motor.Speed; /* Motor reference speed. */

  PMSM_FOC_PLL_PI.ik = PMSM_FOC_PLL_ESTIMATOR.rotor_speed << PMSM_FOC_PLL_PI.scale_kp_ki;

  FOCOutput.New_SVM_SectorNo = SVM.CurrentSectorNo;

  FOCOutput.Previous_SVM_SectorNo = 0;
  FOCOutput.New_SVM_SectorNo = 0; /* Use default SVM sector. */

  PMSM_FOC_PLL_ESTIMATOR.phase_inductance_Ls = DEFAULT_L_SCALEDUP;
  PMSM_FOC_PLL_ESTIMATOR.phase_inductance_scale = DEFAULT_SCALE_OF_L;


  PMSM_FOC_PLL_ESTIMATOR.speed_angle_conversion_factor = SPEED_TO_ANGLE_CONV_FACTOR;
  PMSM_FOC_PLL_ESTIMATOR.speed_angle_conversion_factor_scale = SPEED_TO_ANGLE_CONV_FACTOR_SCALE;

  /* Filter coefficients used inside PLL observer */
  PMSM_FOC_PLL_ESTIMATOR.lpf_n_bemf = USER_PLL_LPF;
  PMSM_FOC_PLL_ESTIMATOR.lpf_n_speed = USER_PLL_SPEED_LPF;

  /* Reset PLL observer parameters */
  PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31 = 0;
  PMSM_FOC_PLL_ESTIMATOR.rotor_speed = 0;

  PMSM_FOC_PLL_ESTIMATOR.current_i_mag = 0;
  PMSM_FOC_PLL_ESTIMATOR.current_i_mag_filtered = 0;

  PMSM_FOC_PLL_ESTIMATOR.bemf_1 = 0;
  PMSM_FOC_PLL_ESTIMATOR.bemf_2 = 0;

  PMSM_FOC_PLL_PI.ik = PMSM_FOC_PLL_ESTIMATOR.rotor_speed << PMSM_FOC_PLL_PI.scale_kp_ki;

} /* End of pmsm_foc_systemparameters_init_onceonly () */


#if(MOTOR_STALL_DETECTION == ENABLED)
PMSM_FOC_STALL_DETECT_t stall_detect;

/* Init variables */
void pmsm_foc_stall_init()
{
  stall_detect.phase_inductance_Lq = MOTOR_LQ_SCALEUP;
  stall_detect.phase_inductance_scale = MOTOR_SCALE_OF_LQ;
  stall_detect.phase_resistance = DEFAULT_R_SCALEDUP;
  stall_detect.phase_resistance_scale = (uint16_t)DEFAULT_SCALE_OF_R;
  stall_detect.stall_coefficient = STALL_RATIO_SCALEUP;
  stall_detect.stall_ratio_scale = STALL_RATIO_SCALE;
  stall_detect.threshold_stall_time = STALL_THRESHOLD_TIME;
  stall_detect.LPF_stall = 4;
  stall_detect.motor_stall_status = 0;
  stall_detect.stall_current = STALL_CURRENT;
#if(IP_STALL)
  PMSM_FOC_STALL_InitVariables(&stall_detect);
#else
  PMSM_FOC_Stall_Detection_Init();
#endif
}
#endif
