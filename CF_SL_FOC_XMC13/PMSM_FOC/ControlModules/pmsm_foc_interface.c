/**
 * @file pmsm_foc_interface.c
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
 * @file pmsm_foc_interface.c
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

#include "pmsm_foc_interface.h"
#include "..\PMSM_FOC\ControlModules\pmsm_foc_functions.h"
#include "..\MIDSys\pmsm_foc_stall_detection.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/* Global variables:*/
extern ClarkeTransformType Clarke_Transform;
extern Car2PolType Car2Polar;
extern PMSM_FOC_PLL_ESTIMATOR_t PMSM_FOC_PLL_ESTIMATOR;
/* Angle γ (1Q23 << 8) of current space vector, from last PWM cycle */
extern int32_t I_AngleQ31;

/* ADC results, trigger positions. */
extern ADCType ADC;
/* Motor control information */
extern MotorControlType Motor;
/* Motor current and current space vector. */
extern CurrentType Current;
/* For trip / over-current detection, and protection. */
extern TripType Trip;
/* For motor startup lock / fail / stall detection, and protection. */
extern StallType Stall;
/* For Hall signal processing. */
extern HallType Hall;
/* SVM information, such as sector 0 ~ 5 (A ~ F) in SVM space vector hexagon. */
extern SVMType SVM;
/* Output for FOC LIB. */
extern FOCOutputType FOCOutput;
/* Parameters input for FOC LIB. */
extern FOCInputType FOCInput;
/* Speed PI controller. */
extern PI_Coefs_Type PI_Speed;
/* Torque / Iq PI controller. */
extern PI_Coefs_Type PI_Torque;
/* Flux /Id PI controller. */
extern PI_Coefs_Type PI_Flux;
/* PLL rotor speed PI controller. */
extern PMSM_FOC_PLL_PI_t PMSM_FOC_PLL_PI;
#if(START_UP_MODE_1 == ENABLED)
/* Catch Free Running. */
extern CFR_type CFR;
#endif

#if(auto_start == ENABLED)
extern uint32_t User_Para[70];
#endif

/* Data Structure initialization */

/* Global variable. MCU Reset Status Information, reason of last reset. */
extern uint32_t * NEW_SHS0_CALOC1;
extern uint32_t g_mcu_reset_status;

/* 0.05s, time that motor keeps in Stop Mode, x PWM period. */
#define TIME_OF_STOP 	(200U)
//#define TIME_OF_STOP  (16000U)

#define TIME_OF_VFRAMP		(750U)
/* Step that voltage increases. */
#define ALIGNMENT_STEP		(32U)
/* Voltage for rotor preposition/alignment. */
#define ALIGNMENT_VOLT		((VQ_VF_OFFSET * 2) >> 1)
/* Ratio for ramp up slowly in V/f open loop.*/
#define RAMP_RATIO_VF		(1U)
/* Indicates a running CORDIC calculation if MATH->STATC[0] (i.e.: BSY) = 1. */
#define CORDIC_IS_BSY (MATH->STATC & 0x01)

#if(uCPROBE_GUI_no_UART == ENABLED)
uint32_t Speed_in_rpm;                     /* uC_Probe variable */
uint32_t Real_Speed_in_rpm;                /* uC_Probe variable */
#endif

/* In case of any error, motor start function won't be started
* until clear all the errors.
* ----------------------------------------------------------*/
void pmsm_foc_motorstart(int32_t user_set_speed)
{
  if((Motor.State != TRAP_PROTECTION) || (Motor.State != DCLINK_OVER_UNDER_VOLTAGE))
  {
    Motor.motorstartstop = 1;
  }
}

void pmsm_foc_motorstop(void)
{
  Motor.motorstartstop = 0;
}

/* API to perform motor STOP
* ----------------------------------------------------------*/
void pmsm_foc_setspeedzero(void)
{
  /* Change motor control state machine to stop */
  ADC.ADC_POT = 0;
}

/* ------------------------------------------GUI Interface Sync------------------------------------------------------ */
/*Motor ON function same as remote function*/
void motor_start(void) /*if (Motor.motorstartstop == 0) 0 - Button 'OFF' State*/
{
#if((Remote_Control == ENABLED) || (IR_Remote_Control == ENABLED))

  if(Motor.initial_autostart == 1) /*For initial speed start when button press*/
  {
    Motor.initial_autostart = 0;
/*===========================================*/
#if(auto_start == ENABLED)
    if((User_Para[61] >= 1) &&(User_Para[61] <=6))
    {
      Motor.Speed_lvl = User_Para[61];
    }
    else
    {
      Motor.Speed_lvl = 1;
    }
#else//#if(auto_start == ENABLED)
    Motor.Speed_lvl = 1;
#endif//#if(auto_start == ENABLED)
/*===========================================*/
    ADC.ADC_POT = spd_1_adc_pot; /*Required above idle value in order for motor start*/
//    Motor.motor_lastspeed = ADC.ADC_POT;  /*For remembering the last speed*/
  }

  else
  {
	  ADC.ADC_POT = spd_1_adc_pot; /*Required above idle value in order for motor start*/
  }

#else/*Further development can be done if remote is not used.*/

#endif/*(Remote_Control == ENABLED)*/

  Motor.SW_overcurrent_Status = 0x00;
  Motor.U_O_Voltage_Status = 0x00;
  Motor.HW_OCP_TRAP_Status = 0x00;

#if(MOTOR_STALL_DETECTION == ENABLED)
  Motor.Stall_detect_status = 0x00;
  pmsm_foc_stall_init();
#endif
  XMC_GPIO_SetOutputLow(LED_protection);

}/*void motor_start(void)*/

void motor_stop(void)
{
#if((Remote_Control == ENABLED) || (IR_Remote_Control == ENABLED))

  pmsm_foc_setspeedzero();            /*A default function that set ADC_POT to zero */
  Motor.Speed_by_POT_PWM = 0U;

#else
  pmsm_foc_setspeedzero();            /*A default function that set ADC_POT to zero */

#endif/*(Remote_Control == ENABLED)*/

}

/* ----------------------------------------------------------------------------------------------------------------- */

/* V/f Open Loop Ramp Up with rotor initial preposition/alignment
 * ----------------------------------------------------------*/
void pmsm_foc_vf_openloop_rampup(void)
{
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  pmsm_foc_current_reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);

#else
    pmsm_foc_get_adcphasecurrent(SVM.PreviousSectorNo, SVM.CurrentSectorNo, &ADC);
    pmsm_foc_current_reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
#endif

    /* To get I_Alpha and I_Beta of last PWM cycle, scale down I_Mag (i.e.: |I|) by 2/3. */
    pmsm_foc_clarketransform(Current.I_U, Current.I_V, Current.I_W, &Clarke_Transform);

    #if(TEMP_STALL_TEST == 1)
    pmsm_foc_parktransform(Clarke_Transform.I_Alpha_1Q31, Clarke_Transform.I_Beta_1Q31, PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31);

    FOCOutput.Previous_SVM_SectorNo = FOCOutput.New_SVM_SectorNo; // Record previous SVM sector number.

    #if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC) ||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC))
    /* PI Controller #1 - Speed / Speed PI controller of FOC */
    pmsm_foc_pi_controller_anti_windup(FOCInput.Ref_Speed,PMSM_FOC_PLL_ESTIMATOR.RotorSpeed_In, &PI_Speed);
    #endif

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
    #endif /*(uCPROBE_GUI == ENABLED)*/
    #endif /*(MOTOR_STALL_DETECTION == ENABLED)*/
    #endif /*(TEMP_STALL_TEST == 1)*/

    Car2Polar.SVM_Vref16 = (VQ_VF_SLEW * (Motor.Speed))>>15;	
    Car2Polar.SVM_Vref16 += VQ_VF_OFFSET;	

    /* To update angle θ (16-bit) of SVM reference vector Vref. */
    pmsm_foc_update_vref_angle (Motor.Speed);

    if (Motor.Status == MOTOR_TRANSITION)
    {       /* Motor is in transition mode */
      if (Motor.Speed < VF_TRANSITION_SPEED)
      {
      /* Motor speed not reach V/f open-loop to MET/FOC transition speed */
      /* Speed ramp counter ++. */
      Motor.Ramp_Counter ++;
      	if (Motor.Ramp_Counter > VF_RAMPUP_RATE)
        {
            /* Motor speed ++. */
            Motor.Speed += VF_SPEEDRAMPSTEP;
            /* Clear ramp counter.*/
            Motor.Ramp_Counter = 0;
        }
      }
      else
      {
        /* Motor run at V/f constant speed for a while.*/
        Motor.Counter ++;
        if (Motor.Counter > TIME_OF_VFRAMP)
        {
          /* Change flag: motor in stable mode of V/f ramp-up. */
          Motor.Status = MOTOR_STABLE;
          Motor.Counter = 0;
        }
      }
    }
    else
    {
#if(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_ONLY)

#elif(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC)
      /* Next, go to MET (Maximum Efficiency Tracking) closed-loop control. */
      Motor.State = MET_FOC;

      FOCInput.BEMF1 = 0;
      FOCInput.BEMF2 = 0;

      /* MET loop unlocked, Init e Threshold LOW. (Initially, e_Th = e_Th_L.) */
      FOCInput.Threshold_LOW = THRESHOLD_LOW;
      FOCInput.Threshold_HIGH = THRESHOLD_HIGH;
      FOCInput.Threshold = (Motor.Speed * THRESHOLD_LOW);

      FOCInput.Phase_L = L_OMEGALI;
      FOCInput.Phase_L_Scale = SCALE_L;

      /* Resolution increase, use (16 + Res_Inc) bit to represent 360 deg. */
      FOCInput.LPF_N_BEMF = SHIFT_MET_PLL;

      /* Motor in transition mode.*/
      Motor.Status = MOTOR_TRANSITION;
      Motor.Counter = 0;
      Motor.Ramp_Counter = 0;
      Motor.Alignment_Counter = 0;
      /* Slower ramp up and ramp down for S-curve profile.*/
      Motor.Ramp_Up_Rate = RAMPUP_RATE << USER_RATIO_S;
#endif
    }

    /* Limit of |Vref| (16-bit). */
    #define SVM_VREF16_MAX    (32767U)
    if (Car2Polar.SVM_Vref16 > SVM_VREF16_MAX)
    {
      /*  Limit |Vref| maximum value.*/
      Car2Polar.SVM_Vref16 = SVM_VREF16_MAX;
    }

    /*  Update SVM PWM. */
    pmsm_foc_svpwm_update(Car2Polar.SVM_Vref16, Car2Polar.SVM_Angle16);

    /* Record SVM reference vector magnitude (32-bit) of last PWM cycle.*/
    Car2Polar.Vref32_Previous = Car2Polar.Vref32;
    Car2Polar.Vref32 = Car2Polar.SVM_Vref16 << CORDIC_SHIFT;

    /* Init for smooth transition from V/f to FOC closed-loop.*/
    pmsm_foc_init_smooth_transition_to_foc (Motor.Speed);
    Car2Polar.Vref_AngleQ31 = Car2Polar.SVM_Angle16 << 16;
	
    if(Motor.motorstartstop == 0)
    {
        pmsm_foc_disable_inverter();
    	Motor.Counter = 0;                /* Clear counters. */
    	Motor.Ramp_Counter = 0;
    	Motor.State = STOP_MOTOR;         /* Next, go to Motor Stop. */
        Motor.Speed = 0;		
    }
}

/** Stop the motor, check PWM or POT ADC (for adjusting motor speed)
 ** Execution time: ?us (O3 - Optimize most).
	* ---------------------------------------------------------------------*/
void pmsm_foc_stop_motor (void)
{
    static uint32_t  local_counter = 0;      // General purpose counter

    #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
    pmsm_foc_current_reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);
    #else

      /* 2or3-shunt 3-phase current reconstruction, to get Iu and Iv */
    pmsm_foc_current_reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
    #endif
    pmsm_foc_clarketransform(Current.I_U, Current.I_V,Current.I_W, &Clarke_Transform);
    local_counter ++;

    // if set target too low, or button is stop, then stay in stop. Otherwise go to Openloop/closeloop
    if (Motor.motorstartstop == 0)
    {
      /* If system is idle, i.e.: PWM duty cycle or POT ADC too low.*/
      /* Reset counter, local_counter < TIME_OF_STOP to prevent it from re-start of motor.*/
      local_counter = 0;
      Motor.Speed = 0;
    }

    if (local_counter > TIME_OF_STOP)  //Enable PWM and go to running
    {
      local_counter = 0;


      /*Main initialization (Require in order to work properly and prevent OCP) */
      pmsm_foc_init();

      PI_Speed.Ik = 0;
      PI_Speed.Uk = 0;
      PI_Torque.Ik = 0;
      PI_Torque.Uk = 0;
      PI_Flux.Ik = 0;
      PI_Flux.Uk = 0;

      Current.I_U = 0;
      Current.I_V = 0;
      Current.I_W = 0;

      #if (CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      /* Init ADC, for current sensing, ADC of DC link Vdc (and POT). Do at later stage of the init */
      pmsm_adc_module_init();
      pmsm_phasecurrent_init();
      pmsm_adc_dclink_init();
      pmsm_adc_pot_init();
      #endif

      CCUx_SynStart();

      /* Direct FOC startup. Motor startup to FOC closed-loop directly, no V/f or MET*/
      #if(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC || MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC || MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)

      #if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)
      Motor.State = CATCH_FREERUNNING;
      #else
      /* Next, go to rotor initial preposition/alignment.*/
      Motor.State = BRAKE_BOOTSTRAP;
      #endif


      #elif(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC)
      Motor.State = BRAKE_BOOTSTRAP;   //go to bootstrap and re-start the motor
      #elif(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_ONLY)
      Motor.State = BRAKE_BOOTSTRAP;
      #endif

      if (Motor.State == VFOPENLOOP_RAMP_UP)
      {
        Motor.Ramp_Up_Rate = (RAMPUP_RATE * 40); /* In V/f, much slower initial ramp up for S-curve profile. */
      }

    }
    else
    {
      /* To update angle θ (16-bit) of SVM reference vector Vref */
      pmsm_foc_update_vref_angle (Motor.Speed);
      pmsm_foc_disable_inverter();
    }

}

/** To brake the motor, charge gate driver bootstrap capacitors (if any)
 ** Execution time: ?us (O3 - Optimize most).
  * -------------------------------------------------------------------------*/
void pmsm_foc_brake_motor_bootstrap_charge(void)
{
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)

    /* Get_ADC_SingleShuntCurrent(&ADC); */
    ADC.ADC_Bias = (uint32_t) ((ADC.ADC_Bias * (((uint32_t) 1 << SHIFT_BIAS_LPF) - 1U)) + ADC.ADC_ResultTz1)
                                   >> SHIFT_BIAS_LPF;
#else
    pmsm_foc_get_adcphasecurrent(SVM.PreviousSectorNo, SVM.CurrentSectorNo, &ADC);

    /* Read Iu ADC bias */
    ADC.ADC_Bias_Iu = (uint32_t) ((ADC.ADC_Bias_Iu * (((uint32_t) 1 << SHIFT_BIAS_LPF) - 1U)) + ADC.ADC_Iu)
                                   >> SHIFT_BIAS_LPF;
    /* Read Iv ADC bias */
    ADC.ADC_Bias_Iv = (uint32_t) ((ADC.ADC_Bias_Iv * (((uint32_t) 1 << SHIFT_BIAS_LPF) - 1U)) + ADC.ADC_Iv)
                                   >> SHIFT_BIAS_LPF;
    /* Read Iw ADC bias */
    ADC.ADC_Bias_Iw = (uint32_t) ((ADC.ADC_Bias_Iw * (((uint32_t) 1 << SHIFT_BIAS_LPF) - 1U)) + ADC.ADC_Iw)
                                   >> SHIFT_BIAS_LPF;
#endif

    Motor.Counter++;

    if (Motor.motorstartstop == 0)
    {
      /* If system is idle, i.e.: PWM duty cycle or POT ADC too low. */
      Motor.Counter = 0U; /* cannot go to ramp up, keep motor braking. */
      Motor.State = STOP_MOTOR;
    }

    /*To determine of the Bootstrap brake timing - from CFR motor state.*/
#if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)
    if(Motor.CFR_brake_time == 3)
    {
      Motor.USER_DEFINE_BOOTSTRAP_BRAKE_TIME = CFR_BRAKE_TIME_3;   /*If CFR normal rev higher speed*/
    }
    else if(Motor.CFR_brake_time == 2)
    {
      Motor.USER_DEFINE_BOOTSTRAP_BRAKE_TIME = CFR_BRAKE_TIME_2;   /*If CFR normal rev lower speed*/
    }
    else if(Motor.CFR_brake_time == 1)
    {
      Motor.USER_DEFINE_BOOTSTRAP_BRAKE_TIME = CFR_BRAKE_TIME_1;   /*If CFR normal non-rev/rev lower speed*/
    }
    else
    {
      Motor.USER_DEFINE_BOOTSTRAP_BRAKE_TIME = BRAKE_TIME;         /*If CFR normal lower speed / stop*/
    }

    if (Motor.Counter > Motor.USER_DEFINE_BOOTSTRAP_BRAKE_TIME)
#else
    if (Motor.Counter > BRAKE_TIME)
#endif
    {
#if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)
      /*Reset condition for bootstrap brake time*/
      Motor.CFR_brake_time = 0;
      Motor.CFR_speed_dir_high = 0;
#endif

      // Enable the PWM and to to running. This is necessary when using our 36W board
      /* Init CCU8 */
        pmsm_foc_ccu8_init();
      /* Init GPIOs */
        pmsm_foc_gpio_Init();  // This is also necessary for open PWM if PWM is disabled before
      /* Synchronous start of CAPCOM modules, e.g.: CCU8x, and or CCU4x */
        CCUx_SynStart();



#if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_ONLY)||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC))
      /* Timer decides when to stop motor braking.*/
      Motor.State = VFOPENLOOP_RAMP_UP;
      /* Next, go to V/f ramp-up.*/
      Motor.Status = MOTOR_TRANSITION;
      /* Motor in transition mode.*/
      if (Motor.State == VFOPENLOOP_RAMP_UP)
      {
        /* In V/f, much slower initial ramp up for S-curve profile.*/
        Motor.Ramp_Up_Rate = (RAMPUP_RATE * 40);
      }
#elif((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC)||(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)||(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC))
#if(USER_ROTOR_IPD_METHOD == ROTOR_IPD_NONE)
      Motor.State = FOC_CLOSED_LOOP;
#elif(USER_ROTOR_IPD_METHOD == ROTOR_IPD_PRE_ALIGNMENT)
      /* Next, go to rotor initial preposition/alignment. */
      Motor.State = (uint32_t) PRE_POSITIONING;
#else
      //ROTOR_IPD_INDUCTIVE_SENSING
      Motor.State = PMSM_FOC_MSM_ROTOR_IPD;
#endif

      /* Motor in transition mode. */
      Motor.Status = MOTOR_TRANSITION;
#endif
      /* Clear counters. */
      Motor.Counter = 0U;
      Motor.Ramp_Counter = 0U;

      Current.I_U = 0;
      Current.I_V = 0;
      Current.I_W = 0;

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      ADC.ADC3Trig_Point = 0;
      ADC.ADC4Trig_Point = 0;
      ADC.ADC_Result1 = 0;
      ADC.ADC_Result2 = 0;
      ADC.Result_Flag = RESULTS_ADCTZ12;
      /* Set ADC trigger for ADCTz1/Tz2, for single-shunt current sensing only. */
      pmsm_foc_adctz12_triggersetting ();
#endif
    }
}


__attribute__((section(".ram_code"))) void pmsm_foc_linear_ramp_generator(int32_t set_val, int32_t rampup_rate, int32_t rampdown_rate,
                                         int32_t speedrampstep, int32_t *reference_val)
{
  if (*reference_val != set_val)
  {
    /* in FOC, ωref not reach the target speed.*/
    if (*reference_val < set_val)
    {
      /* Speed ramp counter ++.*/
      Motor.Ramp_Counter ++;
      if (Motor.Ramp_Counter > Motor.Ramp_Up_Rate)
      {
        /* Ramp up slowly (if needed) at start of FOC.*/
        if (Motor.Ramp_Up_Rate > rampup_rate)
        {
          /* Increase acceleration step by step.*/
          Motor.Ramp_Up_Rate --;
        }

        /*  ωref ++.*/
        *reference_val += speedrampstep;
        /* Clear ramp counter. */
        Motor.Ramp_Counter = 0;
      }
    }
    else
    {
      /* Speed ramp counter ++.*/
      Motor.Ramp_Counter ++;
      if (Motor.Ramp_Counter > rampdown_rate)
      {
        /* ωref --.*/
        *reference_val -= speedrampstep;
        Motor.Ramp_Counter = 0;
      }
    }
  }
  else
  {
    /* ωref reach the target speed */
    /* Update counter */
    Motor.Counter ++;
    /*15, 150 or 1500. Time that FOC becomes stable, x PWM period.*/
    if (Motor.Counter > 2U)
    {
      /* Change flag: in FOC stable mode. */
      Motor.Status = MOTOR_STABLE;
      Motor.Counter = 0;
      /* Clear counter */
      Motor.Ramp_Counter = 0;
    }
  }
}

/*  To use S-curve profile in motor ramp up / down. Comment out to use trapezoidal profile. */
#define S_CURVE_PROFILE   1
/* Speed threshold for entering second S-curve of ramp up / down. */
#define SPEED_TH_2ND_S    (SPEED_LOW_LIMIT >> 0U)

__attribute__((section(".ram_code"))) void pmsm_foc_scurve_ramp_generator(int32_t set_val, int32_t rampup_rate, int32_t rampdown_rate,
                                         int32_t speedrampstep, int32_t *reference_val)
{
  if (*reference_val == set_val)
  {
    /* For most of the time, motor ref speed = speed set by POT ADC or PWM. */
    Motor.Ramp_Counter = 0;
    Motor.Ramp_Up_Rate = rampup_rate << USER_RATIO_S;

    /* Reset to slower ramp up and ramp down for S-curve profile. */
    Motor.Ramp_Dn_Rate = rampdown_rate << (USER_RATIO_S - 1);
  }
  else if (*reference_val < set_val)
  {
    /* Motor ref speed lower than speed set by POT or PWM. */
    /* Speed ramp counter ++. */
    Motor.Ramp_Counter++;
    if (Motor.Ramp_Counter > Motor.Ramp_Up_Rate)
    {
      if ((set_val - *reference_val) > SPEED_TH_2ND_S)
      {
        /* First S-curve of ramp up, and constant acceleration. */
        if (Motor.Ramp_Up_Rate > rampup_rate)
        {
          /* Increase acceleration step by step. */
          Motor.Ramp_Up_Rate--;
        }
      }
      else
      {
        /* Second S-curve of ramp up. */
        if (Motor.Ramp_Up_Rate < (rampup_rate << USER_RATIO_S))
        {
          Motor.Ramp_Up_Rate++;
        }
      }
      /* Motor ref speed ++. */
      *reference_val += speedrampstep;
      Motor.Ramp_Counter = 0;
    }
  }
  else
  {
    /* Motor ref speed higher than speed set by POT or PWM. */
    /* Speed ramp counter ++. */
    Motor.Ramp_Counter++;
    if (Motor.Ramp_Counter > Motor.Ramp_Dn_Rate)
    {
      if ((*reference_val - set_val) > SPEED_TH_2ND_S)
      {
        /* First S-curve of ramp down, and constant deceleration. */
        if (Motor.Ramp_Dn_Rate > rampdown_rate)
        {
          /* Increase deceleration step by step. */
          Motor.Ramp_Dn_Rate--;
        }
      }
      else
      {
        /* Second S-curve of ramp down. */
        if (Motor.Ramp_Dn_Rate < (rampdown_rate << (USER_RATIO_S - 1)))
        {
          Motor.Ramp_Dn_Rate++;
        }
      }

        if (ADC.ADC_DCLink < VDC_MAX_LIMIT)
        {
          /* If DC link voltage Vdc is too high, stop ramp-down motor.*/
          /* Motor ref speed --.*/
          *reference_val -= speedrampstep;
        }
        Motor.Ramp_Counter = 0;
      }

    }

 }

/* Non-Real-Time Tasks Configuration */
/* 2 ~ 100, x CCU8 PWM period. For tasks that don't need real-time computing.*/
#define NON_REALTIME_RATE 64
#define POTADC_LPF    (5U)          // (5U). ADC uses LPF.

uint32_t fault_counter = 0 ; /*number of fault occur for retry*/
uint32_t fault_timer_counter = 0; /*The counter for going into reset timing threshold*/

/** Miscellaneous works in CCU80_0_IRQHandler, such as tasks that don't need real-time computing
	* -------------------------------------------------------------------------------------------------*/
__attribute__((section(".ram_code"))) void pmsm_foc_misc_works_of_irq (void)
{
//  XMC_GPIO_SetOutputHigh(mcu_load);
  /* Handle tasks that don't need real-time computing:*/
  #if(SETTING_TARGET_SPEED == BY_POT_ONLY)
      uint16_t pot_adc_result;
  #endif

  XMC_VADC_GROUP_ScanTriggerConversion(VADC_G0);
  XMC_VADC_GROUP_ScanTriggerConversion(VADC_G1);

      /* Counter ++. */
	Motor.Non_RealTime_Counter ++;
	if (Motor.Non_RealTime_Counter > NON_REALTIME_RATE)
	{
	    /* Reset counter.*/
		  Motor.Non_RealTime_Counter = 0;

      #if(SETTING_TARGET_SPEED == BY_POT_ONLY)
        pot_adc_result = XMC_VADC_GROUP_GetResult(VADC_POT_GROUP,VADC_POT_RESULT_REG);
        /* POT ADC LPF. Read RES7 for ADC result (Previous ADC result). */
        ADC.ADC_POT = (ADC.ADC_POT * ((1<<POTADC_LPF)-1) + pot_adc_result) >> POTADC_LPF;
      #endif

#if(uCPROBE_GUI_no_UART == ENABLED)

#if((Remote_Control == DISABLED) && (IR_Remote_Control == DISABLED))

  if( Motor.motorstartstop == 0)  // Press the button the stop motor
  {
    pmsm_foc_setspeedzero();
  }

  else    /* The motor will be go to brake bootstrap and start*/
  {

  }

#elif((Remote_Control == ENABLED) || (IR_Remote_Control == ENABLED))
/* ------------------------------------------GUI Interface Sync------------------------------------------------------ */
/* 1) If the remote button 'ON' or 'OFF' is pressed, it will run the same motor_start/stop function.
 * 2) Same as when from GUI interface, it will run the same motor_start/stop function.
 * 3) Previously, remote and the GUI interface runs differently especially for the motor start.
 * 4) When coding of the ON/OFF function, make sure to check of the remote functionality too.
 *  */

  if(Motor.motorstartstop == 0)/*Button is 'OFF'*/
  {
    if(Motor.motorstartstop != Motor.prev_button_state )
    {
      motor_stop();
    }
    Motor.prev_button_state = 0;  /*save the current button state status*/
  }

  else if(Motor.motorstartstop == 1)/*Button is 'ON'*/
  {
    if(Motor.motorstartstop != Motor.prev_button_state )
    {
      motor_start();
    }
    Motor.prev_button_state = 1;
  }

#endif/*(Remote_Control == ENABLED)*/

#endif/*(uCPROBE_GUI_no_UART == ENABLED)*/
/* --------------------------------------------------------------------------------------------------------------------- */

      #if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC))
        #define ADC_RESOLUTION_REDUCE_BIT   (5)
			#if(Remote_Control == ENABLED)
  	  	  	  if(Motor.motorstartstop==1)
  	  	  	  {
					if(Motor.Speed_lvl == 5) Motor.Speed_by_POT_PWM = 32767U; /*350rpm - 32767*/
					else if(Motor.Speed_lvl == 4) Motor.Speed_by_POT_PWM = 28081U; /*300rpm - 0.857*32767*/
					else if(Motor.Speed_lvl == 3) Motor.Speed_by_POT_PWM = 23396U; /*250rpm - 0.714*32767*/
					else if(Motor.Speed_lvl == 2) Motor.Speed_by_POT_PWM = 18710U; /*200rpm - 0.571*32767*/
					else if(Motor.Speed_lvl == 1) Motor.Speed_by_POT_PWM = 14043U; /*150rpm - 0.428*32767*/
					else Motor.Speed_by_POT_PWM = 0U;
  	  	  	  }
			#else
				/* POT ADC values 0 ~ 2^(12-ADC_RESOLUTION_REDUCE_BIT) represent motor target speed of SPEED_LOW_LIMIT ~ SPEED_HIGH_LIMIT:*/
				Motor.Speed_by_POT_PWM = SPEED_LOW_LIMIT + (((SPEED_HIGH_LIMIT - SPEED_LOW_LIMIT) * (ADC.ADC_POT >> ADC_RESOLUTION_REDUCE_BIT)) >> (12U - ADC_RESOLUTION_REDUCE_BIT));
				/* Limit speed, in case ADC values not 0 ~ 2^12.*/
				Motor.Speed_by_POT_PWM = MIN_MAX_LIMIT(Motor.Speed_by_POT_PWM, SPEED_HIGH_LIMIT, SPEED_LOW_LIMIT);
			#endif
        #elif(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
        /* POT ADC values 0 ~ 2^12 represent motor target speed of SPEED_LOW_LIMIT ~ SPEED_HIGH_LIMIT:*/
        Motor.Speed_by_POT_PWM = USER_IQ_REF_LOW_LIMIT + (((USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT) * ADC.ADC_POT) >> 12);
        /* Limit speed, in case ADC values not 0 ~ 2^12.*/
        Motor.Speed_by_POT_PWM = MIN_MAX_LIMIT(Motor.Speed_by_POT_PWM, USER_IQ_REF_HIGH_LIMIT, USER_IQ_REF_LOW_LIMIT);
      #elif(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
        /* POT ADC values 0 ~ 2^12 represent motor target speed of SPEED_LOW_LIMIT ~ SPEED_HIGH_LIMIT:*/
        Motor.Speed_by_POT_PWM = USER_VQ_REF_LOW_LIMIT + (((USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT) * ADC.ADC_POT) >> 12);
        /* Limit speed, in case ADC values not 0 ~ 2^12.*/
        Motor.Speed_by_POT_PWM = MIN_MAX_LIMIT(Motor.Speed_by_POT_PWM, USER_VQ_REF_HIGH_LIMIT, USER_VQ_REF_LOW_LIMIT);
      #endif
      #if(WATCH_DOG_TIMER == ENABLED)
			/* Service watchdog. Without WDT service regularly , it will reset system.*/
			XMC_WDT_Service();
      #endif

			Motor.UART_Counter ++;
			if (Motor.UART_Counter > (3U))
			{
        /* Reset counter.*/
        Motor.UART_Counter = 0;
        #if(SETTING_TARGET_SPEED == BY_UART_ONLY)
        /* Use UART to adjust POT ADC values, and hence motor speed.*/
        /* Use UART to set POT ADC, by polling.*/
        pmsm_foc_uart_set_pot_adc();
        #endif
			}

      /* uC Probe Processing Handling */
      /* uC Speed Display */
      #define SPEED_RPM_LPF    (1U)                 // (5U). speed uses LPF.

      Speed_in_rpm = ((Motor.Ref_Speed * SPEED_TO_RPM) >> SCALE_SPEED_TO_RPM);
      Real_Speed_in_rpm = (Real_Speed_in_rpm * ((1<<SPEED_RPM_LPF)-1) + ((Motor.Speed * SPEED_TO_RPM) >> SCALE_SPEED_TO_RPM)) >> SPEED_RPM_LPF;
      if (Motor.State != FOC_CLOSED_LOOP)
      {
        Speed_in_rpm = Real_Speed_in_rpm;
      }

    //add Vdc_link sampling and protection
      uint16_t DCLink_adc_result;

     /* DC link ADC LPF. Read RES5 for ADC result (Previous ADC result) */
      DCLink_adc_result = VADC_VDC_GROUP->RES[VADC_VDC_RESULT_REG];
      ADC.ADC_DCLink = (ADC.ADC_DCLink * ((1<<ADCLPF)-1) + DCLink_adc_result) >> ADCLPF;

/*---------------------------------- Over/under voltage protection ----------------------------------------*/
#if(VDC_UNDER_OVERVOLTAGE_PROTECTION == ENABLED)

        if (ADC.ADC_DCLink > VDC_OVER_LIMIT)
        {
          /* Motor.error_status = PMSM_FOC_EID_OVER_VOLT;*/
          Motor.State = DCLINK_OVER_UNDER_VOLTAGE;
          XMC_GPIO_SetOutputHigh(LED_protection); // Testing pin for ocp
          Motor.U_O_Voltage_Status = 0x01; /*Under/over voltage protection status*/
          Motor.fault_status = 1; /*Fault status for over voltage detect*/

          /* Disable gate driver. */
          if (USE_INVERTER_EN_PIN == 0)
          {
            pmsm_foc_disable_inverter(); /*4: another stop*/

#if(auto_start == ENABLED)
            /*Power disruption auto start up*/
            if(Motor.motorstartstop == 1) /*Button is still on*/
            {
              Motor.power_disrupt = 1;             /*1 - motor running | 0 - motor stop*/
              User_Para[60] = Motor.power_disrupt; /*Flash pos at 14. Saving variable flag to flash pos 14*/
              User_Para[61] = Motor.Speed_lvl; /*Flash pos 15. Saving Speed lvl to flash pos 15*/
            }
            else if(Motor.motorstartstop == 0) /*Button is off*/
            {
              Motor.power_disrupt = 0;             /*1 - motor running | 0 - motor stop*/
              User_Para[60] = Motor.power_disrupt; /*Flash pos at 14. Saving variable flag to flash pos 14*/
              User_Para[61] = Motor.Speed_lvl; /*Flash pos 15. Saving Speed lvl to flash pos 15*/
            }
            /*Required for update flash data */
            __disable_irq();
            uint32_t *MotorCONF_Address = MotorConfig_Addr;
            uint32_t *UserConfig_Address;
            UserConfig_Address = &User_Para[0];
            XMC_FLASH_ProgramVerifyPage(MotorCONF_Address,UserConfig_Address);
            NVIC_SystemReset();
            /*------------------------------*/
#endif

          }
        }

        else if(ADC.ADC_DCLink < VDC_MIN_LIMIT)
        {
          /* Motor.error_status =  PMSM_FOC_EID_UNDER_VOLT; */
          Motor.State = DCLINK_OVER_UNDER_VOLTAGE;
          XMC_GPIO_SetOutputHigh(LED_protection); // Testing pin for ocp
          Motor.U_O_Voltage_Status = 0x01; /*Under/over voltage protection status*/
          Motor.fault_status = 1; /*Fault status for under voltage detect*/

          /* Disable gate driver. */
          if (USE_INVERTER_EN_PIN == 0)
          {
            pmsm_foc_disable_inverter(); /*4: another stop*/

#if(auto_start == ENABLED)
            /*Power disruption auto start up*/
            if(Motor.motorstartstop == 1) /*Button is still on*/
            {
              Motor.power_disrupt = 1;             /*1 - motor running | 0 - motor stop*/
              User_Para[60] = Motor.power_disrupt; /*Flash pos at 14. Saving variable flag to flash pos 14*/
              User_Para[61] = Motor.Speed_lvl; /*Flash pos 15. Saving Speed lvl to flash pos 15*/
            }
            else if(Motor.motorstartstop == 0) /*Button is off*/
            {
              Motor.power_disrupt = 0;             /*1 - motor running | 0 - motor stop*/
              User_Para[60] = Motor.power_disrupt; /*Flash pos at 14. Saving variable flag to flash pos 14*/
              User_Para[61] = Motor.Speed_lvl; /*Flash pos 15. Saving Speed lvl to flash pos 15*/
            }
            __disable_irq();
            uint32_t *MotorCONF_Address = MotorConfig_Addr;
            uint32_t *UserConfig_Address;
            UserConfig_Address = &User_Para[0];
            XMC_FLASH_ProgramVerifyPage(MotorCONF_Address,UserConfig_Address);
            NVIC_SystemReset();

            /*------------------------------*/
#endif
          }
        }

        /*ONLY if both U/O voltage fault is cleared, then it will go to motor state stop below*/
        else if ((Motor.State == DCLINK_OVER_UNDER_VOLTAGE) && (ADC.ADC_DCLink < VDC_OVER_LIMIT) && (ADC.ADC_DCLink > VDC_MIN_LIMIT))
        {
          /*Setting to motor state stop, reset button to 'STOP', speed to zero*/
          Motor.motorstartstop = 0;
          Motor.Speed = 0;
          Motor.State = STOP_MOTOR;
        }

#endif
/*----------------------------- End of over_under voltage function ---------------------------------------------------*/

#if(FAULT_RESET == ENABLED)
/* ------------------------ Auto start retry ---------------------------------------------------------- */
        if ((Motor.fault_status == 1) && (Motor.State == STOP_MOTOR))
        {
          fault_timer_counter++;                  /*fault timer counter to count up to the fault timing being set*/

          if(fault_timer_counter > fault_timing ) /*wait until fault timing before it will go for reset*/
          {
            fault_timer_counter = 0;
            Motor.fault_status = 0;               /* Clear fault status  */
            Motor.motorstartstop = 1;             /* Set GUI button to ON*/

            #if((Remote_Control == ENABLED) || (IR_Remote_Control == ENABLED))
            motor_start();
            #else
            ADC.ADC_POT = 1600; /*39%: 0.37*4096*/
            #endif

            Motor.fault_counter ++;

            if (Motor.fault_counter > FAULT_MAX_RETRY_COUNT)
            {
              Motor.fault_counter = 0;
              ADC.ADC_POT = 0;

              /*Setting to motor state stop, reset button to 'STOP', speed to zero*/
              Motor.motorstartstop = 0;
              Motor.Speed = 0;
              Motor.State = STOP_MOTOR;
            }
          }
        }
#endif
        /* -------------------------------- Power On ---------------------------------------------------------- */

#if(auto_start == ENABLED)
        if(Motor.autostart_check == 1)
        {
          Motor.autostart_check = 0;
          if(User_Para[60] == 1) /*If power disrupt is on*/
          {
            Motor.motorstartstop = 1;         /*1- Button 'ON State */
            motor_start();
          }

          if(User_Para[60] == 0) /*If power disrupt is off*/
          {
            Motor.motorstartstop = 0;         /*0- Button 'OFF State */
            Motor.Speed_lvl = User_Para[61];
          }

        }
#endif

/* -------------------------------- GUI Button Force Stop ---------------------------------------------------------- */
        if(Motor.Emergency_Stop == 1)
        {
          pmsm_foc_disable_inverter();

          /*Setting to motor state stop, reset button to 'STOP', speed to zero*/
          Motor.State = STOP_MOTOR;
          Motor.motorstartstop = 0;
          Motor.Speed = 0;

          Motor.Emergency_Stop = 0;

        }

/* ------------------------------------- Sleep Mode --------------------------------------------------------------- */
#if(MCU_Sleep_Mode == ENABLED)
        if(Motor.State == STOP_MOTOR)
        {
          Motor.sleepmode_timecounter ++;
          if(Motor.sleepmode_timecounter > sleepmode_countertime)
          {

            Motor.sleepmode_timecounter = 0;
            Motor.sleepmode_status = 0x01;

            /*Set up sleep mod pin to input to recieve signal for interrupt wake up*/
            #if(Remote_Control == ENABLED)
            XMC_GPIO_SetMode (ERU_sleepmode,XMC_GPIO_MODE_INPUT_PULL_DOWN); /*Pin2_9*/
            XMC_GPIO_EnableDigitalInput(ERU_sleepmode);
            #endif

            /*Set up sleep mod pin to input to recieve signal for interrupt wake up*/
            /*IR module output of high signal in default*/
            #if(IR_Remote_Control == ENABLED)
            XMC_GPIO_SetMode (ERU_sleepmode,XMC_GPIO_MODE_INPUT_PULL_UP); /*Pin2_9*/
            XMC_GPIO_EnableDigitalInput(ERU_sleepmode);
            #endif

            Motor.State = MCU_SLEEP;
          }
        }
        else
        {
          Motor.sleepmode_timecounter = 0;
        }
#endif  /*(MCU_Sleep_Mode == ENABLED)*/

#if(Remote_Control == ENABLED)
/*     ----------------------------- Remote Control Function --------------------------------------------------------*/
#if (UART_INTERFACE == ENABLED)
        if(User_Para[58] == 1)
        {
#endif
          /* ----------------- Button A - Start_Stop function - P0_10 ----------------- */
          if((XMC_GPIO_GetInput(button_A) == 1) && (Motor.button_hold == 0))
          {

            Motor.remote_counter ++;
            if (Motor.remote_counter > remote_counter_time)
            {
              Motor.remote_counter = 0;
              Motor.button_hold = 1;  /*When button is pressed for a 'remote_counter_time', button hold flag will be '1'*/
              Motor.start_stop_delay_flag = 1; /*To delay */
            }/*if (Motor.remote_counter > remote_counter_time)*/
          } /*End of Button A - ON/OFF function*/

          /*------------###### For motor start/stop delay after confirm button hold #########--------------------*/
          if((Motor.start_stop_delay_flag == 1) && (Motor.button_hold == 1))
          {
              /*BUTTON ON - TURNING ON THE MOTOR --------------------*/
              if (Motor.motorstartstop == 0) /*0 - Button 'OFF' State*/
              {
                Motor.delay_start_stop_counter ++;
                if (Motor.delay_start_stop_counter > start_delay_time) /* Remote function 250 ~ 1s timing |200~0.8s|*/
                {
                  Motor.start_stop_delay_flag = 0;
                  Motor.delay_start_stop_counter = 0;

                  //Motor.remote_start_stop = 1;
                  if((Motor.State != TRAP_PROTECTION) || (Motor.State != DCLINK_OVER_UNDER_VOLTAGE))
                  {
                    Motor.motorstartstop = 1;         /*1- Button 'ON State */

                    motor_start();
                  }/*if((Motor.State != TRAP_PROTECTION) || (Motor.State != DCLINK_OVER_UNDER_VOLTAGE))*/
                }/*if (Motor.delay_start_stop_counter > start_delay_time)*/
              }/*if (Motor.motorstartstop == 0)*/

              /*BUTTON OFF - TURNING OFF THE MOTOR ---------------------*/
              else if(Motor.motorstartstop == 1) /*1 - Button 'ON' State*/
              {
                Motor.delay_start_stop_counter ++;
                if (Motor.delay_start_stop_counter > stop_delay_time) /*Remote function 250 ~ 1s timing |200~0.8s|*/
                {
                  Motor.start_stop_delay_flag = 0;
                  Motor.delay_start_stop_counter = 0;
                  Motor.motorstartstop = 0;           /*0 - Button 'OFF' State*/
                  Motor.fault_counter = 0; /*Motor retry function*/

                  motor_stop();
                }/*if (Motor.delay_start_stop_counter > start_delay_time)*/
              }/*if(Motor.motorstartstop == 1)*/
          }/*if((Motor.start_stop_delay_flag == 1) && (Motor.button_hold == 1))*/

          /* ----------------- Button B - Increase Speed function - P0_11 ----------------- */
          else if((XMC_GPIO_GetInput(button_B) == 1) && (Motor.button_hold == 0))
          {
            Motor.remote_counter ++;
            if (Motor.remote_counter > remote_counter_time)
            {
              Motor.remote_counter = 0;
              Motor.button_hold = 1;  /*When button is pressed for a 'remote_counter_time', button hold flag will be '1'*/

            if(Motor.Speed_lvl < 5)        /*Check for max speed lvl.*/
            {
              Motor.Speed_lvl ++;
            }
            else
            {
              Motor.Speed_lvl = 5;
            }

            if(Motor.Speed_lvl == 1)
            {
              ADC.ADC_POT = spd_1_adc_pot;
//              Motor.motor_lastspeed = ADC.ADC_POT;
            }
            if(Motor.Speed_lvl == 2)
            {
              ADC.ADC_POT = spd_2_adc_pot;
//              Motor.motor_lastspeed = ADC.ADC_POT;
            }
            if(Motor.Speed_lvl == 3)
            {
              ADC.ADC_POT = spd_3_adc_pot;
//              Motor.motor_lastspeed = ADC.ADC_POT;
            }
            if(Motor.Speed_lvl == 4)
            {
              ADC.ADC_POT = spd_4_adc_pot;
//              Motor.motor_lastspeed = ADC.ADC_POT;
            }
            if(Motor.Speed_lvl == 5)
            {
              ADC.ADC_POT = spd_5_adc_pot;
//              Motor.motor_lastspeed = ADC.ADC_POT;
            }
//            if(Motor.Speed_lvl == 6)
//            {
//              ADC.ADC_POT = spd_6_adc_pot;
//              Motor.motor_lastspeed = ADC.ADC_POT;
//            }

              /*ADC.ADC_POT(0-4096) - GUI using this variable to control of speed*/
  //            Motor.ADC_Pot_inc_dec = 4096*percent_inc_dec;   /*Determine how much ADC value increment for each button press of given percent*/
/*================================================================================================*/
            }/*if (Motor.remote_counter > remote_counter_time)*/
          } /*End of button B - Speed increase function*/

          /* ----------------- Button C - Decrease Speed function - P0_12 ----------------- */
          else if((XMC_GPIO_GetInput(button_C) == 1) && (Motor.button_hold == 0))
          {
            Motor.remote_counter ++;
            if (Motor.remote_counter > remote_counter_time)
            {
              Motor.remote_counter = 0;
              Motor.button_hold = 1;  /*When button is pressed for a 'remote_counter_time', button hold flag will be '1'*/

              if(Motor.Speed_lvl > 1)        /*Check for min speed lvl.*/
              {
                Motor.Speed_lvl --;
              }
              if(Motor.Speed_lvl > 5)
              {
                Motor.Speed_lvl = 5;
              }
              if(Motor.Speed_lvl < 1)
              {
                Motor.Speed_lvl = 1;
              }

            if(Motor.Speed_lvl == 1)
            {
              ADC.ADC_POT = spd_1_adc_pot;
//              Motor.motor_lastspeed = ADC.ADC_POT;
            }
            if(Motor.Speed_lvl == 2)
            {
              ADC.ADC_POT = spd_2_adc_pot;
//              Motor.motor_lastspeed = ADC.ADC_POT;
            }
            if(Motor.Speed_lvl == 3)
            {
              ADC.ADC_POT = spd_3_adc_pot;
//              Motor.motor_lastspeed = ADC.ADC_POT;
            }
            if(Motor.Speed_lvl == 4)
            {
              ADC.ADC_POT = spd_4_adc_pot;
//              Motor.motor_lastspeed = ADC.ADC_POT;
            }
            if(Motor.Speed_lvl == 5)
            {
              ADC.ADC_POT = spd_5_adc_pot;
//              Motor.motor_lastspeed = ADC.ADC_POT;
            }
//            if(Motor.Speed_lvl == 6)
//            {
//              ADC.ADC_POT = spd_6_adc_pot;
//              Motor.motor_lastspeed = ADC.ADC_POT;
//            }
           }/*if (Motor.remote_counter > remote_counter_time)*/
        } /*End of button C - Speed decrease function*/

          /* ----------------- When none of the button is pressed       ----------------- */
          else if((XMC_GPIO_GetInput(button_A) == 0) && (XMC_GPIO_GetInput(button_B) == 0) && (XMC_GPIO_GetInput(button_C) == 0) && (Motor.start_stop_delay_flag == 0))
          {
            Motor.remote_counter = 0;
            Motor.button_hold = 0;
          }

#if (UART_INTERFACE == ENABLED)
        } /*if(User_Para[58] == 1)*/
#endif

#endif
/* --------------------------- End of the remote Control Function -------------------------------------------- */

/* --------------------------- Micro Inspector GUI Update Parameters -------------------------------------------- */
        if(Motor.uInspector_GUI_update_para == 1)
        {
        	uInspector_update_tune_para();
        }

	} /*End of (Motor.Non_RealTime_Counter > NON_REALTIME_RATE)*/
//	XMC_GPIO_SetOutput+Low(mcu_load);

} /*End of: pmsm_foc_misc_works_of_irq() FUNCTION*/

#if(START_UP_MODE_1 == ENABLED)



#define TIME_CPU_WAIT   (300)               // 0.2s, Time that CPU wait, x SVMPWM period.
/* Motor phase U, V and W are High_Z, motor coasting. CPU wait for some time, e.g.: for motor current decrease to zero
   * -------------------------------------------------------------------*/
void pmsm_foc_motor_coasting(void)
{
  pmsm_foc_adc_cfr_init();
  pmsm_foc_ccu8_cfr_init();

  CFR.counter1 ++;
//  if(SYSTEM_BE_IDLE)
//  {
//    CFR.counter1 = 0;
//  }

  if(CFR.counter1 > TIME_CPU_WAIT)
  {
    CFR.counter1 = 0;
    CFR.counter2 = 0;
    Motor.State = PRE_CHARGE;
  }

}


#define   TIME_1PHASE_LOW               1              /* Time that one motor phase is pulled to LOW continuously, x SVMPWM period.*/
#define   TOTAL_PRECHARGE_TIME          (400U)         /* Total pre-charge time of bootstrap capacitors. */
/* Repetitive and alternate charging sequences for bootstrap capacitors
 * to sequentially ON only one low-side switching device at one time (U->V->W->U->V->W->...)
   * -------------------------------------------------------------------*/
void pmsm_foc_cfr_precharge_bootstrap (void)
{
    CFR.counter1 ++;
    if(CFR.counter1 > TIME_1PHASE_LOW)
    {
      CFR.counter1 = 0;
      CFR.motor_phase++;
      if(CFR.motor_phase > CFR_PHASE_W)
      {
        CFR.motor_phase = CFR_PHASE_U;
      }

      CFR.counter2 ++;
      if(CFR.counter2 > TOTAL_PRECHARGE_TIME)
      {
        CFR.motor_phase = (CFR_PHASE_W + 1);

        CFR.counter1 = 0;
        CFR.counter2 = 0;
        ADC.BEMF_U = 0;
        ADC.BEMF_V = 0;
        ADC.BEMF_W = 0;

        Motor.State = CATCH_FREERUNNING;
      }
    }
}


/* Set CCU8 to pull only one motor phase LOW in next PWM cycle. For catch free-running motor
 *
 * -------------------------------------------------------------------*/
void pmsm_foc_pull_1phase_low(uint16_t phase_no)
{
  switch(phase_no)
  {
    case CFR_PHASE_U:
      break;
    case CFR_PHASE_V:
      break;
    case CFR_PHASE_W:
    break;
    default:
      break;

  }
  CCU8_MODULE->GCSS = (uint32_t)(XMC_CCU8_SHADOW_TRANSFER_SLICE_0|XMC_CCU8_SHADOW_TRANSFER_SLICE_1|XMC_CCU8_SHADOW_TRANSFER_SLICE_2);

}

#endif


// added PWM close function
void pmsm_foc_disable_inverter(void){

  XMC_GPIO_SetMode(PHASE_U_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
  XMC_GPIO_SetMode(PHASE_U_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
  XMC_GPIO_SetMode(PHASE_V_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
  XMC_GPIO_SetMode(PHASE_V_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
  XMC_GPIO_SetMode(PHASE_W_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
  XMC_GPIO_SetMode(PHASE_W_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
}

#if(MCU_Sleep_Mode == ENABLED)
#define ENABLE_DEEP_SLEEP_MODE    ENABLED   /*1. ENABLED       2. DISABLED (Disable for Normal Sleep mode)*/

/* ------------------------- MCU_Sleep_Mode ------------------------------- */
void MCU_sleep_mode(void)
{
#if ENABLE_DEEP_SLEEP_MODE == ENABLED
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;        /*Deep Sleep Mode*/
#else
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;       /*Sleep Mode*/
#endif

  /*Disabled all interrupt - NVIC_EnableIRQ() | NVIC_DisableIRQ()*/
#if(VADC_ISS_GROUP_NO == 0U)
  NVIC_DisableIRQ(VADC0_G0_1_IRQn);  /*VADC interrupt*/
#else
  NVIC_DisableIRQ(VADC0_G1_1_IRQn);  /*VADC interrupt*/
#endif
  NVIC_DisableIRQ(CCU80_0_IRQn);     /*CCU8 interrupt*/

  SCU_CLK->PWRSVCR |= SCU_CLK_PWRSVCR_FPD_Msk; /*Flash power down*/

  /*Going into the sleep mode function*/
  __DSB(); /*The Data Synchronization Barrier (DSB) instruction ensures that outstanding memory transactions complete before subsequent instructions execute. */
  __WFI(); /*Wait for interrupt, cause immediate entry to sleep mode*/
}

#endif /*End of (MCU_Sleep_Mode == ENABLED)*/

/*For Flash saving of value*/
void uInspector_update_tune_para(void)
{
  pmsm_foc_disable_inverter();

  /*Setting to motor state stop, reset button to 'STOP', speed to zero*/
  Motor.State = STOP_MOTOR;
  Motor.motorstartstop = 0;
  Motor.Speed = 0;

  User_Para[1] = PI_Speed.Kp;               /*spd kp*/
  User_Para[2] = PI_Speed.Ki;               /*spd ki*/
  User_Para[3] = PI_Speed.Scale_KpKi;       /*spd scale*/
  User_Para[4] = PI_Torque.Kp;              /*tor kp*/
  User_Para[5] = PI_Torque.Ki;              /*tor ki*/
  User_Para[6] = PI_Torque.Scale_KpKi;      /*tor scale*/
  User_Para[7] = PI_Flux.Kp;                /*flux kp*/
  User_Para[8] = PI_Flux.Ki;                /*flux ki*/
  User_Para[9] = PI_Flux.Scale_KpKi;        /*flux scale*/
  User_Para[10] = PMSM_FOC_PLL_PI.kp;           /*pll kp*/
  User_Para[11] = PMSM_FOC_PLL_PI.ki;           /*pll ki*/
  User_Para[12] = PMSM_FOC_PLL_PI.scale_kp_ki;  /*pll scale*/

  NVIC_DisableIRQ(CCU80_0_IRQn);
  NVIC_DisableIRQ(VADC0_G1_1_IRQn);  /*VADC interrupt*/
  uint32_t *MotorCONF_Address = MotorConfig_Addr;
  uint32_t *UserConfig_Address = &User_Para[0];
  XMC_FLASH_ProgramVerifyPage(MotorCONF_Address,UserConfig_Address);

  NVIC_SystemReset();
}

void user_para_init(void)
{
	/*========================= No UART GUI Flash ==================================*/
	#if(uCPROBE_GUI_no_UART == ENABLED)

	    uint32_t * MotorCONF_Addr = MotorConfig_Addr;

	    User_Para[0] = *MotorCONF_Addr;
	    /*PI Configuration*/
	    User_Para[1] = *(MotorCONF_Addr+1); /*spd kp*/
	    User_Para[2] = *(MotorCONF_Addr+2); /*spd ki*/
	    User_Para[3] = *(MotorCONF_Addr+3); /*spd scale*/
	    User_Para[4] = *(MotorCONF_Addr+4); /*tor kp*/
	    User_Para[5] = *(MotorCONF_Addr+5); /*tor ki*/
	    User_Para[6] = *(MotorCONF_Addr+6); /*tor scale*/
	    User_Para[7] = *(MotorCONF_Addr+7); /*flux kp*/
	    User_Para[8] = *(MotorCONF_Addr+8); /*flux ki*/
	    User_Para[9] = *(MotorCONF_Addr+9); /*flux scale*/
	    User_Para[10] = *(MotorCONF_Addr+10); /*pll kp*/
	    User_Para[11] = *(MotorCONF_Addr+11); /*pll ki*/
	    User_Para[12] = *(MotorCONF_Addr+12); /*pll scale*/

	#if(auto_start == ENABLED)
	    User_Para[60] = *(MotorCONF_Addr+60); /*Flag indicator for Power disruption*/
	    User_Para[61] = *(MotorCONF_Addr+61); /*Fan Speed Level*/
	#endif


	  /*Checking invalid flash value*/
	  uint32_t count;
	  uint32_t invalid_para = 0;
	  for(count = 0; count < 62; count++)
	  {
	    if(User_Para[count] > 65535U)
	    {
	      invalid_para = 1U;
	    }
	  }

	  /*Preload parameters if from new mcu. Default value is -1*/
	//    if (User_Para[0] != 1U)
	  if((invalid_para == 1U) || (User_Para[0] != 1U))
	  {
	      User_Para[0] = 1U;
	      User_Para[1] = 12000U;   	/*spd kp*/
	      User_Para[2] = 2U;       	/*spd ki*/
	      User_Para[3] = 12U;      	/*spd scale*/
	      User_Para[4] = 1544U;    	/*tor kp*/
	      User_Para[5] = 19U;     	/*tor ki*/
	      User_Para[6] = 13U;      	/*tor scale*/
	      User_Para[7] = 1544U;    	/*flux kp*/
	      User_Para[8] = 19U;     	/*flux ki*/
	      User_Para[9] = 13U;      	/*flux scale*/
	      User_Para[10] = 20U;   	/*pll kp*/
	      User_Para[11] = 1U;     	/*pll ki*/
	      User_Para[12] = 9U;     	/*pll scale*/

	      uint32_t *MotorCONF_Address = MotorConfig_Addr;
	      uint32_t *UserConfig_Address;
	      UserConfig_Address = &User_Para[0];
	      XMC_FLASH_ProgramVerifyPage(MotorCONF_Address,UserConfig_Address);
	      NVIC_SystemReset();
	  }

	#endif /*(uCPROBE_GUI == ENABLED)*/
	/*---------------------------------------------------------------------------------*/
}
