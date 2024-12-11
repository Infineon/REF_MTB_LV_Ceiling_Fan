/**
 * @file pmsm_foc_catch_free_running.c
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

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

#include <xmc_common.h>

#include "pmsm_foc_catch_free_running.h"
#include "../FOCLib/pmsm_foc_ip.h"

#include "stdlib.h"   //to use abs() function

#if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)
/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/

AMCLIB_CATCH_FREE_RUNNING_t CatchFreeRunning;

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

/* Motor control information */
extern MotorControlType Motor;
/* Motor current and current space vector. */
extern CurrentType Current;
extern Car2PolType Car2Polar;
/*********************************************************************************************************************
 * API Prototype
 ********************************************************************************************************************/


static int32_t cfr_rotor_speed;
static int32_t cfr_startup_flag;


/*********************************************************************************************************************
 * API Implementation
 ********************************************************************************************************************/

/*###* Todo: varify correct init variables for catch free-running Motor ####
   * -----------------------------------------------------*/
void PMSM_FOC_CFR_Variables_Init ()
{
  CatchFreeRunning.motor_identified_dir = AMCLIB_MOTOR_DIR_POSITIVE;
  CatchFreeRunning.cfr_startup_flag = AMCLIB_CFR_STARTUP_CFR;
  CatchFreeRunning.cfr_status = AMCLIB_CFR_STATUS_IN_PROGRESS;

  CatchFreeRunning.bemf_v_scaling_factor = BEMF_MAG_SCALING;
  CatchFreeRunning.bemf_v_scaling_factor_scale = SCALE_BEMF_MAG;

  CatchFreeRunning.cfr_bemf_thrshold = BEMF_THRESHOLD_VALUE;
  CatchFreeRunning.cfr_total_catch_time = TOTAL_CATCH_TIME;

  CatchFreeRunning.cfr_speed_LPF = 3;
  CatchFreeRunning.estimated_rotor_speed_filtered = 0;


  CatchFreeRunning.init_flag = 0;

  AMCLIB_CFR_Init(&CatchFreeRunning);
}

/* Initialize ADC & CCU8 peripherals for catch free running *
 * Motor phase U, V and W are High_Z                        *
 * ADC initialized for BEMF sensing                         *
 * -----------------------------------------------------------------------------------*/
void PMSM_FOC_CFR_Peripherals_Init (void)
{
  /* Initialize ADC for BEMF sensing */
  PMSM_FOC_VADC_BEMF_Init();

  /* Initialize CCU8 for to turn OFF all the switches */
  XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_MODULE_PHASE_U, 0);
  XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_MODULE_PHASE_U, CCU8_PERIOD_REG+1);

  XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_MODULE_PHASE_V, 0);
  XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_MODULE_PHASE_V, CCU8_PERIOD_REG+1);

  XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_MODULE_PHASE_W, 0);
  XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_MODULE_PHASE_W, CCU8_PERIOD_REG+1);

  /* Reverse Passive Level of inverter low side */
  #if(MOTOR_RUN_HIGH_SIDE == XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW)
  XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_U,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
  XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_V,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
  XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_W,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
  #else
  XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_U,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
  XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_V,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
  XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_W,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
  #endif

  /* Enable shadow transfer for slice 0,1,2,3 for CCU80 Kernel */
  XMC_CCU8_EnableShadowTransfer(CCU8_MODULE, ((uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_0 |
                                              (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_1 |
                                              (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_2));

  /* Peripheral initialization completed for catch free running */
  CatchFreeRunning.init_flag = 1U;
}

/*###* To reset peripheral settings of CCU8 and ADC for catch free-running Motor ####
   * --------------------------------------------------------------------------------*/
void PMSM_FOC_CFR_Peripherals_Reset (void)
{
  XMC_VADC_GROUP_ScanRemoveChannel(VADC_BEMF_U_GROUP,VADC_BEMF_U_CHANNEL);
  XMC_VADC_GROUP_ScanRemoveChannel(VADC_BEMF_V_GROUP,VADC_BEMF_V_CHANNEL);
  XMC_VADC_GROUP_ScanRemoveChannel(VADC_BEMF_W_GROUP,VADC_BEMF_W_CHANNEL);

  #if(MOTOR_RUN_HIGH_SIDE == XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW)
  /* Reset Passive Level of inverter low side to normal */
  XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_U,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
  XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_V,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
  XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_W,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
  #else
  XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_U,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
  XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_V,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
  XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_W,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
  #endif

  /* Enable shadow transfer for slice 0,1,2,3 for CCU80 Kernel */
  XMC_CCU8_EnableShadowTransfer(CCU8_MODULE, ((uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_0 |
                                              (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_1 |
                                              (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_2 ));

  /* Restore peripheral configurations to drive Motor */
  CatchFreeRunning.init_flag = 0U;
}


/* Reads the BEMF voltage from ADC*/
__attribute__((section(".ram_code"))) void PMSM_FOC_CFR_ReadBEMF()
{
	/* Trigger the scan request source for the conversion */
	XMC_VADC_GROUP_ScanTriggerConversion(VADC_BEMF_U_GROUP);
	XMC_VADC_GROUP_ScanTriggerConversion(VADC_BEMF_V_GROUP);

	/* Wait for conversion to be over */
	while(1U == ( XMC_VADC_GROUP_ScanIsChannelPending(VADC_BEMF_U_GROUP, VADC_BEMF_U_CHANNEL) ||
				        XMC_VADC_GROUP_ScanIsChannelPending(VADC_BEMF_V_GROUP, VADC_BEMF_V_CHANNEL) ||
                XMC_VADC_GROUP_ScanIsChannelPending(VADC_BEMF_W_GROUP, VADC_BEMF_W_CHANNEL)));

	/* Read the BEMF.BEMF referring to Motor neutral point. */
	ADC.adc_res_bemf_u = (int32_t)(VADC_BEMF_U_GROUP->RES[VADC_BEMF_U_RESULT_REG] & 0xFFFF); /*Ignore the top 16 bit*/
	ADC.adc_res_bemf_v = (int32_t)(VADC_BEMF_V_GROUP->RES[VADC_BEMF_V_RESULT_REG] & 0xFFFF);
	ADC.adc_res_bemf_w = (int32_t)(VADC_BEMF_W_GROUP->RES[VADC_BEMF_W_RESULT_REG] & 0xFFFF);

}

/*###* Catch free-running Motor ####
 *###* Execution time: ?us (O3 - Optimize most).
   * ------------------------------------------*/

volatile int32_t voltage_bemf_u;
volatile int32_t voltage_bemf_v;
volatile int32_t voltage_bemf_w;

__attribute__((section(".ram_code"))) void PMSM_FOC_MSM_CFR_Func ()
{

  /* Initialize CCU8 and ADC peripherals for catch free running */
  if(CatchFreeRunning.init_flag == 0U)
  {
    PMSM_FOC_CFR_Variables_Init();
    PMSM_FOC_CFR_Peripherals_Init();
  }

  /* Read & calculate BEMF line-line from ADC */
  PMSM_FOC_CFR_ReadBEMF();

  /* Calculate BEMF line to line ignore  X 1/3 factor here. */
  voltage_bemf_u = (int32_t)(ADC.adc_res_bemf_u << 1U) - ADC.adc_res_bemf_v - ADC.adc_res_bemf_w;
  voltage_bemf_v = (int32_t)(ADC.adc_res_bemf_v << 1U) - ADC.adc_res_bemf_u - ADC.adc_res_bemf_w;
  voltage_bemf_w = (int32_t)(ADC.adc_res_bemf_w << 1U) - ADC.adc_res_bemf_v - ADC.adc_res_bemf_u;
  /* Calculate rotor position, speed, and BEMF magnitude */
  AMCLIB_CFR_CalcPosSpeed(ADC.adc_res_bemf_u, ADC.adc_res_bemf_v, ADC.adc_res_bemf_w);

  ///////////////////////////////////////////////////////////////////////////
  /*Checking for the CFR speed by reading of adc bemf uvw.*/
  if(Motor.CFR_counter_status == 1) /*First initialize in pmsm_foc_variables_init() */
  {
    Motor.CFR_counter++;
  }
  if(Motor.CFR_counter > CFR_Spd_Status_Time)
  {
    Motor.CFR_counter_status = 0; /*Disable of the check of cfr status for speed.*/
    if ((ADC.adc_res_bemf_u > CFR_ADC_BEMF_UVW_HIGH_LIMIT) || (ADC.adc_res_bemf_v > CFR_ADC_BEMF_UVW_HIGH_LIMIT) || (ADC.adc_res_bemf_w > CFR_ADC_BEMF_UVW_HIGH_LIMIT))
    {/*High speed, higher magnitude ADC value*/
      Motor.CFR_speed_dir_high = 1;   /*Indicate of a certain higher speed threshold*/
    }
  }
  ///////////////////////////////////////////////////////////////////////////

#if(1) /*Flag to indicate CFR startup(0) or normal startup(1) && fan rotation direction,positive(1) or negative(-1)*/
  if(CatchFreeRunning.cfr_status == AMCLIB_CFR_STATUS_COMPLETED)
  {
	  if(CatchFreeRunning.cfr_startup_flag == AMCLIB_CFR_STARTUP_CFR) cfr_startup_flag=0;
	  else cfr_startup_flag=1;

    if(CatchFreeRunning.cfr_startup_flag == AMCLIB_CFR_STARTUP_CFR && CatchFreeRunning.motor_identified_dir == AMCLIB_MOTOR_DIR_POSITIVE)
    {
      /* only when the motor is free-wheeling in the same direction as to be controlled */
      PMSM_FOC_CFR_Startup(&CatchFreeRunning);
    }
    else
    {
      /*To determine of the Bootstrap brake timing*/
      if((CatchFreeRunning.motor_identified_dir == AMCLIB_MOTOR_DIR_NEGATIVE) && (Motor.CFR_speed_dir_high == 1))
      {
        Motor.CFR_brake_time = 3; /*Reverse in higher speed*/
      }
      else if((CatchFreeRunning.motor_identified_dir == AMCLIB_MOTOR_DIR_NEGATIVE) && (Motor.CFR_speed_dir_high == 0))
      {
        Motor.CFR_brake_time = 2; /*Reverse in lower speed below threshold*/
      }
      else if(Motor.CFR_speed_dir_high == 1)
      {
        Motor.CFR_brake_time = 1; /*if in non reverse but in higher speed.*/
      }
      else
      {
        Motor.CFR_brake_time = 0; /*if in non reverse but in lower speed below threshold*/
      }
      //////////////////////////////////////////////////////

      /* Fan in Reverse direction [or] in motor stop [or] near motor stop --> goes to brake */
//      XMC_GPIO_SetOutputHigh(TEST_PIN);
      PMSM_FOC_CFR_NormalStartup();
    }
  }
#endif
}

/*###* Transition from catch free-running Motor to closed-loop MET or FOC ####
 *###* Execution time: ?us (O3 - Optimize most)
   * -----------------------------------------------------------------------*/
void PMSM_FOC_CFR_Startup (AMCLIB_CATCH_FREE_RUNNING_t * const HandlePtr)
{
//  pmsm_foc_init();                //PMSM_FOC_VariablesInit();
//
//  pmsm_foc_variables_init ();                /* Init variables. */
  pmsm_phasecurrent_init();
  pmsm_foc_get_current_bias();

  cfr_rotor_speed = HandlePtr->estimated_rotor_speed_filtered;
  /* Check if measured speed in valid speed range for the Motor */
  HandlePtr->estimated_rotor_speed_filtered = MIN_MAX_LIMIT(HandlePtr->estimated_rotor_speed_filtered, SPEED_HIGH_LIMIT, 0);

  FOCOutput.Speed_by_Estimator = HandlePtr->estimated_rotor_speed_filtered;
  Car2Polar.Vref_AngleQ31 = HandlePtr->measured_bemf_angle_q31;
  Car2Polar.Vref32 = HandlePtr->measured_bemf_mag << CORDIC_SHIFT;
  //FOCInput.Ref_Speed = FOCOutput.Speed_by_Estimator;                    // Init FOC Motor reference speed.
  Motor.Ref_Speed = FOCOutput.Speed_by_Estimator;
  //Motor.Ref_Speed = 5000;

  /* PLL initialization based upon speed and rotor position detected from BEMF */
  PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31 = HandlePtr->estimated_rotor_angle_q31;    // Next rotor angle φ (1Q31) estimated from BEMF, for FOC.
  PMSM_FOC_PLL_ESTIMATOR.rotor_speed = FOCOutput.Speed_by_Estimator;              // Init estimated rotor speed of PLL Observer.
  PMSM_FOC_PLL_PI.uk = FOCOutput.Speed_by_Estimator;                  // Rotor speed ωr = PMSM_FOC_PLL_PI.Uk is needed for ωL|I| in PLL Observer of FOC.
  PMSM_FOC_PLL_PI.ik = FOCOutput.Speed_by_Estimator << (PMSM_FOC_PLL_PI.scale_kp_ki);       // Init integral terms of PI controllers.
  PI_Torque.Ik = (HandlePtr->measured_bemf_mag << PI_Torque.Scale_KpKi);

#if(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC)
  PI_Speed.Ik = Motor.Speed << PI_Speed.Scale_KpKi;
#endif

#if(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
  FOCInput.Vq = HandlePtr->measured_bemf_mag;
#endif

#if(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
//  PI_Torque.Ik = HandlePtr->voltage_bemf_mag; //missing voltage_bemf_mag****
#endif

  /* Next go to FOC close loop */
  Motor.State = FOC_CLOSED_LOOP;
//  Motor.Status = MOTOR_TRANSITION;
  Motor.Status = MOTOR_STABLE;

  /* Update SVM PWM. Execution time: 5.9us */
//  pmsm_foc_svpwm_update((Car2Polar.Vref32 >> CORDIC_SHIFT), (Car2Polar.Vref_AngleQ31 >> 16U)); /*closed loop already have this code.*/
  PMSM_FOC_CFR_Peripherals_Reset();
}

/*###* Transition from catch free-running motor to normal motor start ####
 *###* Bootstrap / brake  -> V/f -> MET -> FOC.
 *###* Execution time: ?us (O3 - Optimize most)
   * -------------------------------------------------------------------*/
void PMSM_FOC_CFR_NormalStartup (void)
{
//  PMSM_FOC_VariablesInit();
//  PMSM_FOC_VADC_PhCurrentInit();
  pmsm_foc_variables_init ();                /* Init variables. */
  pmsm_foc_get_current_bias();

  /*Resetting PI parameter back to zero*/
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

  PMSM_FOC_CFR_Peripherals_Reset();
  Motor.CFR_counter_status = 1; /*Re-enable of the check of cfr status for speed.*/

  Motor.Counter = 0U; /* cannot go to ramp up, keep motor braking. */
//  XMC_GPIO_SetOutputLow(TEST_PIN);
  Motor.State = BRAKE_BOOTSTRAP;

}

#define TIME_CPU_WAIT   (0xFFFF)               /* Time that CPU wait, x SVMPWM period.*/

/*###* motor phase U, V and W are High_Z, motor coasting. CPU wait for some time, e.g.: for motor current decrease to zero ####
 *###* Execution time: ?us (O3 - Optimize most).
   * --------------------------------------------------------------------------------------------------------------------------*/
static uint32_t idle_counter = 0U;
void PMSM_FOC_MSM_MOTOR_COASTING_Func (void)
{
  /* initialize CCU8 and ADC peripherals for catch free running */
  if(CatchFreeRunning.init_flag == 0U)
  {
    PMSM_FOC_CFR_Peripherals_Init();
  }

  //#define TH_POT_ADC                   50        /* 50. Threshold POT ADC that motor can enter or exit motor idle state */
  //#define SYSTEM_BE_IDLE               (ADC.ADC_POT < TH_POT_ADC)    /* POT ADC is too low */

  if (ADC.ADC_POT > TH_POT_ADC)/*Motor pot set above threshold*/
//  if (ADC.adc_res_pot > USER_TH_POT_ADC_START)
  {
    #if(BOOTSTRAP_PRE_CHARGE == ENABLED)
    /* Next go to bootstrap pre-charge without braking  */
    Motor.State = PRE_CHARGE;
    #else
    Motor.State = CATCH_FREERUNNING;                /* motor start command received goto CATCH_FREERUNNING */
    #endif
    idle_counter = 0U;                              /* Reset counter for the next iteration */
  }
  else
  {
    /* Wait for motor to stop completely and then go to STOP_MOTOR state     */
    idle_counter++;
    if(idle_counter >= TIME_CPU_WAIT)
    {
      Motor.State = STOP_MOTOR;  // Next, go to motor stop.
      idle_counter = 0U;
    }
  }
}

/*###* Set CCU8 to pull only one motor phase LOW in next PWM cycle. For catch free-running motor ####
 *###* Execution time: ?us (O3 - Optimize most).
   * -----------------------------------------------------------------------------------------------*/
void PMSM_FOC_Pull1PhLow (uint32_t motor_phase)
{
  switch (motor_phase)
  {
    case CFR_PHASE_U:                     // To pull motor Phase_U LOW in next PWM cycle.
      #if(MOTOR_RUN_HIGH_SIDE == XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW)
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_U,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_V,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_W,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
      #else
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_U,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_V,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_W,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
      #endif
      break;

    case CFR_PHASE_V:                     // To pull motor Phase_V LOW.
      #if(MOTOR_RUN_HIGH_SIDE == XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW)
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_U,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_V,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_W,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
      #else
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_U,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_V,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_W,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
      #endif
      break;

    case CFR_PHASE_W:                     // To pull motor Phase_W LOW.
      #if(MOTOR_RUN_HIGH_SIDE == XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW)
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_U,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_V,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_W,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
      #else
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_U,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_V,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_W,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
      #endif
      break;

    default:                        // Process for all other cases. Exit pre-charge of bootstrap capacitors. motor phase U, V and W are High_Z.
      #if(MOTOR_RUN_HIGH_SIDE == XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW)
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_U,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_V,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_W,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
      #else
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_U,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_V,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
      XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_W,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
      #endif
      /* revert back normal pwm configuration */
      XMC_GPIO_SetMode(PHASE_U_HS_PIN, PHASE_U_HS_ALT_SELECT);
      XMC_GPIO_SetMode(PHASE_V_HS_PIN, PHASE_V_HS_ALT_SELECT);
      XMC_GPIO_SetMode(PHASE_W_HS_PIN, PHASE_W_HS_ALT_SELECT);
      break;
  }

  CCU8_MODULE->GCSS = (uint32_t)(XMC_CCU8_SHADOW_TRANSFER_SLICE_0|XMC_CCU8_SHADOW_TRANSFER_SLICE_1|XMC_CCU8_SHADOW_TRANSFER_SLICE_2|XMC_CCU8_SHADOW_TRANSFER_SLICE_3);

}

#define TIME_1PHASE_LOW       (50U)           // Time that one motor phase is pulled to LOW continuously, x SVMPWM period.
#define TOTAL_PRECHARGE_TIME  (400U)         // Total pre-charge time of bootstrap capacitors.

/*###* Repetitive and alternate charging sequences for bootstrap capacitors ####
 *###* to sequentially ON only one low-side switching device at one time (U->V->W->U->V->W->...)
 *###* Execution time: ?us (O3 - Optimize most).
   * -----------------------------------------------------------------------------------------*/
void PMSM_FOC_MSM_PRE_CHARGE_Func ()
{
  static uint32_t pre_charge_time_counter = 0U;
  static uint32_t pull_one_ph_time_counter = 0U;
  static uint32_t motor_phase = CFR_PHASE_U;

  /* Initialize CCU8 and ADC peripherals for catch free running */
  if(CatchFreeRunning.init_flag == 0U)
  {
    /* Enable GPIO mode to turn on lower SW and turn off upper SW. */
    XMC_GPIO_SetMode(PHASE_U_HS_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetMode(PHASE_V_HS_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetMode(PHASE_W_HS_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);

    #if(CCU8_PASSIVE_LEVEL_OUT0 == CCU8_PASSIVE_HIGH)
    XMC_GPIO_SetOutputHigh(PHASE_U_HS_PIN);
    XMC_GPIO_SetOutputHigh(PHASE_V_HS_PIN);
    XMC_GPIO_SetOutputHigh(PHASE_W_HS_PIN);
    #else
    XMC_GPIO_SetOutputLow(PHASE_U_HS_PIN);
    XMC_GPIO_SetOutputLow(PHASE_V_HS_PIN);
    XMC_GPIO_SetOutputLow(PHASE_W_HS_PIN);
    #endif

    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_MODULE_PHASE_U, 0);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_MODULE_PHASE_U, CCU8_PERIOD_REG+1);

    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_MODULE_PHASE_V, 0);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_MODULE_PHASE_V, CCU8_PERIOD_REG+1);

    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_MODULE_PHASE_W, 0);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_MODULE_PHASE_W, CCU8_PERIOD_REG+1);

    /* Reverse Passive Level of inverter low side */
    #if(MOTOR_RUN_HIGH_SIDE == XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW)
    XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_U,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
    XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_V,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
    XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_W,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH);
    #else
    XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_U,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
    XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_V,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
    XMC_CCU8_SLICE_SetPassiveLevel(CCU8_MODULE_PHASE_W,XMC_CCU8_SLICE_OUTPUT_1,XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW);
    #endif

    /* Enable shadow transfer for slice 0,1,2,3 for CCU80 Kernel */
    XMC_CCU8_EnableShadowTransfer(CCU8_MODULE, ((uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_0 |
                                                (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_1 |
                                                (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_2));

    /* Enable inverter */
    #ifdef INVERTER_EN_PIN
    /* Enable gate driver. */
    XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, USER_INVERTER_ENABLE_LEVEL);
    #endif
  }

  /* Counter to track the total pre-charge time */
  pre_charge_time_counter ++;

  /* Check if pre-charge time is over */
  if (pre_charge_time_counter < TOTAL_PRECHARGE_TIME)
  {
    pull_one_ph_time_counter++;
    if (pull_one_ph_time_counter > TIME_1PHASE_LOW)
    {
      pull_one_ph_time_counter = 0;

      PMSM_FOC_Pull1PhLow(motor_phase);                // To pull one motor phase LOW, or all inverter switching devices OFF.

      /* Move on to the next phase */
      motor_phase++;
      if (motor_phase > CFR_PHASE_W)
      {
        motor_phase = CFR_PHASE_U;                // Return to pulling phase_U LOW.
      }
    }
  }
  else
  {
    /*Not PHASE_U/V/W, so motor phase U, V and W are High_Z. */
    motor_phase = (CFR_PHASE_W + 1U);
    PMSM_FOC_Pull1PhLow(motor_phase);
    pull_one_ph_time_counter = 0;

    /* Check if system is IDLE for too long then disable the inverter */
    if(SYSTEM_BE_IDLE)
    {
      if(pre_charge_time_counter > (100 * TOTAL_PRECHARGE_TIME))
      {
        /* Disable gate driver. */
        #ifdef INVERTER_EN_PIN
        XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, USER_INVERTER_DISABLE_LEVEL);
        #endif

        Motor.State = STOP_MOTOR;  // Next, go to motor stop.
        pre_charge_time_counter = 0; // Clear counters.
      }
    }
    else
    {
      Motor.State = CATCH_FREERUNNING;          // Next, go to catch free-running motor.
      pre_charge_time_counter = 0; // Clear counters.
    }
  }
}

#endif // End of #if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)

