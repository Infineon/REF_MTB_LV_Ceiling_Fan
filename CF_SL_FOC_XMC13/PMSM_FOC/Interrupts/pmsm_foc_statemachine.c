/**
 * @file pmsm_foc_statemachine.c
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
 * @file pmsm_foc_statemachine.c
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
#include "pmsm_foc_statemachine.h"
#include "stdlib.h"   //use abs() function
#include "..\MIDSys\pmsm_foc_stall_detection.h"
#include "..\MIDSys\pmsm_foc_catch_free_running.h"

#include "..\MIDSys\shell.h"
#include "..\MIDSys\serial.h"
#include "..\MIDSys\pmsm_foc_uart.h"
#if (uCPROBE_GUI_OSCILLOSCOPE == ENABLED)
#include "..\ProbeScope\probe_scope.h"
#endif

#if(USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)
#include "../MIDSys/pmsm_foc_rotor_ipd.h"
#endif

/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/
extern MotorControlType Motor; /* Motor control information */
extern SVMType SVM; /* SVM information, such as sector 0 ~ 5 (A ~ F) in SVM space vector hexagon. */
extern FOCOutputType FOCOutput; /* Output for FOC LIB. */
extern FOCInputType FOCInput; /* Parameters input for FOC LIB. */
extern Car2PolType Car2Polar;
extern CurrentType Current; /* Motor current and current space vector. */
extern PMSM_FOC_PLL_ESTIMATOR_t PMSM_FOC_PLL_ESTIMATOR;
extern ADCType ADC;
extern PMSM_FOC_STALL_DETECT_t stall_detect;

extern uint32_t Real_Speed_in_rpm;   /*using extern for global.*/
#if(START_UP_MODE_1 == ENABLED)
extern CFR_type CFR;
#endif
/*********************************************************************************************************************
 * LOCAL ROUTINES
 ********************************************************************************************************************/
#if (USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)
__STATIC_INLINE void PMSM_FOC_MSM_ROTOR_IPD_Func(void);
__STATIC_INLINE void PMSM_FOC_MSM_TRANSITION_Func(void);
#endif

__attribute__((section(".ram_code"))) void pmsm_foc_controlloop_isr(void);

__attribute__((section(".ram_code"))) void pmsm_foc_controlloop_isr(void)
{
#if(UART_INTERFACE == ENABLED)
  if((Motor.motorstartstop == 0) && (Real_Speed_in_rpm<= 20)) /*Button off, then can update uart*/
//  if(Motor.motorstartstop == 0) /*Button off, then can update uart*/
  {
	  uart_shell_state_machine();
  }
#endif

#if (MCU_Load_time_test == ENABLED)
    XMC_GPIO_SetOutputHigh(mcu_load);
#endif

  switch (Motor.State)
    {
#if((MY_FOC_CONTROL_SCHEME != SPEED_CONTROLLED_VF_ONLY))
      case FOC_CLOSED_LOOP:

  #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
        if(SVM.SVM_Flag == SVM_USE_PZV)
        {
          pmsm_foc_adc34_triggersetting(&ADC);
          ADC.Result_Flag = RESULTS_ADCTZ12;
        }
        else
        {
          /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
          ADC.Result_Flag = RESULTS_STANDARD_SVM;
        }
  #endif

  #if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC) ||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC))
        pmsm_foc_speed_controller();                  // FOC LIB. /*testing*/
        //pmsm_foc_torque_controller();
  #elif(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
        pmsm_foc_torque_controller();
  #elif(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
        pmsm_foc_vq_controller();
  #endif

        /*Stall Dectection*/
        #if (MOTOR_STALL_DETECTION == ENABLED)
#if(IP_STALL)
        PMSM_FOC_StallDetection(FOCOutput.I_q, Car2Polar.Torque_Vq, FOCOutput.current_i_mag_filtered);
        if(stall_detect.motor_stall_status == 1)
        {
          Motor.error_status |= PMSM_FOC_EID_STALL;
          Motor.State = PMSM_FOC_MSM_ERROR;
        }
#else
        PMSM_FOC_Stall_Detection();
        if(PMSM_FOC_STALL_DETECTION.Stall_detected == 1)
        {
          Motor.error_status |= PMSM_FOC_EID_STALL;
          Motor.State = PMSM_FOC_MSM_ERROR;
        }
#endif

        #endif

        /* Update SVM PWM. Execution time: 5.9us */
        pmsm_foc_svpwm_update((Car2Polar.Vref32 >> CORDIC_SHIFT), (Car2Polar.Vref_AngleQ31 >> 16U));

        /* Miscellaneous works in FOC, such as ramp up, speed adjustment, stop motor, etc. Execution time: 2.8us*/
        pmsm_foc_misc_works_of_foc ();

        /* Record SVM sector information. */
        FOCOutput.New_SVM_SectorNo = SVM.CurrentSectorNo;

        break;
#endif

#if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_ONLY)|| (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC))
      case  VFOPENLOOP_RAMP_UP:

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      if(SVM.SVM_Flag == SVM_USE_PZV)
      {
        pmsm_foc_adc34_triggersetting(&ADC);
        ADC.Result_Flag = RESULTS_ADCTZ12;
      }
      else
      {
        /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
        ADC.Result_Flag = RESULTS_STANDARD_SVM;
      }
#endif
      pmsm_foc_vf_openloop_rampup();
      /*Stall Dectection*/
      #if (MOTOR_STALL_DETECTION == ENABLED)

#if(IP_STALL)
        PMSM_FOC_StallDetection(FOCOutput.I_q, Car2Polar.Torque_Vq, FOCOutput.current_i_mag_filtered);
        if(stall_detect.motor_stall_status == 1)
        {
          Motor.error_status |= PMSM_FOC_EID_STALL;
          Motor.State = PMSM_FOC_MSM_ERROR;
        }
#else
        PMSM_FOC_Stall_Detection();
        if(PMSM_FOC_STALL_DETECTION.Stall_detected == 1)
        {
          Motor.error_status |= PMSM_FOC_EID_STALL;
          Motor.State = PMSM_FOC_MSM_ERROR;
        }
#endif

      #endif

        break;
#endif
        #if(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC)
      case MET_FOC:
  #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
        if(SVM.SVM_Flag == SVM_USE_PZV)
        {
          pmsm_foc_adc34_triggersetting(&ADC);
          ADC.Result_Flag = RESULTS_ADCTZ12;
        }
        else
        {
          /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
          ADC.Result_Flag = RESULTS_STANDARD_SVM;
        }
  #endif
        Motor.Status = pmsm_foc_vf_smooth_transition_foc();
        pmsm_foc_misc_works_of_met();

          break;
#endif
        case BRAKE_BOOTSTRAP:
          /* Brake the motor before motor startup. Charge gate driver bootstrap capacitors (if any)*/
          pmsm_foc_brake_motor_bootstrap_charge();
          break;

        case STOP_MOTOR:
  #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
          if(SVM.SVM_Flag == SVM_USE_PZV)
          {
            pmsm_foc_adc34_triggersetting(&ADC);
            ADC.Result_Flag = RESULTS_ADCTZ12;
          }
          else
          {
            /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
            ADC.Result_Flag = RESULTS_STANDARD_SVM;
          }
  #endif
          pmsm_foc_stop_motor ();
          break;

#if(MCU_Sleep_Mode == ENABLED)
        case MCU_SLEEP:
          MCU_sleep_mode();
          break;
#endif

#if(CATCH_FREE_RUNNING_WITH_BEMF== ENABLED)
        case CATCH_FREERUNNING:
          PMSM_FOC_MSM_CFR_Func ();
          break;

        case PRE_CHARGE:
          PMSM_FOC_MSM_PRE_CHARGE_Func();
          break;

        case MOTOR_COASTING:
          PMSM_FOC_MSM_MOTOR_COASTING_Func();
        break;
#endif

#if (USER_ROTOR_IPD_METHOD == ROTOR_IPD_PRE_ALIGNMENT)
        case PRE_POSITIONING:
  #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
          if(SVM.SVM_Flag == SVM_USE_PZV)
          {
            pmsm_foc_adc34_triggersetting(&ADC);
            ADC.Result_Flag = RESULTS_ADCTZ12;
          }
          else
          {
            /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
            ADC.Result_Flag = RESULTS_STANDARD_SVM;
          }
  #endif
          pmsm_foc_directfocrotor_pre_positioning ();
          break;
#endif

#if (USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)
        case PMSM_FOC_MSM_ROTOR_IPD:
        	PMSM_FOC_MSM_ROTOR_IPD_Func();
        	break;
#endif

        case PMSM_FOC_MSM_TRANSITION: /*Added for IPD. Default FW have no transition state*/
        	PMSM_FOC_MSM_TRANSITION_Func();
        	break;

#if(START_UP_MODE_1 == ENABLED)
        case MOTOR_COASTING:
          pmsm_foc_motor_coasting();
          break;
        case PRE_CHARGE:
          pmsm_foc_cfr_precharge_bootstrap();
          break;
        case CATCH_FREERUNNING:
          pmsm_foc_readbemf_voltage(ADC.BEMF_U,ADC.BEMF_V, ADC.BEMF_W,&CFR);
          pmsm_foc_calculate_rotor_mag_angle(&CFR);
          break;
#endif
        default:
          /* For trap protection if CCU8_TRAP_ENABLE (CCU8 TRAP functionality enabled)*/
          pmsm_foc_error_handling ();

          break;
      }

    /*
     * Miscellaneous works in CCU80_0_IRQHandler, such as tasks that don't need real-time computing.
     * Execution time: 1.65us
     */
     pmsm_foc_misc_works_of_irq ();

//#if (Test_point == ENABLED)
////    XMC_GPIO_ToggleOutput(TEST_PIN);
//    XMC_GPIO_PORT0->OMR = 0x10001U << 13;
//#endif

      #if (uCPROBE_GUI_OSCILLOSCOPE == ENABLED)
      ProbeScope_Sampling();
      #endif

#if (MCU_Load_time_test == ENABLED)
     XMC_GPIO_SetOutputLow(mcu_load);
#endif


//added phase over current protection
   /* ------------------------------------ SW Over Current Protection -------------------------------------- */
       if((abs(Current.I_U)> I_OCP_SW)|(abs(Current.I_V)> I_OCP_SW)|(abs(Current.I_W)> I_OCP_SW))
       {
    	 // stop motor immediately
         pmsm_foc_disable_inverter();
         XMC_GPIO_SetOutputHigh(LED_protection); /* Testing pin for ocp */
         Motor.SW_overcurrent_Status = 0x01;
         Motor.fault_status = 1;                /* For reset function */

         /*Setting to motor state stop, reset button to 'STOP', speed to zero*/
         Motor.motorstartstop = 0;
         Motor.Speed = 0;
         Motor.State = STOP_MOTOR;
       }
   /* ------------------------------------ SW Over Current Protection -------------------------------------- */
}

/**********************************************************************************************************************
Rotor initial position detection state machine function
-----------------------------------------------------------------------------------------------------------------------*/
#if(USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)
/**
 * @brief This function is called when motor control state machine is set to PMSM_FOC_MSM_ROTOR_IPD.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_INLINE void PMSM_FOC_MSM_ROTOR_IPD_Func(void)
{
#if(IPD_TEST_FLAG == ENABLED)
    if(ipd_cnt >= IPD_TEST_CNT)
    {
      PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_STOP_MOTOR;
    }
    else
    {
      if(PMSM_FOC_Rotor_IPD.status == PMSM_FOC_ROTOR_IPD_STATUS_UNKNOWN)
      {
        PMSM_FOC_Rotor_IPD_Init();
      }
      else if(PMSM_FOC_Rotor_IPD.status == PMSM_FOC_ROTOR_IPD_STATUS_COMPLETED)
      {

        ipd_data[ipd_cnt] = PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31;
        ipd_cnt++;
        if(ipd_cnt < IPD_TEST_CNT)
        {
          /* Reset status to unknown for next iteration */
          PMSM_FOC_Rotor_IPD.status = PMSM_FOC_ROTOR_IPD_STATUS_UNKNOWN;
          PMSM_FOC_Rotor_IPD.ind_sense_pat_index = 0U;
          PMSM_FOC_Rotor_IPD.read_current_enable = 0U;

          /* restart IPD to repeated test the position estimation */
          PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_ROTOR_IPD;

          /* Disable Global Start Control CCU80 */
          XMC_SCU_SetCcuTriggerLow(PMSM_FOC_CCU8_CCU4_SYNC_START);

          #if(IPD_TEST_WITH_FIRST_KICK)

          /* Revert back CCU8 configuration for normal operation */
          pmsm_foc_ccu8_init();

          /* Revert back VADC configuration  */
          PMSM_FOC_VADC_PhCurrentInit();

          /* go to first kick to see the movement of rotor */
          PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_TRANSITION;
          #endif

          /* Start the PWM synchronously */
          XMC_SCU_SetCcuTriggerHigh(PMSM_FOC_CCU8_CCU4_SYNC_START);
        }
        else
        {
          /* Disable Global Start Control CCU80 */
          XMC_SCU_SetCcuTriggerLow(PMSM_FOC_CCU8_CCU4_SYNC_START);

          /* Revert back CCU8 configuration for normal operation */
          pmsm_foc_ccu8_init();

          /* Revert back VADC configuration  */
          PMSM_FOC_VADC_PhCurrentInit();

          /* Reset status to unknown for next iteration */
          PMSM_FOC_Rotor_IPD.status = PMSM_FOC_ROTOR_IPD_STATUS_UNKNOWN;
          PMSM_FOC_Rotor_IPD.ind_sense_pat_index = 0U;
          PMSM_FOC_Rotor_IPD.read_current_enable = 0U;
          /* Start the PWM synchronously */
          XMC_SCU_SetCcuTriggerHigh(PMSM_FOC_CCU8_CCU4_SYNC_START);

          PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_STOP_MOTOR;
        }
      }
      else
      {
        /* Rotor initial position identification is in progress */
        PMSM_FOC_Rotor_Init_Pos_Detect();
      }
    }
#else

	if(PMSM_FOC_Rotor_IPD.status == PMSM_FOC_ROTOR_IPD_STATUS_UNKNOWN)
	{
//		XMC_GPIO_SetOutputHigh(TEST_PIN);
		PMSM_FOC_Rotor_IPD_Init();
#if (Test_point == ENABLED)
//    XMC_GPIO_ToggleOutput(TEST_PIN);
    XMC_GPIO_PORT0->OMR = 0x10001U << 13;
#endif
	}
	else if(PMSM_FOC_Rotor_IPD.status == PMSM_FOC_ROTOR_IPD_STATUS_COMPLETED)
	{
	   /* Disable Global Start Control CCU80 */
	  XMC_SCU_SetCcuTriggerLow(PMSM_FOC_CCU8_CCU4_SYNC_START);

		/* Revert back CCU8 configuration for normal operation */
	  	pmsm_foc_ccu8_init();

		/* Revert back VADC configuration  */
	  	pmsm_phasecurrent_init();

		/* Reset status to unknown for next iteration */
		PMSM_FOC_Rotor_IPD.status = PMSM_FOC_ROTOR_IPD_STATUS_UNKNOWN;
		PMSM_FOC_Rotor_IPD.ind_sense_pat_index = 0U;
		PMSM_FOC_Rotor_IPD.read_current_enable = 0U;

#if (Test_point == ENABLED)
//    XMC_GPIO_ToggleOutput(TEST_PIN);
    XMC_GPIO_PORT0->OMR = 0x10001U << 13;
#endif

		Motor.test_LED = 0x01;

	    /* Next go to transition state to enter into close loop */
	    Motor.State = PMSM_FOC_MSM_TRANSITION;

	  /* Start the PWM synchronously */
		CCUx_SynStart();
	}
	else
	{
		/* Rotor initial position identification is in progress */
		PMSM_FOC_Rotor_Init_Pos_Detect();
//#if (Test_point == ENABLED)
////    XMC_GPIO_ToggleOutput(TEST_PIN);
//    XMC_GPIO_PORT0->OMR = 0x10001U << 13;
//#endif
	}
#endif  /* #if(IPD_TEST_FLAG == ENABLED) #else*/
}

/**********************************************************************************************************************
Transition - V/F to close loop function
-----------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief This function is called when motor control state machine is set to PMSM_FOC_MSM_TRANSITION.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_INLINE void PMSM_FOC_MSM_TRANSITION_Func(void)
{
#if (Test_point == ENABLED)
//    XMC_GPIO_ToggleOutput(TEST_PIN);
//    XMC_GPIO_PORT0->OMR = 0x10001U << 13;
    XMC_GPIO_SetOutputHigh(TEST_PIN);
	XMC_GPIO_PORT0->OMR = (uint32_t)0x1U << 13;
//	XMC_GPIO_SetOutputLow(TEST_PIN);
//	XMC_GPIO_PORT0->OMR = 0x10000U << 13;
#endif

	#if(USER_ROTOR_IPD_METHOD != ROTOR_IPD_NONE)
	if(Motor.first_kick_counter == 0U)
	{
	  /* If IPD is inductive sensing then PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31 will be updated by IPD algorithm. */
		/* First voltage vector to apply with higher fix amplitude - First kick*/

//	    if(PMSM_FOC_CTRL.rotation_dir == DIRECTION_DEC)
//	    {
//	      PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31 = -PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31;
//	    }

		Car2Polar.SVM_Angle16 = (uint16_t)((PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31 + PMSM_FOC_ANGLE_090_DEGREE_Q31)>>16);


      /* Calculate voltage from SVPWM */
      #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
	    /* !!! In transistion state, the FSM loop execution time is too short and the CC83 reg is changed in PMSM_FOC_SVPWM_Update_Singleshunt
	     * before the 2Tz period finishes, which will cause wrong ADC triggering sequence after this !!!*/
	    //PMSM_FOC_SVPWM_Update_Singleshunt(FIRST_KICK_VALUE, PMSM_FOC_OUTPUT.svm_angle_16);
	    PMSM_FOC_SVPWM_Update_Singleshunt_Trans(FIRST_KICK_VALUE, Car2Polar.SVM_Angle16);
      #else
      PMSM_FOC_SVPWM_Update(FIRST_KICK_VALUE, PMSM_FOC_OUTPUT.svm_angle_16);
      #endif
      /* !!to make sure SVPWM.previous_sector_num == SVPWM.current_sector_num in first kick !!
       * this solved current sampling error in the transition from first kick to closed-loop FOC */
      SVM.PreviousSectorNo = SVM.CurrentSectorNo;           /* Record sector information of last PWM cycle. */
      FOCOutput.Previous_SVM_SectorNo = SVM.CurrentSectorNo;
      Motor.first_kick_counter++;
	}
	else
	{
		/* Minimum 2 counts are required to synchronize SVPWM & ADC readings to enter into close loop */
		if(Motor.first_kick_counter >= FIRST_KICK_TIME)
		{

#if (Test_point == ENABLED)
//			XMC_GPIO_ToggleOutput(TEST_PIN);
//			XMC_GPIO_PORT0->OMR = 0x10001U << 13;
//			XMC_GPIO_SetOutputHigh(TEST_PIN);
//			XMC_GPIO_PORT0->OMR = (uint32_t)0x1U << 13;
//			XMC_GPIO_SetOutputLow(TEST_PIN);
			XMC_GPIO_PORT0->OMR = 0x10000U << 13;
#endif
			Motor.State = FOC_CLOSED_LOOP;
//			Motor.Ref_Speed = FOCOutput.Speed_by_Estimator;
//			Motor.Ref_Speed = 2000U;
			Motor.first_kick_counter = 0U;

			/* update necessary variables for closed loop control */
			PI_Torque.Uk = FIRST_KICK_VALUE;
			PI_Torque.Ik = FIRST_KICK_VALUE;
			FOCOutput.New_SVM_SectorNo = SVM.CurrentSectorNo;
		}
		else
		{
		  Motor.first_kick_counter++;
		}
	}

  #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
	//pmsm_foc_current_reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);
  #else
  /* Read phase current from ADC */
  PMSM_FOC_VADC_GetPhasecurrent(SVPWM.previous_sector_num, SVPWM.current_sector_num, &ADC);
  /* Remove offset and target scaling */
  PMSM_FOC_CurrentReconstruction(ADC.adc_res_iu, ADC.adc_res_iv, ADC.adc_res_iw, &PMSM_FOC_INPUT);
  #endif

	#endif    /* #if(USER_ROTOR_IPD_METHOD != ROTOR_IPD_NONE) */
#if (Test_point == ENABLED)
//    XMC_GPIO_ToggleOutput(TEST_PIN);
//    XMC_GPIO_PORT0->OMR = 0x10001U << 13;
#endif
}
#endif /*#if(USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)*/
