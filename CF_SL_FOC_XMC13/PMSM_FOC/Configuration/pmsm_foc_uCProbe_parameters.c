/**
 * @file pmsm_foc_uCProbe_parameters.c
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
 * @file pmsm_foc_uCProbe_parameters.c
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

#include "..\PMSM_FOC\ControlModules\pmsm_foc_functions.h"

/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/

/* Global variables: */
extern ADCType ADC; /* ADC results, trigger positions. */
MotorControlType Motor; /* Motor control information */
extern CurrentType Current; /* Motor current and current space vector. */
extern Car2PolType Car2Polar;

extern TripType Trip; /* For trip / over-current detection, and protection. */
extern StallType Stall; /* For motor startup lock / fail / stall detection, and protection. */
extern HallType Hall;
extern SVMType SVM; /* SVM information, such as sector 0 ~ 5 (A ~ F) in SVM space vector hexagon. */

FOCInputType FOCInput; /* Parameters input for FOC LIB. */
FOCOutputType FOCOutput; /* Output for FOC LIB. */

void pmsm_foc_variables_init (void)
{
  Motor.Status = MOTOR_TRANSITION;				// Motor in transition mode.

  Motor.L_METPLL = L_OMEGALI;						// Motor inductance per phase
                                     //Using L_OMEGALI instead of Motor.L_METPLL in multiplication saves one MCU clock.

  Motor.Counter = 0;								// Init counters.
  Motor.Ramp_Counter = 0;
  Motor.Alignment_Counter = 0;
  Motor.Non_RealTime_Counter = 1;
  Motor.UART_Counter = 0;
  Motor.UART_Debug_Counter = 0;

  Motor.Speed = DEFAULT_SPEED_STARTUP;			// Init for V/f ramp-up.
  Motor.FG_Speed = Motor.Speed;					// Motor speed for Frequency Generation (FG) only.
  Motor.Ref_Speed = 0;


  Motor.Ramp_Up_Rate = RAMPUP_RATE << USER_RATIO_S;	// Slower ramp up and ramp down for S-curve profile.
  Motor.Ramp_Dn_Rate = RAMPDOWN_RATE << (USER_RATIO_S - 1);


  Motor.PWM_DutyCycle = 0;
  Motor.PWM_Speed_Raw = 0;
  Motor.PWM_Freq = 20;							// Init PWM frequency 20Hz.

  /*All GUI indicator status ----------------------------------------------------*/
  Motor.CCU8_Trap_Status = 0x00;        /* Not use in this current version - replaced by ACMP hardware protection*/
  Motor.U_O_Voltage_Status = 0x00;      /* Under/over voltage protection status*/
  Motor.SW_overcurrent_Status = 0x00;   /* software overcurrent protection status*/
  Motor.HW_OCP_TRAP_Status = 0x00;      /* Hardware OCP protection - ACMP*/
  Motor.Stall_detect_status = 0x00;     /* Stall Detection*/

  Motor.test_LED = 0x00;

  /*All Catch Free Run ---------------------------------------------------------*/
  Motor.CFR_counter = 0; /*Counter for checking for the CFR speed by reading of adc bemf uvw.*/
  Motor.CFR_counter_status = 1;

  /*All GUI button ON/OFF -------------------------------------------------*/
  Motor.prev_button_state = 0; /*GUI button, for the initial button press to check and stop checking once status change.*/

  /*All Emergency button --------------------------------------------------*/
  Motor.Emergency_Stop = 0; /*GUI emergency stop button*/

  /*Micro Inspector GUI update para to flash -------------------------------*/
  Motor.uInspector_GUI_update_para = 0;

  /*All Sleep Mode --------------------------------------------------------*/
  Motor.sleepmode_timecounter = 0;
  Motor.sleepmode_status = 0x00;

  /* Initial Position Detection - IPD*/
  Motor.first_kick_counter = 0;

  /*Power Limit*/
  Motor.power_limit_iq = 0;
  Motor.power_limit_flag = 0;
  Motor.POWER_RAW_SCALE_UP_value = 0;


  Car2Polar.SVM_Vref16 = 0;
  Car2Polar.SVM_Angle16 = (DEGREE_X >> 16U);		// Init Vref angle θ = X°.

  Car2Polar.Vref_AngleQ31 = Car2Polar.SVM_Angle16 << 16U;
  Car2Polar.Vref_AngleQ31_Previous = Car2Polar.Vref_AngleQ31;

  ADC.ADCTrig_Point = (uint32_t)(PERIOD_REG) >> 1;			// For ADC trigger for 2or3-shunt current sensing.

  ADC.ADC_DCLink = ADC_DCLINK_IDEAL;
  ADC.ADC_IDCLink = 0;

//  Motor.fault_status = 0; /* For fault retry counter*/

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  ADC.ADC3Trig_Point = 0;           /* For ADC 3 trigger, of single-shunt current sensing.*/
  ADC.ADC4Trig_Point = 0;           /* For ADC 4 trigger. */

  ADC.ADC_Result1 = 0;
  ADC.ADC_Result2 = 0;
  ADC.ADC_ResultTz1 = 0;
  ADC.ADC_ResultTz2 = 0;
  ADC.ADC_Result3 = 0;
  ADC.ADC_Result4 = 0;
  SVM.SVM_Flag = SVM_USE_PZV;        /* Init using SVM with Pseudo Zero Vectors (PZV). */
  ADC.Result_Flag = RESULTS_ADCTZ12;
#endif

  /* Init motor phase currents */
  Current.I_U = 0;
  Current.I_V = 0;
  Current.I_W = 0;

  SVM.PreviousSectorNo = 0;						// Init SVM sector No.

  SVM.Flag_3or2_ADC = USE_ALL_ADC;				// Init to use all (e.g.: three) ADC samplings for current reconstruction, for 2or3-shunt.

  Motor.Adjust_Para_Flag = ADJUST_NOT_DONE;

  pmsm_foc_pi_controller_init();							// Init parameters (Kp / Ki, limits) of PI controllers.

  speed_ctrl_first_time_flag = 0;

  pmsm_foc_systemparameters_init_onceonly();

  /* STALL detection */
  #if (MOTOR_STALL_DETECTION == ENABLED)
  pmsm_foc_stall_init();
  #endif



}	// End of pmsm_foc_variables_init ()

/*------------------ Initialize once in main ----------------------------------------*/
void variables_init_main (void)
{
  Motor.fault_status = 0; /*For fault retry counter*/
  Motor.fault_counter = 0; /*Motor retry function*/
  Motor.motor_lastspeed = 0; /* Initial Motor speed value should be zero*/

  Motor.power_disrupt = 0;
  Motor.Speed_lvl = 1;

#if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)
  /*Had to be declare in here as CFR normal startup will call of the variable init and will reset the flag.*/
  Motor.CFR_brake_time = 0;  /*flag status for CFR reverse direction*/
  Motor.CFR_speed_dir_high = 0; /*flag status for CFR normal controlled direction in higher speed*/
#endif
}

#if((Remote_Control == ENABLED) || (IR_Remote_Control == ENABLED))
void remote_control_variables_init (void)
{
#if(Remote_Control == ENABLED)
  Motor.remote_counter = 0; /*remote control*/
  Motor.delay_start_stop_counter = 0;
  Motor.start_stop_delay_flag = 0;
  Motor.button_hold = 0; /*remote control - indication of button hold status*/
#endif

#if(auto_start == ENABLED)
//  Motor.motor_run_status = User_Para[14]; /*Set the motor status according to saved flag status value from flash*/
  Motor.autostart_check = 1;
//  Motor.Speed_lvl = User_Para[15];

#else
  Motor.Speed_lvl = 0;
#endif

//  Motor.last_speed_lvl = 0;
  Motor.initial_autostart = 1;
  ADC.ADC_POT = 0; /*remote control, set initial adc pot value*/
} // End of remote_control_variables_init ()
#endif
/*------------------------------------------------------------------------------------*/
