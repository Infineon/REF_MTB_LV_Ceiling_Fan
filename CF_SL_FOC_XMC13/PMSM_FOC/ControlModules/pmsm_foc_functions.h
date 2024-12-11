/**
 * @file pmsm_foc_functions.h
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
 * @file pmsm_foc_functions.h
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

#ifndef PMSM_FOC_CONTROLMODULES_PMSM_FOC_FUNCTIONS_H
#define PMSM_FOC_CONTROLMODULES_PMSM_FOC_FUNCTIONS_H
#define MATH                           ((MATH_Type               *) MATH_BASE)
/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "cybsp.h"
#include "cy_utils.h"
#include <XMC1300.h>											// SFR declarations of the selected device
#include "..\MCUInit\wdt.h"
#include "..\MCUInit\uart.h"
#include "..\MIDSys\pmsm_foc_current_threeshunt.h"
#include "..\MIDSys\pmsm_foc_current_singleshunt.h"
#include "pmsm_foc_pi.h"
#include "..\MIDSys\pmsm_foc_pwmsvm.h"
#include "..\Configuration\pmsm_foc_motor_parameters.h"   /*For Startup Torque*/
#if(START_UP_MODE_1 == ENABLED)
  #include "..\MIDSys\pmsm_foc_catch_free_running.h"
#endif

typedef struct FOCInputType
{
  int32_t Phase_L;
  int32_t Phase_R;
  uint16_t Phase_L_Scale;
  uint16_t CCU8_Period;
  uint16_t Res_Inc;
  int16_t SVM_Scale;
  uint16_t LPF_N_BEMF;
  uint32_t Threshold;
  uint16_t Threshold_LOW;
  uint16_t Threshold_HIGH;
  uint16_t Flag_State;
  uint16_t overcurrent_factor;
  int32_t BEMF1;
  uint32_t BEMF2;
  uint16_t SVM_5_Segment_Flag;
  uint32_t Vref32;
  int32_t Vref_AngleQ31;
  int32_t I_U;
  int32_t I_V;
  int32_t I_W;
  int32_t Ref_Speed;
  int32_t Vq_Flag;
  int32_t Vq;
  int32_t Ref_Id;
  int32_t Ref_Iq;
  uint16_t Iq_PI_Flag;
  int32_t RotorAngleQ31;
  int32_t RotorSpeed_In;

  int32_t Single_Shunt_Flag;
} FOCInputType;

typedef struct FOCOutputType
{
  int32_t I_Alpha_1Q31;
  int32_t I_Beta_1Q31;
  int32_t I_d;
  int32_t I_q;
  int32_t Speed_by_Estimator;
  int32_t Rotor_PositionQ31;
  uint16_t Previous_SVM_SectorNo;
  uint16_t New_SVM_SectorNo;

  uint32_t Vref32;
  int32_t Vref_AngleQ31;

  uint16_t SVM_U_CR1S;
  uint16_t SVM_V_CR1S;
  uint16_t SVM_W_CR1S;

  int32_t Debug_1;

  #if (MOTOR_STALL_DETECTION == ENABLED)
  int32_t current_i_mag;
  int32_t current_i_mag_filtered;
  #endif

} FOCOutputType;


typedef struct CurrentType
{
  int32_t I_U; /* Current of motor phase U, Iu */
  int32_t I_V; /* Current of motor phase V, Iv */
  int32_t I_W; /* Current of motor phase W, Iw */

  uint32_t I_Mag; /* |I|, magnitude of current space vector */

  int32_t I_Speed; /* ωi, current space vector speed */
} CurrentType;


typedef struct ClarkeTransformType
{
  int32_t I_Alpha_1Q31; /* Iα (1Q31), Alpha value of current space vector */
  int32_t I_Beta_1Q31; /* Iβ (1Q31) */

} ClarkeTransformType;

typedef struct ParkTransformType
{
  int32_t Id; /* Id */
  int32_t Iq; /* Iq */
} ParkTransformType;

typedef struct Car2PolType
{
  int32_t Flux_Vd; /* Vd */
  int32_t Torque_Vq; /* Vq */

  uint32_t Vref32;
  int32_t Vref_AngleQ31;

  uint32_t Vref32_Previous; /* |Vref| of last PWM cycle */
  int32_t Vref_AngleQ31_Previous; /* Angle θ of last PWM cycle */

  uint16_t SVM_Vref16; /* |Vref|, Magnitude (1Q15) of reference vector (for SVM) */
  uint16_t SVM_Angle16; /* Angle θ (16-bit) of reference vector. 0 ~ 2^16 represent electrical angle 0° ~ 360° */
} Car2PolType;

typedef struct MotorControlType
{
  uint32_t L_METPLL; /* Motor inductance per phase, used in ωL|I| of MET and FOC PLL observer */

  int32_t Ref_Speed; /* Rotor reference speed ωref, e.g.: determined by POT ADC or PWM duty cycle */
  uint32_t Speed; /* Motor shaft speed of V/f, MET (and FOC) */

  int32_t Speed_by_POT_PWM; /* Target motor speed set by POT ADC, or PWM duty cycle */

  uint32_t PWM_DutyCycle; /* Duty cycle of the PWM for speed adjustment */
  uint32_t PWM_Period; /* Period of the PWM (10kHz ~ 50kHz) for speed adjustment, almost a constant value */
  int32_t PWM_Speed_Raw; /* PWM-set speed, raw data */
  uint32_t PWM_Freq; /* Frequency of PWM. */

  int32_t Ramp_Up_Rate; /* Motor speed ramp up rate */
  int32_t Ramp_Dn_Rate; /* Motor speed ramp down rate */

  uint32_t State; /* Motor state (e.g.: V/f, MET, FOC) */
  uint16_t Rotation_Dir; /* Rotation direction of motor (rotor angle increasing, or decreasing) */
  uint16_t Status; /* Flag to indicate if motor is in transition (MOTOR_TRANSITION) or stable (MOTOR_STABLE) */
  uint16_t motorstartstop;
  uint16_t user_control_speed;
  uint16_t Emergency_Stop;
  uint16_t Control_Mode;

  uint32_t Adjust_Para_Flag; /*
                              * Flag to indicate parameter scheduling status,
                              * e.g.: for parameter adjust of PI controllers in FOC steady state.
                              */

  uint32_t Counter; /* General purpose counter */
  uint32_t Ramp_Counter; /* General purpose counter, or counter for motor speed ramp up/down. */
  uint32_t Alignment_Counter; /* Counter for rotor initial positioning / alignment in V/f */
  uint32_t Non_RealTime_Counter; /* Counter for tasks that don't need real-time computing */

  /* --------------------------- Micro Inspector GUI Update Parameters -------------------------------------------- */
  uint16_t uInspector_GUI_update_para;

  /*Remote_Control: --------------------------------------------------------------*/
  uint32_t remote_start_stop;         /* Remote_Control: start stop status*/
  uint32_t button_hold;               /* Remote_Control: indicate button hold status*/
  uint32_t remote_counter;            /* Remote_Control: counter when pressed*/
  uint32_t delay_start_stop_counter;  /* Delay timing after button pressed*/
  uint32_t start_stop_delay_flag;     /* Delay flag for start stop.*/
  uint32_t ADC_Pot_inc_dec;           /* Remote_Control: ADC_pot value increase_decrease per button pressed*/
  uint32_t Speed_lvl;                 /* Remote_Control: Different speed level for different speed*/
  uint32_t last_speed_lvl;
  uint32_t initial_autostart;         /* Remote_Control: indicate the very start button press status for start of program*/
  uint32_t initial_variable_init;
  uint32_t prev_button_state;
  uint32_t motor_lastspeed;           /* Saving of Motor last speed value in here */

  uint32_t autostart_check;           /*Just to check once*/
  uint32_t power_disrupt;             /*Flag indicator for Power disruption*/

  /* Sleep Mode */
  uint32_t sleepmode_timecounter;         /*Sleep mode cycle count in stop state before going sleep mode*/
  uint32_t sleepmode_status;              /*GUI indicator status*/

  uint32_t FW_Counter; /* Counter for Flux Weakening (FW) */
  uint32_t UART_Counter; /* Counter for UART communication */
  uint32_t UART_Debug_Counter;

  int32_t FG_Speed; /* Motor speed for Frequency Generation (FG) only */

  uint16_t CCU8_Trap_Status;
  uint16_t U_O_Voltage_Status;    /*Under/over voltage protection status*/
  uint16_t SW_overcurrent_Status; /*software overcurrent protection status*/
  uint16_t HW_OCP_TRAP_Status;    /* Hardware OCP protection - ACMP*/
  uint16_t Stall_detect_status;   /* Stall Detection*/

  /* Auto Retry */
  uint32_t fault_status;          /* All Fault status flag*/
  uint32_t fault_counter;         /* Number of Retry counter*/

  /* Catch Free Run - CFR */
  uint16_t CFR_exit_status;       /*Exiting of CFR startup status: 1- exit| 0-still in CFR startup*/
  uint16_t CFR_brake_time;        /*flag status for different CFR situation: high reverse|low reverse|high non-reverse/rev|low non-reverse/rev*/
  uint16_t CFR_speed_dir_high;    /*flag status for CFR in running higher speed*/

  uint16_t CFR_counter;           /*Timer counter to check for cfr speed status*/
  uint16_t CFR_counter_status;    /*Flag to Enable/Disable of the check of cfr status for speed.*/
  /* ------------------- */

  uint32_t USER_DEFINE_BOOTSTRAP_BRAKE_TIME; /*Bootstrap Brake time */

  uint16_t UART_Data; /* Data received via UART */

  int32_t error_status; /*!< Error status to identify which error has occurred */
  uint32_t braking_counter; /*!< General purpose counter */

  uint16_t test_LED;

  /* Initial Position Detection - IPD*/
  uint32_t first_kick_counter;                     /*!<  Counter for first kick after rotor alignment or IPD */

  /*Power Limit*/
  uint32_t power_limit_iq;
  uint32_t power_limit_flag;
  uint32_t POWER_RAW_SCALE_UP_value;

} MotorControlType;

typedef struct StallType /* For motor startup lock / fail / stall detection and protection. */
{
  uint32_t Counter; /* Counter for detection of motor startup lock / fail / stall */

} StallType;


typedef struct TripType /* For trip / over-current detection and protection */
{
  uint32_t Counter; /* Counter for trip / over-current protection */

  int32_t ADC_Ave_DC_Link; /* Average of ADC value (with LPF) for dc link current, to detect trip / over-current. */
} TripType;

typedef struct OverUnderVoltType /* For over/under-voltage detection and protection */
{
  uint32_t Counter; /* Counter for over/under-voltage detection */
  uint32_t Under_OverVoltage;
} OverUnderVoltType;

typedef struct HallType /* For Hall signal processing */
{
  int32_t Speed; /* Rotor speed obtained from Hall */
  int32_t Speed_rpm; /* Rotor speed obtained from Hall, in rpm */
  int32_t Rotor_AngleQ31; /* Estimated rotor angle (1Q23 << 8) from Hall */
  uint16_t Flag; /* Flag to indicate if one Hall event has occurred */
  uint32_t Event_Counter; /* Counter for Hall events. */

  uint32_t Stall_Counter; /* Counter for Hall stall detection */
  uint32_t Restart_Counter; /* Counter for retry times to start motor if stall has been detected by Hall */
  uint32_t Rst_Restart_Counter; /* To reset Hall_Restart_Counter if no motor stall for certain time (e.g.: 20s) */
} HallType;

typedef enum StateMachine
{
	FOC_CLOSED_LOOP = 0,
	BRAKE_BOOTSTRAP,
	STOP_MOTOR,
	VFOPENLOOP_RAMP_UP,
	MET_FOC,
	PRE_POSITIONING,
	DCLINK_OVER_UNDER_VOLTAGE,
	MCU_SLEEP,
	TRAP_PROTECTION,
	CFR_PRE_CHARGE,
	CATCH_FREERUNNING,
	MOTOR_COASTING,
	PRE_CHARGE,
	PMSM_FOC_MSM_ERROR,
	PMSM_FOC_MSM_ROTOR_IPD,
	PMSM_FOC_MSM_TRANSITION,
}StateMachine;


typedef enum StatusFlag
{
  MOTOR_STABLE,
  MOTOR_TRANSITION,
}StatusFlag;

extern FOCOutputType FOCOutput;                 // Output for FOC LIB.
extern SVMType SVM;                          // SVM information, such as sector 0 ~ 5 (A ~ F) in SVM space vector hexagon.
extern uint16_t speed_ctrl_first_time_flag;
  /**
   *  @brief PI PLL Control parameters
   */
  typedef struct PMSM_FOC_PLL_PI
  {
    int32_t error;                              /*!< PI error signal (reference value � feedback value), error[k] */
    int32_t uk;                                 /*!< PI output U[k] */
    int32_t ik;                                 /*!< Integral result I[k] */
    int32_t ik_limit_min;                       /*!< Integral buffer limit  - minimum */
    int32_t ik_limit_max;                       /*!< Integral buffer limit  - maximum */
    int32_t uk_limit_min;                       /*!< PI output limit - minimum */
    int32_t uk_limit_max;                       /*!< PI output limit - maximum */
    int32_t uk_limit_max_scaled;                /*!< Internal variable - PI output limit scaled - maximum */
    int32_t uk_limit_min_scaled;                /*!< Internal variable - PI output limit scaled - minimum */
    uint16_t kp;                                /*!< Proportional gain Kp */
    uint16_t ki;                                /*!< Integral gain Ki */
    int16_t scale_kp_ki;                        /*!< Scale-up Kp and Ki by 2^Scale_KpKi */

  } PMSM_FOC_PLL_PI_t;

  /**
   *  @brief PLL Estimator parameters
   */
  typedef struct PMSM_FOC_PLL_ESTIMATOR
  {
    int32_t phase_inductance_Ls;                /*!< Motor stator phase inductance scaled */
    int32_t bemf_1;                             /*!< PLL observer internal signal */
    int32_t bemf_2;                             /*!< PLL observer internal signal */
    int32_t current_i_mag;                      /*!< stator current magnitude */
    int32_t current_i_mag_filtered;             /*!< */
    int32_t delta_vi_angle_q31;                 /*!< */
    int32_t vref_x_sin_delta;                   /*!< */
    int32_t vref_x_cos_delta;                   /*!< */
    int32_t rotor_angle_q31;                    /*!< */
    int32_t rotor_speed;                        /*!< */
    int16_t phase_inductance_scale;             /*!< */
    int16_t lpf_n_bemf;                         /*!< Low pass filter coefficient for PLL internal signals */
    int32_t lpf_n_speed;                        /*!< Low pass filter coefficient for estimated speed */
    int16_t speed_angle_conversion_factor;      /*!< Rotor speed to angle conversion factor */
    int16_t speed_angle_conversion_factor_scale; /*!< Rotor speed to angle conversion factor scale */

  }PMSM_FOC_PLL_ESTIMATOR_t;

extern void pmsm_foc_svpwm_update(uint16_t Amplitude, uint16_t Angle);
extern void pmsm_foc_pi_controller_init(void);
void pmsm_foc_systemparameters_init_onceonly (void);

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
void pmsm_foc_current_reconstruction (uint16_t Previous_SVM_SectorNo,int32_t ADC_result1, int32_t ADC_result2, CurrentType * const HandlePtr);
#else
void pmsm_foc_current_reconstruction (int32_t ADC_Iu, int32_t ADC_Iv, int32_t ADC_Iw, CurrentType * const HandlePtr);
#endif
inline  __attribute__((section(".ram_code"))) void pmsm_foc_clarketransform(int32_t CurrentPhaseU, int32_t CurrentPhaseV, int32_t CurrentPhaseW,
                                          ClarkeTransformType* const HandlePtr);
inline  __attribute__((section(".ram_code"))) void pmsm_foc_parktransform_getresult(ParkTransformType* const HandlePtr);
inline  __attribute__((section(".ram_code"))) void pmsm_foc_parktransform (int32_t I_Alpha1Q31, int32_t I_Beta_1Q31, int32_t RotorAngleQ31);
inline void pmsm_foc_cart2polar(int32_t Torque_Vq, int32_t Flux_Vd, int32_t RotorAngleQ31);
inline  __attribute__((section(".ram_code"))) void pmsm_foc_car2pol_getresult(Car2PolType * const HandlePtr);

void pmsm_foc_speed_controller (void);
__attribute__((section(".ram_code"))) void pmsm_foc_scurve_ramp_generator(int32_t set_val, int32_t rampup_rate, int32_t rampdown_rate,
                                         int32_t speedrampstep, int32_t *reference_val);
__attribute__((section(".ram_code"))) void pmsm_foc_linear_ramp_generator(int32_t set_val, int32_t rampup_rate, int32_t rampdown_rate,
                                         int32_t speedrampstep, int32_t *reference_val);
extern __attribute__((section(".ram_code"))) void pmsm_foc_torque_controller(void);
extern __attribute__((section(".ram_code"))) void pmsm_foc_linear_torque_ramp_generator(int32_t current_set, int32_t inc_step,
                                                                               int32_t dec_step,
                                                                               FOCInputType* const HandlePtr);
extern __attribute__((section(".ram_code"))) void pmsm_foc_vq_controller(void);
extern __attribute__((section(".ram_code"))) void pmsm_foc_linear_vq_ramp_generator(int32_t current_set, int32_t inc_step,
                                                                           int32_t dec_step,
                                                                           FOCInputType* const HandlePtr);
void pmsm_foc_misc_works_of_irq (void);
void pmsm_foc_init_foc_rotorangle (void);
void pmsm_foc_init_foc_pi_iks (void);
__attribute__((section(".ram_code"))) void pmsm_foc_misc_works_of_foc (void);
__attribute__((section(".ram_code"))) void pmsm_foc_adjust_foc_parameters (void);
void pmsm_foc_stop_motor (void);
void pmsm_foc_variables_init (void);
void remote_control_variables_init (void); //Remote Control initial variable stored
void variables_init_main (void); //fault reset

extern void pmsm_foc_ccu4_debug3output(int32_t In04, uint16_t In04_Flag, uint16_t In04_N, int32_t In10, uint16_t In10_Flag,
                              uint16_t In10_N);

void pmsm_foc_update_vref_angle (int32_t Speed);

void pmsm_foc_stall_init();

/**
 * @brief To get current I_Alpha / I_Beta of last PWM cycle
 * I_Alpha = I_U
 * I_Beta = (I_U + 2 * I_V)/√3 = (I_V - I_W)/√3
 * Above transform scales down I_Mag (i.e.: |I|) by 2/3. Need scale up by 3/2.
 * Alternatively, can scale up inductance L in ωL|I| by 3/2 (legacy scaling).
 *
 * @param Current.I_U
 *      Current.I_V
 *      Current.I_W
 *
 *@retval Current.I_Alpha_1Q31
 *      Current.I_Beta_1Q31
 */

inline __attribute__((section(".ram_code"))) void pmsm_foc_clarketransform(int32_t CurrentPhaseU, int32_t CurrentPhaseV, int32_t CurrentPhaseW,
                                          ClarkeTransformType* const HandlePtr)
{
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  HandlePtr->I_Alpha_1Q31 = CurrentPhaseU << CORDIC_SHIFT;
  HandlePtr->I_Beta_1Q31 = (CurrentPhaseU + (CurrentPhaseV << 1)) * (DIV_SQRT3_Q14 <<(CORDIC_SHIFT-14));

#else

  if(SVM.Flag_3or2_ADC == 0){
    /* I_Alpha = (2 * I_U - (I_V + I_W))/3 */
    HandlePtr->I_Alpha_1Q31 = ((CurrentPhaseU << 1) - (CurrentPhaseV + CurrentPhaseW)) * (DIV_3 << (CORDIC_SHIFT-14));

    /*  I_Beta = (I_V - I_W)/√3 in 1Q31 */
    HandlePtr->I_Beta_1Q31 = (CurrentPhaseV - CurrentPhaseW) * (DIV_SQRT3_Q14 << (CORDIC_SHIFT-14));
  }
  else
  {
    switch(FOCOutput.Previous_SVM_SectorNo)
    {
      case 0:
      case 5:
          HandlePtr->I_Alpha_1Q31 = (-(CurrentPhaseV + CurrentPhaseW)) << CORDIC_SHIFT;
          HandlePtr->I_Beta_1Q31 = (CurrentPhaseV - CurrentPhaseW) * (DIV_SQRT3_Q14 << (CORDIC_SHIFT-14));
      break;
      case 1:
      case 2:
          HandlePtr->I_Alpha_1Q31 =  CurrentPhaseU << CORDIC_SHIFT;
          HandlePtr->I_Beta_1Q31 =  (CurrentPhaseU + (CurrentPhaseW << 1)) * (-(DIV_SQRT3_Q14 <<(CORDIC_SHIFT-14)));
        break;
      default:
          HandlePtr->I_Alpha_1Q31 = CurrentPhaseU << CORDIC_SHIFT;
          HandlePtr->I_Beta_1Q31 = (CurrentPhaseU + (CurrentPhaseV << 1)) * (DIV_SQRT3_Q14 <<(CORDIC_SHIFT-14));
        break;
    }
  }
#endif
}

/**
 * @brief CORDIC #1 - Park Transform
 * Iq = K[I_Beta cos(φ)-I_Alpha sin(φ)]/MPS   * Iq = Xfinal = K[X cos(Z) - Y sin(Z)] / MPS, where K = 1.646760258121.
 * Id = K[I_Alpha cos(φ)+I_Beta sin(φ)]/MPS   * Id = Yfinal = K[Y cos(Z) + X sin(Z)] / MPS      (Zfinal = 0).
 *
 * @param FOCInput.RotorAngleQ31
 *      Current.I_Alpha_1Q31
 *      Current.I_Beta_1Q31
 *
 *@retval MATH->CORRX
 *      MATH->CORRY
 */
inline __attribute__((section(".ram_code"))) void pmsm_foc_parktransform (int32_t I_Alpha1Q31, int32_t I_Beta_1Q31, int32_t RotorAngleQ31)
{
  /* General control of CORDIC Control Register */
  MATH->CON = CORDIC_ROTATION_MODE;

  /* Z = φ, Hall rotor angle, or estimated rotor angle of last PWM cycle from PLL */
  MATH->CORDZ = RotorAngleQ31;

  /* Y = I_Alpha */
  MATH->CORDY = I_Alpha1Q31;

  /* X = I_Beta. Input CORDX data, and auto start of CORDIC calculation (~62 kernel clock cycles) */
  MATH->CORDX = I_Beta_1Q31;

}

/**
 * @brief Get Cordic Result from Park Transform
 *
 * @param MATH->CORRX
 *      MATH->CORRY
 *
 *@retval FOCOutput.I_q
 *      FOCOutput.I_d
 */
inline __attribute__((section(".ram_code"))) void pmsm_foc_parktransform_getresult(ParkTransformType* const HandlePtr)
{
  /* Wait if CORDIC is still running calculation */
  while (MATH->STATC & 0x01)
  {
    continue;
  }
  /* Read CORDIC results Iq and Id - 32-bit. CORDIC Result Register [7:0] are 0x00 */
  HandlePtr->Iq = MATH->CORRX;
  HandlePtr->Id = MATH->CORRY;

  /*CPU computes the following simultaneously when CORDIC #2 is computing */
  HandlePtr->Iq >>= CORDIC_SHIFT;

  /* Shift to get real results */
  HandlePtr->Id >>= CORDIC_SHIFT;

  HandlePtr->Iq = (HandlePtr->Iq * 311) >> 8;   // x MPS/K.;

  HandlePtr->Id = (HandlePtr->Id * 311) >> 8;   // x MPS/K.;

}

/**
 * @brief Cartesian to Polar + Angle Addition, optimized FOC of Infineon
 *      Vref = K/MPS * sqrt(V_q^2+V_d^2)    * Xfinal = K/MPS * sqrt(X^2+Y^2), where K = 1.646760258121.
 *      Θ = atan(V_q/V_d) + Phi         * Zfinal = Z + atan(Y/X)          (Yfinal = 0).
 *
 * @param PI_Torque.Uk
 *      PI_Flux.Uk
 *
 *@retval FOCInput.RotorAngleQ31
 */
inline __attribute__((section(".ram_code"))) void pmsm_foc_cart2polar(int32_t Torque_Vq, int32_t Flux_Vd, int32_t RotorAngleQ31)
{

  /* General control of CORDIC Control Register */
  MATH->CON = CORDIC_VECTORING_MODE;

  /* Z = φ. Θ = atan(Vq/Vd) + rotor angle φ, equivalent to Inv. Park Transform */
  MATH->CORDZ = RotorAngleQ31;

  /* Y = Vq = PI_Torque.Uk */
  MATH->CORDY = Torque_Vq << CORDIC_SHIFT;

  /* X = Vd = PI_Flux.Uk. Input CORDX data, and auto start of CORDIC calculation */
  MATH->CORDX = Flux_Vd << CORDIC_SHIFT;

}

/**
 * @brief Cartesian to Polar + Angle Addition, optimized FOC of Infineon
 *
 * @param MATH->CORRX
 *      MATH->CORRZ
 *
 *@retval FOCInput.Vref32
 *      FOCInput.Vref_AngleQ31
 */
inline __attribute__((section(".ram_code"))) void pmsm_foc_car2pol_getresult(Car2PolType * const HandlePtr)
{
  /* Read CORDIC result |Vref| - 32-bit unsigned */
  HandlePtr->Vref32 = MATH->CORRX;

  /* Angle addition by CORDIC directly, where Θ = atan(Vq/Vd), φ is rotor angle */
  HandlePtr->Vref_AngleQ31 = MATH->CORRZ;

}

#if(OVERCURRENT_PROTECTION == ENABLED)
__STATIC_INLINE __attribute__((section(".ram_code"))) void pmsm_foc_over_current_protection_check(int32_t IDCLink, int32_t current_iq, uint16_t *factor)
{

    uint8_t status;

    if(IDCLink > IDC_MAX_LIMIT)
    {
      status = 1;
    }
    #if(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
    else if(current_iq > USER_IQ_REF_HIGH_LIMIT)
    {
      status = 1;
    }
    #endif
    else
    {
      status = 0;
    }

    if(status)
    {
      if(*factor > 5)
      {
        *factor -= 4;
      }
    }
    else
    {
      if(*factor < 4096)
      {
        *factor += 2;
      }

    }

}

#endif

#if(MOTOR_STALL_DETECTION == ENABLED)
/**
* @brief Calculates magnitude of resultant vector in circular mode.
* @param x Input X co-ordinate value
* @param y Input Y co-ordinate value
* @return None <BR>
*
* \par<b>Description</b><br>
* This function calculates the magnitude of resultant vector in circular mode.\n
* Magnitude = SQRT(x*x+y*y); where k = 1.646760258121 \n
*
* \par<b>Note</b><br>
* On XMC14 device this API will use HW CORDIC module for computation of resultant magnitude.\n
*/
__STATIC_INLINE __attribute__((section(".ram_code"))) void PMSM_FOC_CircularMag(int32_t x, int32_t y, int32_t angle)
{
	/* General control of CORDIC Control Register */
	MATH->CON = CORDIC_CIRCULAR_VECTORING_MODE;
	/* Z = φ. Θ = atan(Vq/Vd) + rotor angle φ, equivalent to Inv. Park Transform */
	MATH->CORDZ = angle;
	/* Y = Vq = PI_Torque.Uk */
	MATH->CORDY = (y << CORDIC_SHIFT);
	/* X = Vd = PMSM_FOC_FLUX_PI.Uk. Input CORDX data, and auto start of CORDIC calculation */
	MATH->CORDX = (x << CORDIC_SHIFT);
}


/**
* @brief Reads the results of magnitude of resultant vector in hyperbolic mode.
*
* @param None
*
*@retval None
*/
__STATIC_INLINE __attribute__((section(".ram_code"))) uint32_t PMSM_FOC_CircularMag_GetResult()
{
	uint32_t resultant_magnitude;
	while (MATH->STATC & 0x01); /* Check if CORDIC is busy */
	/* Read CORDIC result - 32-bit unsigned and scale down to get real value */
	resultant_magnitude = MATH->CORRX;

	/* Get real values by scaling down */
	resultant_magnitude = (uint32_t)(resultant_magnitude >> CORDIC_SHIFT);
	resultant_magnitude = (uint32_t)((resultant_magnitude * 311) >> 8); // x MPS/K.
	return (resultant_magnitude);
}

#endif

__STATIC_INLINE void pmsm_foc_divide(uint32_t dividend, uint32_t divisor)
{
	MATH->DIVCON = (0 << MATH_CON_ST_MODE_Pos | 1 << MATH_CON_X_USIGN_Pos);
	MATH->DVD = dividend;
	MATH->DVS = divisor;
}

__STATIC_INLINE void pmsm_foc_divide_getresult(uint32_t* result)
{
	while(MATH->DIVST);
	*result = MATH->QUOT;
}

#endif /* PMSM_FOC_CONTROLMODULES_PMSM_FOC_FUNCTIONS_H */


