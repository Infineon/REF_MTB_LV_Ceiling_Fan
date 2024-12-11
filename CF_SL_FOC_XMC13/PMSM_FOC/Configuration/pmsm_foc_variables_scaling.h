/**
 * @file pmsm_foc_variables_scaling.h
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
 * @file pmsm_foc_variables_scaling.h
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
 *
 */

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_VARIABLES_SCALING_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_VARIABLES_SCALING_H_

#include "pmsm_foc_invertercard_parameters.h"
#include "pmsm_foc_mcucard_parameters.h"
#include "pmsm_foc_motor_parameters.h"
#include "math.h"
#if(UART_INTERFACE == ENABLED)
extern uint32_t User_Para[70];            /*Flash Array size - depend how many parameter needed to be stored in flash*/
/*========================================= All define Parameter =====================================================*/
#define MotorConfig_Addr                    (uint32_t *)0x10032500		/* Flash location for Configuration data storage - 0x10032FFF(Max)*/
#define PI_SPEED_KP                         (uint32_t)User_Para[1]		/*PI Parameter*/
#define PI_SPEED_KI                         (uint32_t)User_Para[2]
#define PI_SPEED_SCALE_KPKI                 (uint32_t)User_Para[3]
#define PI_TORQUE_KP                        (uint32_t)User_Para[4]
#define PI_TORQUE_KI                        (uint32_t)User_Para[5]
#define PI_TORQUE_SCALE_KPKI                (uint32_t)User_Para[6]
#define PI_FLUX_KP                          (uint32_t)User_Para[7]
#define PI_FLUX_KI                          (uint32_t)User_Para[8]
#define PI_FLUX_SCALE_KPKI                  (uint32_t)User_Para[9]
#define PI_PLL_KP                           (uint32_t)User_Para[10]
#define PI_PLL_KI                           (uint32_t)User_Para[11]
#define PI_PLL_SCALE_KPKI                   (uint32_t)User_Para[12]

#define CFR_ADC_BEMF_UVW_HIGH_LIMIT         (uint32_t)User_Para[13]		/*Catch Free Run*/

#define CFR_BRAKE_TIME_1_MS                 (uint32_t)User_Para[14]		/*Brake Bootstrap Brake time*/
#define USER_BOOTSTRAP_PRECHARGE_TIME_MS    (uint32_t)User_Para[15]

#define USER_ROTOR_PREPOSITION_TIME_MS      (uint32_t)User_Para[16]		/*Pre-Alignment   --- Causing problem when enable, reason unknown  */
#define VREF_INC_STEP                       (uint32_t)User_Para[17]

#define I_OCP_SW                            (uint32_t)(User_Para[31])	/*Over Current Protection*/

/*FLOATING PARAMETERS*/
#if (float_uart == ENABLED)
#define STARTUP_CURRENT_A                   (uint32_t)User_Para[32]
#define CALCULATED_DEFAULT_IQID_KI          (uint32_t)User_Para[33]
#define DEFAULT_SCALE_OF_R                  (uint32_t)User_Para[34]
#define DEFAULT_R_SCALEDUP                  (uint32_t)User_Para[35]
#define DEFAULT_SCALE_OF_L                  (uint32_t)User_Para[36]
#define MOTOR_SCALE_OF_LQ                   (uint32_t)User_Para[37]
#define MOTOR_LQ_SCALEUP                    (uint32_t)User_Para[38]
#define CALCULATED_DEFAULT_SCALING_CURRENT_KPKI    (uint32_t)User_Para[39]
#define CALCULATED_DEFAULT_IQID_KP          (uint32_t)User_Para[40]
#define L_OMEGALI                           (uint32_t)User_Para[41]

#define STARTUP_SPEED                       (uint32_t)User_Para[42]
#define STARTUP_SPEED_THRESHOLD             (uint32_t)User_Para[43]
#define STARTUP_VF_SLEWRATE                 (uint32_t)User_Para[44]
#define SPEED_LOW_LIMIT_RPM                 (uint32_t)User_Para[45]
#define REFERENCE_SPEED_USER                (uint32_t)User_Para[46]
#define SPEED_HIGH_LIMIT_RPM                (uint32_t)User_Para[47]
#define RAMP_UP_SPEED                       (uint32_t)User_Para[48]
#define RAMP_DOWN_SPEED                     (uint32_t)User_Para[49]

#define BEMF_THRESHOLD_VALUE                (uint32_t)User_Para[50]
#define STARTUP_VF_OFFSET                   (uint32_t)User_Para[51]
#define STALL_RATIO_SCALEUP                 (uint32_t)User_Para[52]
#define STALL_CURRENT                       (uint32_t)User_Para[53]

#define VADC_DCLINK                         (uint32_t)User_Para[54]
#define BEMF_MAG_SCALING                    (uint32_t)User_Para[55]

#define USER_SPEED_HIGH_LIMIT_RPM           (uint32_t)User_Para[59]
#endif

#define FAULT_MAX_RETRY_COUNT               (uint32_t)User_Para[56]
#define sleepmode_countertime               (uint32_t)User_Para[57]
/************************************************************************************************************/
#else
/*================================================== PI define parameters from code/ PI parameters from Flash =====================================================*/
extern uint32_t User_Para[70];
#if(uCPROBE_GUI_no_UART == ENABLED)
//#define PARAM_HEADER                                (0x1010 + CURRENT_SENSING)                    /*for declare header for the Flash array */
//#define MotorConfig_Addr                            (uint32_t *)0x10032500 /*FOr declare*/
//#define PI_SPEED_KP                                 USER_PI_SPEED_KP
//#define PI_SPEED_KI                                 USER_PI_SPEED_KI                              /* (1<<3). Integral gain Ki, uint16_t. */
//#define PI_SPEED_SCALE_KPKI                         USER_PI_SPEED_SCALE_KPKI                      /* Angle/speed resolution increase from 16 bit.*/
//#define PI_TORQUE_KP                                USER_PI_TORQUE_KP
//#define PI_TORQUE_KI                                USER_PI_TORQUE_KI
//#define PI_TORQUE_SCALE_KPKI                        USER_PI_TORQUE_SCALE_KPKI
//#define PI_FLUX_KP                                  USER_PI_FLUX_KP
//#define PI_FLUX_KI                                  USER_PI_FLUX_KI
//#define PI_FLUX_SCALE_KPKI                          USER_PI_FLUX_SCALE_KPKI
//#define PI_PLL_KP                                   USER_PI_PLL_KP
//#define PI_PLL_KI                                   USER_PI_PLL_KI
//#define PI_PLL_SCALE_KPKI                           USER_PI_PLL_SCALE_KPKI

/*For GUI tuning and saving of its value*/
#define MotorConfig_Addr                            (uint32_t *)0x10032500   /* Flash location for Configuration data storage*/
#define PI_SPEED_KP                                 User_Para[1]
#define PI_SPEED_KI                                 User_Para[2]
#define PI_SPEED_SCALE_KPKI                         User_Para[3]
#define PI_TORQUE_KP                                User_Para[4]
#define PI_TORQUE_KI                                User_Para[5]
#define PI_TORQUE_SCALE_KPKI                        User_Para[6]
#define PI_FLUX_KP                                  User_Para[7]
#define PI_FLUX_KI                                  User_Para[8]
#define PI_FLUX_SCALE_KPKI                          User_Para[9]
#define PI_PLL_KP                                   User_Para[10]
#define PI_PLL_KI                                   User_Para[11]
#define PI_PLL_SCALE_KPKI                           User_Para[12]
#else
/*================================================================= Original Flash Code ==================================================================*/
#define PARAM_HEADER                                (0x1010 + CURRENT_SENSING)                    /* header for the Flash array */
#define MotorConfig_Addr                            (uint32_t *)0x10032500   /* Flash location for Configuration data storage*/
#define PI_SPEED_KP                                 User_Para[1]
#define PI_SPEED_KI                                 User_Para[2]
#define PI_SPEED_SCALE_KPKI                         User_Para[3]
#define PI_TORQUE_KP                                User_Para[4]
#define PI_TORQUE_KI                                User_Para[5]
#define PI_TORQUE_SCALE_KPKI                        User_Para[6]
#define PI_FLUX_KP                                  User_Para[7]
#define PI_FLUX_KI                                  User_Para[8]
#define PI_FLUX_SCALE_KPKI                          User_Para[9]
#define PI_PLL_KP                                   User_Para[10]
#define PI_PLL_KI                                   User_Para[11]
#define PI_PLL_SCALE_KPKI                           User_Para[12]
#endif
#endif/*#if(UART_INTERFACE == ENABLED)*/

/* ********************************************* MCU Parameters  **************************************************************************************************************/
#define CCU8_PERIOD_REG                             ((uint32_t)(USER_PCLK_FREQ_MHz*1000000)/USER_CCU8_PWM_FREQ_HZ)
#define CCU4_PERIOD_REG                             ((uint32_t)(USER_PCLK_FREQ_MHz*1000)/USER_CCU4_DEBUG_KHZ)
#define CCU8_DEADTIME_RISE                          (uint32_t)((USER_DEAD_TIME_US*USER_PCLK_FREQ_MHz) - 0.5)
#define CCU8_DEADTIME_FALL                          (uint32_t)((USER_DEAD_TIME_US*USER_PCLK_FREQ_MHz) - 0.5)
#define CCU8_DEAD_TIME                              (uint32_t)(CCU8_DEADTIME_RISE + 256 * CCU8_DEADTIME_FALL)
#define ADC_TRIGGER_POINT                           (uint32_t)(SVM_TZ_PZV * 0.85f)
#define SVM_LUTTABLE_SCALE                          (uint32_t)((CCU8_PERIOD_REG / MAX_VREF_AMPLITUDE) * 32768)

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#ifndef MIN_MAX_LIMIT1                                    /*!< Limits the input as per limits */
#define MIN_MAX_LIMIT1(Buffer,LimitH,LimitL) ((Buffer) > (LimitH)) ? (LimitH) : (((Buffer) < (LimitL))? (LimitL): (Buffer))
#endif

#ifndef MIN
#define MIN(a, b)          (((a) < (b)) ? (a) : (b))   /*!< macro returning smallest input */
#endif

#ifndef MAX
#define MAX(a, b)          (((a) > (b)) ? (a) : (b))   /*!< macro returning biggest input */
#endif

#ifndef ROUND
#define ROUND(x) 		   ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#endif

#ifndef ROUNDUP
#define ROUNDUP(x)         ((x)>=0?(((x - (int)x)==0 ? (int)x : (int)x+1)):(((x - (int)x)==0 ? (int)x : (int)x-1)))
#endif

/* **************************************** PLL Observer Setting *****************************************************************/
#define USER_PLL_LPF                                (1)                                 /*!<0 - Filter Disabled, >0 - Filter enabled(Time const = PWM_PERIOD * 2^USER_PLL_LPF) */
#define USER_PLL_SPEED_LPF                          (2)                                 /*!<0 - Filter Disabled, >0 - Filter enabled(Time const = PWM_PERIOD * 2^USER_PLL_LPF) */

#define PI                                          (3.1415926536f)
#define N_ESPEED_RAD_FCL 							(float)(2*PI*((float)USER_SPEED_HIGH_LIMIT_RPM/60)*USER_MOTOR_POLE_PAIR*(1/(float)USER_CCU8_PWM_FREQ_HZ))
#define N_ESPEED_T 									((1U<<15)-1)
#define N_EROTORANGLE_RAD 							(float)PI
#define N_EROTORANGLE_T 							((1U<<31)-1)
#define SPEED_TO_ANGLE_CONV_RAW 					(float)(N_ESPEED_RAD_FCL/N_ESPEED_T)*(N_EROTORANGLE_T/N_EROTORANGLE_RAD)
#define SPEED_TO_ANGLE_CONV_FACTOR_SCALE 			ROUND(MIN((float)(14 - (int16_t)log2(SPEED_TO_ANGLE_CONV_RAW)),14))
#define SPEED_TO_ANGLE_CONV_FACTOR 					ROUND(SPEED_TO_ANGLE_CONV_RAW*(1<<SPEED_TO_ANGLE_CONV_FACTOR_SCALE))
#define ANGLE_TO_SPEED_CONV_RAW 					(float)(1/((N_ESPEED_RAD_FCL/N_ESPEED_T)*(N_EROTORANGLE_T/N_EROTORANGLE_RAD)))
#define ANGLE_TO_SPEED_CONV_FACTOR_SCALE 			ROUNDUP(MIN((float)(15 - (int16_t)log2(ANGLE_TO_SPEED_CONV_RAW)),15))
//#define ANGLE_TO_SPEED_CONV_FACTOR_SCALE 			(int16_t)log2(ANGLE_TO_SPEED_CONV_RAW)
#define ANGLE_TO_SPEED_CONV_FACTOR 					ROUND(ANGLE_TO_SPEED_CONV_RAW*(1<<ANGLE_TO_SPEED_CONV_FACTOR_SCALE))

#if(OVERCURRENT_PROTECTION == ENABLED)
#define USER_IDC_MAXCURRENT_A                       (1.0f)
#define G_OPAMP_DC_CURRENT                          (USER_R_DCCURRENT_FEEDBACK_KOHM/ USER_RIN_DCCURRENT_KOHM)
#define IDC_MAX_LIMIT                               (uint32_t)((USER_IDC_MAXCURRENT_A * (1<<12)) / ((VAREF_V/(USER_DC_SHUNT_OHM*G_OPAMP_DC_CURRENT))/2.0f))          /* IDC Max limit to USER defined IDC Max Current */
#endif

#if(VDC_UNDER_OVERVOLTAGE_PROTECTION == ENABLED)
#define VDC_OVER_LIMIT                              ((uint16_t)(VADC_DCLINK * 115 / 100))                      /* VADC_DCLINK + 20%, DC link voltage Vdc maximum limit */
#define VDC_MIN_LIMIT                               ((uint16_t)(VADC_DCLINK * 80 / 100))                     /* VADC_DCLINK - 20%, DC link voltage Vdc min limit */
#endif
/* ********************************************* Normalization (u,v,w) represented by +2^15 ************************************************************************************/
#define N_I_UVW_A                                   I_MAX_A

#if(float_uart == DISABLED)
#define N_VREF_SVM_V                                VREF_MAX_V
#endif

/* ********************************************* Normalization (alpha,beta) represented by +2^15 *******************************************************************************/
#define N_I_ALPHABETA_A                            I_MAX_A

#if(float_uart == DISABLED)
#define N_VREF_ALPHABETA_V                         VREF_MAX_V
#endif

/* ********************************************* Normalization (d,q) represented by +2^15 **************************************************************************************/
#define N_I_DQ_A                                    I_MAX_A

#if(float_uart == DISABLED)
#define N_V_DQ_V                                    VREF_MAX_V
#endif

/* ********************************************* Motor Control Timing   ********************************************************************************************************/
#define BOOTSTRAP_BRAKE_TIME                       ((USER_BOOTSTRAP_PRECHARGE_TIME_MS * 1000) / PWM_PERIOD_TS_US)
#if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)
#define CFR_BOOTSTRAP_BRAKE_TIME_3                 ((CFR_BRAKE_TIME_3_MS * 1000) / PWM_PERIOD_TS_US)
#define CFR_BOOTSTRAP_BRAKE_TIME_2                 ((CFR_BRAKE_TIME_2_MS * 1000) / PWM_PERIOD_TS_US)
#define CFR_BOOTSTRAP_BRAKE_TIME_1                 ((CFR_BRAKE_TIME_1_MS * 1000) / PWM_PERIOD_TS_US)
#endif
#if (USER_ROTOR_IPD_METHOD == ROTOR_IPD_PRE_ALIGNMENT)
#define PRE_ALIGNMENT_TIME                          (uint32_t)(((USER_ROTOR_PREPOSITION_TIME_MS * 1000) / PWM_PERIOD_TS_US))
#endif
/* ********************************************* Amplitude Limit ***************************************************************************************************************/
#define MAX_VREF_AMPLITUDE                          (32768 - 0.5f)

#if(float_uart == DISABLED)
#define VREF_MAX_V                                  (USER_VDC_LINK_V / USER_SQRT_3_CONSTANT)
#endif

//using the internal gain. Seems the CIRCUIT_GAIN_Factor are not used in the control
//#define I_MAX_A                                     ((VAREF_V/(USER_R_SHUNT_OHM * OP_GAIN_FACTOR)) / 2U)               /* For IFX_XMC_LVPB_R3, I_MAX_A = 13.16A */
#define CIRCUIT_GAIN_FACTOR             (OP_GAIN_FACTOR*G_OPAMP_PER_PHASECURRENT)
#define I_MAX_A                         ((VAREF_V/(USER_R_SHUNT_OHM * CIRCUIT_GAIN_FACTOR))/ 2U)
//  4.54A

// 4.54A is 16384*2, we set 3.5A as the OCP, 3.5A/4.54*32768 = 25261, 30000/32768*4.54= 4.156A
#if(UART_INTERFACE == DISABLED)
#define I_OCP_SW                                   (25261U)  // defined software OCP value
#endif/*#if(UART_INTERFACE == DISABLED)*/

/* ********************************************* V/F Startup Parameter  ********************************************************************************************************/
#if(float_uart == DISABLED)
#define STARTUP_CURRENT_A                          (uint32_t)((USER_STARTUP_VF_OFFSET_V / USER_MOTOR_R_PER_PHASE_OHM))
#define STARTUP_SPEED                              (uint32_t)((float)USER_STARTUP_SPEED_RPM / USER_SPEED_HIGH_LIMIT_RPM * ((1<<15)-1))
#define STARTUP_SPEED_THRESHOLD                    (uint32_t)((float)USER_STARTUP_SPEED_THRESHOLD_RPM / USER_SPEED_HIGH_LIMIT_RPM * ((1<<15)-1))
#define STARTUP_VF_OFFSET                          (uint32_t)((USER_STARTUP_VF_OFFSET_V * 32768) / (N_VREF_SVM_V))
#define STARTUP_VF_SLEWRATE                        (uint32_t)(((USER_STARTUP_VF_SLEWRATE_V_PER_HZ * 32768) / (N_VREF_SVM_V) * (USER_MOTOR_POLE_PAIR)) / \
                                                     (USER_CCU8_PWM_FREQ_HZ*60)*SPEED_TO_ANGLE_CONV_RAW*32768)
#endif

/* ********************************************* POT ADC, or PWM to Adjust Speed  *********************************************************************************************/
#if(float_uart == DISABLED)
#define SPEED_LOW_LIMIT_RPM                        (uint32_t)((float)USER_SPEED_LOW_LIMIT_RPM / USER_SPEED_HIGH_LIMIT_RPM * ((1<<15)-1))
#define REFERENCE_SPEED_STARTUP_RPM                (uint32_t)(((USER_SPEED_STARTUP_RPM * USER_MOTOR_POLE_PAIR) /(USER_CCU8_PWM_FREQ_HZ*60))*65536)
#define REFERENCE_SPEED_USER                       (uint32_t)(((USER_REFERENCE_SPEED_RPM * USER_MOTOR_POLE_PAIR) /(USER_CCU8_PWM_FREQ_HZ*60))*65536)
#define SPEED_HIGH_LIMIT_RPM                       (uint32_t)((1<<15)-1)
//#define RAMP_UP_SPEED  				   			   (uint32_t)((float)USER_CCU8_PWM_FREQ_HZ * USER_SPEED_HIGH_LIMIT_RPM/((float)USER_SPEED_RAMPUPDOWN_RATE_RPM_PER_S * 32767) + 1.0f)
//#define RAMP_DOWN_SPEED				   			   (uint32_t)((float)USER_CCU8_PWM_FREQ_HZ * USER_SPEED_HIGH_LIMIT_RPM/((float)USER_SPEED_RAMPUPDOWN_RATE_RPM_PER_S * 32767) + 1.0f)
#define RAMP_UP_SPEED  				   			   (uint32_t)((float)USER_CCU8_PWM_FREQ_HZ * USER_SPEED_HIGH_LIMIT_RPM/((float)USER_SPEED_RAMPUP_RATE_RPM_PER_S * 32767) + 1.0f)
#define RAMP_DOWN_SPEED				   			   (uint32_t)((float)USER_CCU8_PWM_FREQ_HZ * USER_SPEED_HIGH_LIMIT_RPM/((float)USER_SPEED_RAMPDOWN_RATE_RPM_PER_S * 32767) + 1.0f)

#define VF_RAMPUP_RATE  			   			   (uint32_t)((float)USER_CCU8_PWM_FREQ_HZ * USER_SPEED_HIGH_LIMIT_RPM/((float)USER_VF_SPEED_RAMPUP_RATE_RPM_PER_S * 32767) + 1.0f)
#endif

#define SPEED_RAMP_UPDOWN_STEP		               (uint32_t) ((float)USER_SPEED_RAMPUPDOWN_RATE_RPM_PER_S * 32767 * USER_SPEED_RAMPUPDOWN_SLEWRATE/((float)USER_SPEED_HIGH_LIMIT_RPM * USER_CCU8_PWM_FREQ_HZ))
/**/
#define SPEEDRAMPUPSTEP                       (uint32_t) ((float)USER_SPEED_RAMPUP_RATE_RPM_PER_S * 32767 * USER_SPEED_RAMPUP_SLEWRATE/((float)USER_SPEED_HIGH_LIMIT_RPM * USER_CCU8_PWM_FREQ_HZ))
#define SPEEDRAMPDOWNSTEP                       (uint32_t) ((float)USER_SPEED_RAMPDOWN_RATE_RPM_PER_S * 32767 * USER_SPEED_RAMPDOWN_SLEWRATE/((float)USER_SPEED_HIGH_LIMIT_RPM * USER_CCU8_PWM_FREQ_HZ))
/**/
#define VF_SPEEDRAMPSTEP		            	   (uint32_t) ((float)USER_VF_SPEED_RAMPUP_RATE_RPM_PER_S * 32767 * USER_VF_SPEED_RAMPUP_SLEWRATE/((float)USER_SPEED_HIGH_LIMIT_RPM * USER_CCU8_PWM_FREQ_HZ))

#if(float_uart == DISABLED)
#define ELECTRICAL_SPEED_FREQ_HZ                    ((float)USER_SPEED_HIGH_LIMIT_RPM/(60/USER_MOTOR_POLE_PAIR) )
#endif

/* ********************************************* PI Controller Parameters (pmsm_foc_pi.h, if used) *****************************************************************************/
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)

#if(float_uart == DISABLED)
#define USER_CURRENTCTRL_CUTOFF_FREQ_HZ             (ELECTRICAL_SPEED_FREQ_HZ * 1.0f)
#endif

#else
#define USER_CURRENTCTRL_CUTOFF_FREQ_HZ             (ELECTRICAL_SPEED_FREQ_HZ * 3)
#endif

#if(float_uart == DISABLED)
#define CALCULATED_DEFAULT_SCALING_CURRENT_KPKI     (uint32_t)((log2((1<<15)/((USER_CURRENTCTRL_CUTOFF_FREQ_HZ/ELECTRICAL_SPEED_FREQ_HZ)*(SPEED_HIGH_LIMIT_RPM)*DEFAULT_L_SCALEDUP /(1<<(uint32_t)DEFAULT_SCALE_OF_L))))-1)
#define CALCULATED_DEFAULT_IQID_KP                  (uint32_t)((USER_CURRENTCTRL_CUTOFF_FREQ_HZ/ELECTRICAL_SPEED_FREQ_HZ)*(SPEED_HIGH_LIMIT_RPM)*DEFAULT_L_SCALEDUP /(1<<(uint32_t)(DEFAULT_SCALE_OF_L- CALCULATED_DEFAULT_SCALING_CURRENT_KPKI)))
#define CALCULATED_DEFAULT_IQID_KI                  (uint32_t)((CALCULATED_DEFAULT_IQID_KP) * USER_MOTOR_R_PER_PHASE_OHM * \
                                                       PWM_PERIOD_TS_US / USER_MOTOR_L_PER_PHASE_uH)
#endif

#define SCALEUP_MPS_K                               (uint16_t)(1<<8)
#define CORDIC_MPS_PER_K                            (USER_CORDIC_MPS / CORDIC_K) * (1 << SCALEUP_MPS_K)
/* ********************************************* Motor Parameter Scaling Conversion ********************************************************************************************/
#if(float_uart == DISABLED)
#define DEFAULT_L_NO_SCALEUP                       (float)((3.0f/2.0f)*(2.0f*USER_PI*USER_CCU8_PWM_FREQ_HZ) * \
                                                     (N_I_ALPHABETA_A/N_VREF_ALPHABETA_V) *(USER_MOTOR_L_PER_PHASE_uH/1000000)/(1<<16))

#define DEFAULT_L_SCALEDUP 						     ROUND(1.5f * N_ESPEED_RAD_FCL * USER_CCU8_PWM_FREQ_HZ * N_I_DQ_A / N_V_DQ_V * USER_MOTOR_L_PER_PHASE_uH / 1000000*(1<<DEFAULT_SCALE_OF_L))
#define DEFAULT_SCALE_OF_L 							 MIN(15-ROUNDUP(log2(1.5f * N_ESPEED_RAD_FCL * USER_CCU8_PWM_FREQ_HZ * N_I_DQ_A / N_V_DQ_V * USER_MOTOR_L_PER_PHASE_uH / 1000000 )),15)
#endif

/* ********************************************* MISC Integer Speed in SW to RPM speed of real world  **************************************************************************/
#define CONVERT_SPEED_TO_RPM                        (uint32_t)((USER_SPEED_HIGH_LIMIT_RPM * (1<<SPEED_TO_RPM_SCALE)) / \
                                                       SPEED_HIGH_LIMIT_RPM)
/*      --------------------------------------------------- MET Fine-tuning ---------------------------------------- */
#define USER_MET_THRESHOLD_HIGH                     (64U)
#define USER_MET_THRESHOLD_LOW                      (16U)
#define USER_MET_LPF                                (2U)
/*      --------------------------------------------------- pmsm_foc_parameter.h ---------------------------------------- */
/* ********************************************* Scaling SVM Modulator *********************************************************************************************************/
#define PWM_PERIOD_TS_US                            (1.0f/(USER_CCU8_PWM_FREQ_HZ)*1000000)
//#define SVM_LAMDA                                   (1.0f/USER_INVERSE_SVM_LAMDA)
#define SVM_LAMDA                                   (1.0f/20.0f)
#define SVM_TZ_PZV                                  (uint32_t)((CCU8_PERIOD_REG * SVM_LAMDA) + 0.5f)
#define uTZ_LAMDA_TS_US                             (SVM_LAMDA * PWM_PERIOD_TS_US)
/* ********************************************* Scaling SVM Modulator *********************************************************************************************************/
#define KS_SCALE_SVM                                (CCU8_PERIOD_REG / MAX_VREF_AMPLITUDE)
/* ********************************************* ADC Range  ********************************************************************************************************************/
#define VAREF_V                                     (5.0f)

#if(float_uart == DISABLED)
#define VADC_DCLINK                                 (uint32_t)(((USER_VDC_LINK_V * USER_DC_LINK_DIVIDER_RATIO)/VAREF_V) * (1<<12))
#endif

#define VDC_MAX_LIMIT                               ((uint16_t)((VADC_DCLINK * 19U)>>4U))                                         /* Vdc_ideal + 18.7%, DC link voltage Vdc maximum limit, only for braking usage, voltage clamping */




/* ********************************************* uC Probe Parameters  ***********************************************************/
/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
/* Timing parameters */
#define PERIOD_REG                    CCU8_PERIOD_REG
#define TZ_PZV                        SVM_TZ_PZV
#define BRAKE_TIME                    BOOTSTRAP_BRAKE_TIME
#if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)
#define CFR_BRAKE_TIME_3              CFR_BOOTSTRAP_BRAKE_TIME_3
#define CFR_BRAKE_TIME_2              CFR_BOOTSTRAP_BRAKE_TIME_2
#define CFR_BRAKE_TIME_1              CFR_BOOTSTRAP_BRAKE_TIME_1
#endif
#if (USER_ROTOR_IPD_METHOD == ROTOR_IPD_PRE_ALIGNMENT)
#define ALIGNMENT_TIME                PRE_ALIGNMENT_TIME
#endif

/* Scale of SVM sine Look-Up Table (LUT) */
#define SVM_LUT_SCALE                 SVM_LUTTABLE_SCALE

/* Motor parameters for ωL|I|, Vref/L in MET and PLL Observer */
#if(float_uart == DISABLED)
#define L_OMEGALI                     DEFAULT_L_SCALEDUP
#endif

#define SCALE_L                       DEFAULT_SCALE_OF_L

/* V/f parameter */
#define VQ_VF_OFFSET                  STARTUP_VF_OFFSET
#define VQ_VF_SLEW                    STARTUP_VF_SLEWRATE
#define DEFAULT_SPEED_STARTUP         STARTUP_SPEED
#define VF_TRANSITION_SPEED           STARTUP_SPEED_THRESHOLD
#define DEFAULT_SPEED_REFERENCE       REFERENCE_SPEED_USER

#define RAMPUP_RATE                   RAMP_UP_SPEED
#define RAMPDOWN_RATE                 RAMP_DOWN_SPEED

/* Motor speed limit */
#define SPEED_LOW_LIMIT               SPEED_LOW_LIMIT_RPM
#define SPEED_HIGH_LIMIT              SPEED_HIGH_LIMIT_RPM
#define TZ_PZVX2                      (SVM_TZ_PZV<<1)
#define HALF_TZ_PZV                   (SVM_TZ_PZV>>1)

/* For SW debug */
#define CCU4_PWM_PERIOD               CCU4_PERIOD_REG

/* CCU8 dead time */
#define DEAD_TIME                     CCU8_DEAD_TIME

#define SPEEDRAMPSTEP                 SPEED_RAMP_UPDOWN_STEP

/* For MET Fine-Tuning */
#define THRESHOLD_HIGH                USER_MET_THRESHOLD_HIGH
#define THRESHOLD_LOW                 USER_MET_THRESHOLD_LOW
#define SHIFT_MET_PLL                 USER_MET_LPF

/* ADC trigger point of Pseudo Zero Vectors */
#define TRIGGER_POINT                 ADC_TRIGGER_POINT

/* SVM voltage compensation */
#define ADC_DCLINK_IDEAL              VADC_DCLINK
/* Angle/speed resolution increase, especially for low-speed motor drive. */
/* Parameters for Startup Lock / Stall Detection */
#define MAX_RETRY_START_STALL         USER_MAX_RETRY_MOTORSTARTUP_STALL

/* Parameters for Trip / Over-Current Protection */
#define MAX_RETRY_START_TRIP          USER_MAX_RETRY_MOTORSTARTUP_TRIP

/* Increase Angle (and Speed) Resolution */

/* Integer speed in SW to rpm speed of real world (for debug) */
#define SPEED_TO_RPM                  CONVERT_SPEED_TO_RPM
#define SCALE_SPEED_TO_RPM            SPEED_TO_RPM_SCALE

/* Use UART to set POT ADC, and hence target motor speed. */
#define ADC_FOR_STEP_50_RPM         ADC_STEP_INC_EACHRPM

#if(MOTOR_STALL_DETECTION == ENABLED)
/************************************ MOTOR STALL DETECTION *****************************************************************************************/
#if(float_uart == DISABLED)
#define STALL_RATIO_SCALEUP         (uint32_t)((STALL_RATIO * 1024.0f))
#endif

#define STALL_RATIO_SCALE           (10U)

#if(float_uart == DISABLED)
#define STALL_CURRENT               ((int16_t)(STALL_CURRENT_A * 32767.0f/I_MAX_A)) /* Max. Iq current allowed in q15 format */
#endif

/************************************ STALL DETECTION CONFIG *******************************************************************************************/
#define STALL_THRESHOLD_TIME        (uint32_t)((STALL_THRESHOLD_TIME_ms * 1000) / PWM_PERIOD_TS_US)

/* Inductance scaled value for motor STALL detection */
#if(float_uart == DISABLED)
#define MOTOR_LQ_NO_SCALEUP   (float)((N_I_DQ_A/N_V_DQ_V) *(USER_MOTOR_L_PER_PHASE_uH/1000000))
#define MOTOR_SCALE_OF_LQ     ((uint16_t)(log2(((1<<12)/(MOTOR_LQ_NO_SCALEUP)))))
#define MOTOR_LQ_SCALEUP      ((int32_t)((MOTOR_LQ_NO_SCALEUP) * (1<<(uint32_t)MOTOR_SCALE_OF_LQ)))
#endif

/* Motor resistance scaling */
#if(float_uart == DISABLED)
#define DEFAULT_R_NO_SCALEUP (float)((N_I_DQ_A/N_V_DQ_V)*USER_MOTOR_R_PER_PHASE_OHM)
#define DEFAULT_SCALE_OF_R (uint32_t)(log2((1<<12)/DEFAULT_R_NO_SCALEUP))
#define DEFAULT_R_SCALEDUP (uint32_t)(((DEFAULT_R_NO_SCALEUP) * (1<<(uint32_t)DEFAULT_SCALE_OF_R)))
#endif

#endif /*#if(MOTOR_STALL_DETECTION == ENABLED)*/

/* ********************************************* BEMF Scaling - CATCH FREE RUNNING ***************************************************************************/
#if(float_uart == DISABLED)
#define BEMF_MAG_SCALING        (uint32_t)(USER_MAX_ADC_VDD_V * 8192.0f / (USER_VBEMF_RATIO * 3.0f * VREF_MAX_V))
#endif

#define SCALE_BEMF_MAG          (10U)

#if(float_uart == DISABLED)
#define BEMF_THRESHOLD_VALUE    ((uint16_t)((BEMF_THRESHOLD_V * 1024.0f * USER_VBEMF_RATIO)/USER_MAX_ADC_VDD_V)) /*Threshold BEMF in volts to enter into catch free */
#endif

/* new stall detection algo */
#define NO_LOAD_SPEED_RPM         USER_SPEED_HIGH_LIMIT_RPM     /* No-load speed for calculation of the BEMF constant */
#define STALL_TORQUE_NM           (0.05f)       /* Stall torque threshold */
#define KT_CONST                  (USER_SQRT_3_CONSTANT * (USER_VDC_LINK_V / 1.414f) / (NO_LOAD_SPEED_RPM) * 60.0 / (2.0 * PI))
#define KT_CONST_INV_Q12          (int32_t)((float)(1.0/KT_CONST) * (1<<12))

#define I_NORM_Q15                I_MAX_A //(int32_t)((1.414f * USER_R_SHUNT_OHM * USER_CURRENT_AMPLIFIER_GAIN / (USER_MAX_ADC_VDD_V / 2.0f)) * (1<< 15))
#define STALL_DETECTION_BLANKING  10 /* Define the threshold for blanking count at startup */

/********************************** Inductive Sensing Startup Parameters  ****************************************************************************/
#if(USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)
/* CCU8 count in edge-aligned mode with prescaler as 5 i.e 333ns resolution */

/*xmc13, 64MHz
 * prescaler =5, 64MHz/(2^prescaler) = 2,000,000Hz
 * resolution = 1/2,000,000Hz = 0.0000005s = 500ns
 * convert to millisecond = 0.0005ms
 * */

#define IPD_SENSE_PRESCALAR_VAL                    (5U)          // counter frequency = f(96MHz)/2^IPD_SENSE_PRESCALAR_VAL
#define IPD_SENSE_CCU8_PULSE_WIDTH_COUNT           (uint32_t)(USER_ROTOR_IPD_IND_SENSE_PATTERN_ONTIME / 0.0005f)
#define IPD_SENSE_CCU8_CURRENT_DECAY_COUNT         (uint32_t)(USER_ROTOR_IPD_IND_SENSE_CURRENT_DECAY_TIME / 0.0005f)
#define IPD_SENSE_DRIVER_DELAY                     (uint32_t)((USER_DRIVERIC_DELAY_US / 0.0005f) / 1000.0f)

/**/
#define FIRST_KICK_VALUE_V                          (1.2f)                               /*!<  First kick voltage applied after IPD to start the motor */
#define FIRST_KICK_VALUE                           (uint16_t)(32767U * FIRST_KICK_VALUE_V/VREF_MAX_V)                     /*!<  First kick voltage applied after IPD to start the motor */
#define FIRST_KICK_TIME                             (50U)                                 /*!< First kick time applied = FIRST_KICK_TIME(Count) * PWM_period */

/******************************************************************************************************************************************************
 * Initial Rotor Position Detection(IPD) Multi-channel pattern based on CCU8 slices for
 ******************************************************************************************************************************************************/
/** Both high and low side switches are OFF */
#define BOTH_OFF     (0U)
/** High side switch is ON */
#define HIGH_ON      (1U)
/** Low side switch is ON */
#define LOW_ON       (2U)
/** Both high and low side switches are ON */
#define BOTH_ON      (3U)

/** Ph W high side switch ON, Ph V low side switch ON, Ph U both switches OFF */
#define WH_VL_UOFF   (((uint32_t)HIGH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_W_NUM)) | ((uint32_t)LOW_ON   << (uint32_t)(4U*CCU8_SLICE_PHASE_V_NUM)) | ((uint32_t)BOTH_OFF << (uint32_t)(4U*CCU8_SLICE_PHASE_U_NUM)))
/** Ph W both switches OFF, Ph V low side switch ON, Ph U high side switch ON */
#define WOFF_VL_UH   (((uint32_t)BOTH_OFF << (uint32_t)(4U*CCU8_SLICE_PHASE_W_NUM)) | ((uint32_t)LOW_ON   << (uint32_t)(4U*CCU8_SLICE_PHASE_V_NUM)) | ((uint32_t)HIGH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_U_NUM)))
/** Ph W low side switch ON, Ph V both switches OFF, Ph U high side switch ON */
#define WL_VOFF_UH   (((uint32_t)LOW_ON   << (uint32_t)(4U*CCU8_SLICE_PHASE_W_NUM)) | ((uint32_t)BOTH_OFF << (uint32_t)(4U*CCU8_SLICE_PHASE_V_NUM)) | ((uint32_t)HIGH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_U_NUM)))
/** Ph W low side switch ON, Ph V high side switch ON, Ph U both switches OFF */
#define WL_VH_UOFF   (((uint32_t)LOW_ON   << (uint32_t)(4U*CCU8_SLICE_PHASE_W_NUM)) | ((uint32_t)HIGH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_V_NUM)) | ((uint32_t)BOTH_OFF << (uint32_t)(4U*CCU8_SLICE_PHASE_U_NUM)))
/** Ph W both switches OFF, Ph V high side switch ON, Ph U low side switch ON */
#define WOFF_VH_UL   (((uint32_t)BOTH_OFF << (uint32_t)(4U*CCU8_SLICE_PHASE_W_NUM)) | ((uint32_t)HIGH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_V_NUM)) | ((uint32_t)LOW_ON   << (uint32_t)(4U*CCU8_SLICE_PHASE_U_NUM)))
/** Ph W high side switch ON, Ph V both switches OFF, Ph U low side switch ON */
#define WH_VOFF_UL   (((uint32_t)HIGH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_W_NUM)) | ((uint32_t)BOTH_OFF << (uint32_t)(4U*CCU8_SLICE_PHASE_V_NUM)) | ((uint32_t)LOW_ON   << (uint32_t)(4U*CCU8_SLICE_PHASE_U_NUM)))
/** Ph U high side switch ON, Ph V low side switch ON , Ph W low side switch ON */
#define WL_VL_UH     (((uint32_t)LOW_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_W_NUM))  | ((uint32_t)LOW_ON   << (uint32_t)(4U*CCU8_SLICE_PHASE_V_NUM)) | ((uint32_t)HIGH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_U_NUM)))
/** Ph U low side switch ON, Ph V high side switch ON , Ph W high side switch ON */
#define WH_VH_UL     (((uint32_t)HIGH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_W_NUM)) | ((uint32_t)HIGH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_V_NUM)) | ((uint32_t)LOW_ON   << (uint32_t)(4U*CCU8_SLICE_PHASE_U_NUM)))
/** Ph U high side switch ON, Ph V high side switch ON , Ph W low side switch ON */
#define WL_VH_UH     (((uint32_t)LOW_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_W_NUM))  | ((uint32_t)HIGH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_V_NUM)) | ((uint32_t)HIGH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_U_NUM)))
/** Ph U low side switch ON, Ph V low side switch ON , Ph W high side switch ON */
#define WH_VL_UL     (((uint32_t)HIGH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_W_NUM)) | ((uint32_t)LOW_ON   << (uint32_t)(4U*CCU8_SLICE_PHASE_V_NUM)) | ((uint32_t)LOW_ON   << (uint32_t)(4U*CCU8_SLICE_PHASE_U_NUM)))
/** Ph U low side switch ON, Ph V high side switch ON , Ph W low side switch ON */
#define WL_VH_UL     (((uint32_t)LOW_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_W_NUM))  | ((uint32_t)HIGH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_V_NUM)) | ((uint32_t)LOW_ON   << (uint32_t)(4U*CCU8_SLICE_PHASE_U_NUM)))
/** Ph U high side switch ON, Ph V low side switch ON , Ph W high side switch ON */
#define WH_VL_UH     (((uint32_t)HIGH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_W_NUM)) | ((uint32_t)LOW_ON   << (uint32_t)(4U*CCU8_SLICE_PHASE_V_NUM)) | ((uint32_t)HIGH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_U_NUM)))
/** All lower switches are turned ON */
#define WL_VL_UL     (((uint32_t)LOW_ON  << (uint32_t)(4U*CCU8_SLICE_ADC_TR_NUM))   | ((uint32_t)LOW_ON   << (uint32_t)(4U*CCU8_SLICE_PHASE_W_NUM)) | ((uint32_t)LOW_ON   << (uint32_t)(4U*CCU8_SLICE_PHASE_V_NUM)) | ((uint32_t)LOW_ON   << (uint32_t)(4U*CCU8_SLICE_PHASE_U_NUM)))
/** All switches are turned ON */
#define ALL_ON       (((uint32_t)BOTH_ON  << (uint32_t)(4U*CCU8_SLICE_ADC_TR_NUM))  | ((uint32_t)BOTH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_W_NUM)) | ((uint32_t)BOTH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_V_NUM)) | ((uint32_t)BOTH_ON  << (uint32_t)(4U*CCU8_SLICE_PHASE_U_NUM)))
/** All switches are OFF */
#define ALL_OFF      (((uint32_t)BOTH_OFF  << (uint32_t)(4U*CCU8_SLICE_ADC_TR_NUM)) | ((uint32_t)BOTH_OFF << (uint32_t)(4U*CCU8_SLICE_PHASE_W_NUM)) | ((uint32_t)BOTH_OFF << (uint32_t)(4U*CCU8_SLICE_PHASE_V_NUM)) | ((uint32_t)BOTH_OFF << (uint32_t)(4U*CCU8_SLICE_PHASE_U_NUM)))
/** Synchronous start of three phases of CCU8 */
#define PMSM_FOC_CCU8_SYNC_START                   ((uint32_t)1 << (uint32_t)(8U + CCU8_MODULE_NUM))
/** Synchronous start of three phases of CCU4 */
#define PMSM_FOC_CCU4_SYNC_START                   (SCU_GENERAL_CCUCON_GSC40_Msk)
/* Synchronous start of CCU8 PWM generation and CCU4 AZ signal generation module */
#define PMSM_FOC_CCU8_CCU4_SYNC_START              ((uint32_t)(PMSM_FOC_CCU8_SYNC_START) | PMSM_FOC_CCU4_SYNC_START)
#endif // End of #if(USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)

/* =========================================== Power Limit =======================================*/
#if(MAX_POWER_LIMIT_EN_DIS == ENABLED)
#define LIMIT_MAX_IS 32767 /*current limit max*/

/*Power limit raw value calculation*/
#define SPEED_HIGH_LIMIT_RAD_S 				(float)((2*USER_PI)/60*USER_SPEED_HIGH_LIMIT_RPM)					/*Converting RPM to rad/s 				--  2*3.1415926536f/60*350 = 36.645 */
//#define POWER_BASE 							(float)(TORQUE_CONST_VP_RAD_S * I_MAX_A * SPEED_HIGH_LIMIT_RAD_S) 	/*K_t * Iq_base * w_mechanical_speed  	-- 0.2*36767*36.645= 269,465  || 0.2*4.54*36.645=33.27366*/
#define POWER_BASE 							(float)(I_MAX_A*USER_VDC_LINK_V) /*4.54*24.7=112.138*/

#define POWER_RAW 							(float)(POWER_LIMIT_MAX_W/POWER_BASE*32767) 						/* P_raw_max_int = raw_user_set_power_limit_watt /base * 2^15  	-- 35/269,465*32767=4.77 || 35/12.138*32767=10,227*/
#define POWER_RAW_SCALE_UP 					ROUND(POWER_RAW*32767)												/* P_raw_max_int_scale_up = P_max_int * 2^15  					-- 4.77*32767=156480 || 10,227*32767 */

//#define BRAKE_MIN_SPEED_RAW					(float)(BRAKE_MIN_SPEED_RAD_S/USER_CCU8_PWM_FREQ_HZ)
//#define BRAKE_MIN_SPEED						ROUND((BRAKE_MIN_SPEED_RAW/N_ESPEED_RAD_FCL)*N_ESPEED_T)

#endif/*#if(MAX_POWER_LIMIT_EN_DIS == ENABLED)*/


#endif /* PMSM_FOC_CONFIGURATION_PMSM_FOC_VARIABLES_SCALING_H_ */
