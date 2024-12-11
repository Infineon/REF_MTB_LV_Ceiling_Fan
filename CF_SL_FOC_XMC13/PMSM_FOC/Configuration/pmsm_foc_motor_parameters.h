/**
 * @file pmsm_foc_motor_parameters.h
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
 * @file pmsm_foc_motor_parameters.h
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
 * *  PMSM FOC Software Support Hardware Naming and Description
 *=============================================================================
 *=============================================================================
 *        Hardware Kit (MCU Card + Inverter Card)|  MCU Card              | Inverter Card              | Motor Type                       |Hardware Description
 *--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *     1. KIT_XMC1X_AK_MOTOR_001                 |  KIT_XMC13_BOOT_001    | PMSM_LV15W                 | MAXON_MOTOR                      |* Support 1&3 shunts current sensing, External Op-Amp, BEMF Voltage Sensing Circuit
 *--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *     2. KIT_XMC750WATT_MC_AK_V1                |  KIT_XMC1300_DC_V1     | POWERINVERTER_750W         | EBM_PAPST_VENTI_FAN_MOTOR        |* Support 1&3 shunts current sensing, External Op-Amp, BEMF Voltage Sensing Circuit
 *--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *     8. CUSTOM_KIT                             |  CUSTOM_MCU            | CUSTOM_INVERTER            | CUSTOM_MOTOR                     |* Reserved for new add-in Hardware configuration
 *--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 ******************************************************************************
 *
 */

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_MOTOR_PARAMETERS_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_MOTOR_PARAMETERS_H_
#include "pmsm_foc_user_config.h"

/* ********************************************* MCI_DRONE_MOTOR *********************************************************************************************************/
#if(MOTOR_TYPE == MCI_DRONE_MOTOR)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (0.1f)          /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (60.0f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (6.0f)          /* Motor Pole Pairs (change to integer) */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (16000U)          /* Max Speed of User Motor*/
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM / 30U)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (3000U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (3000U)
/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (100U)            /* threshold Speed to transit from Open loop to closed loop*/
//#define USER_STARTUP_VF_OFFSET_V                   (float) (USER_VDC_LINK_V * 0.05f)                                  /* V/F startup offset in V */
//#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ          (float) (USER_VDC_LINK_V /ELECTRICAL_SPEED_FREQ_HZ)
#define USER_STARTUP_VF_OFFSET_V                    (0.3f)            /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.05f)           /* V/F start up slew rate in V/Hz */

/*      --------------------------------------------------- PI Controller Parameters ---------------------------------------- */
#define USER_PI_SPEED_KP                             ((uint16_t)1U<<1U)          /* (1<<15). Proportional gain Kp, uint16_t. */
#define USER_PI_SPEED_KI                             ((uint16_t)1U<<1U)                /* (1<<3). Integral gain Ki, uint16_t. */
#define USER_PI_SPEED_SCALE_KPKI                     (16)               /* Angle/speed resolution increase from 16 bit.*/
#define USER_PI_TORQUE_KP                            (CALCULATED_DEFAULT_IQID_KP >> 6)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_TORQUE_KI                            (CALCULATED_DEFAULT_IQID_KI >> 6)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_TORQUE_SCALE_KPKI                    (CALCULATED_DEFAULT_SCALING_CURRENT_KPKI + 0)
#define USER_PI_FLUX_KP                              (CALCULATED_DEFAULT_IQID_KP >> 6)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_FLUX_KI                              (CALCULATED_DEFAULT_IQID_KI >> 6)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_FLUX_SCALE_KPKI                      (CALCULATED_DEFAULT_SCALING_CURRENT_KPKI + 0)
#define USER_PI_PLL_KP                               ((uint16_t)1 << 5)              /* Proportional gain Kp, uint16_t. */
#define USER_PI_PLL_KI                               ((uint16_t)1 << 4)              /* (1<<4). Integral gain Ki, uint16_t. */
#define USER_PI_PLL_SCALE_KPKI                       (18)

/*      --------------------------------------------------- Speed PI Controller Parameters ---------------------------------------- */
/* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
#define PI_SPEED_IK_LIMIT_MIN                       (-(((1<<14) * 3) >> 2))      /* (-(1<<15)). I[k] output limit LOW. */
#define PI_SPEED_IK_LIMIT_MAX                       (((1<<14) * 3) >> 2)         /* (1<<15). I[k] output limit HIGH. */
#define PI_SPEED_UK_LIMIT_MIN                       (16)            /* (-32767), 16. U[k] output limit LOW. */
#define PI_SPEED_UK_LIMIT_MAX                       (32767)         /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */
/*      --------------------------------------------------- Torque/Iq PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MIN                      (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MAX                      (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MIN                      (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MAX                      32767         /* U[k] output limit HIGH. Normally no need change. */
/*      --------------------------------------------------- Flux/Id PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MIN                       (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MAX                       (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MIN                       (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MAX                       32767         /* U[k] output limit HIGH. Normally no need change. */
/*      --------------------------------------------------- PLL Estimator PI Controller Parameters ---------------------------------------- */
/* I[k] output limit LOW. */
#define PI_PLL_IK_LIMIT_MIN                        (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
#define PI_PLL_IK_LIMIT_MAX                        ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */

#if(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
#define PI_PLL_UK_LIMIT_MIN                        ((uint32_t)400 )                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX                        (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT)            /* U[k] output limit HIGH.*/
#elif((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC))
#define PI_PLL_UK_LIMIT_MIN                        ((uint32_t)SPEED_LOW_LIMIT >> 4)                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX                        (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT)            /* U[k] output limit HIGH.*/
#elif(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
#define PI_PLL_UK_LIMIT_MIN                        ((uint32_t)USER_VQ_REF_LOW_LIMIT >> 1)                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX                        (USER_VQ_REF_HIGH_LIMIT + USER_VQ_REF_LOW_LIMIT)            /* U[k] output limit HIGH.*/
#endif

/* ********************************************* DJI_DRONE_MOTOR *********************************************************************************************************/
#elif(MOTOR_TYPE == DJI_DRONE_MOTOR)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (0.1f)                                    /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (20.0f)                                   /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (7.0f)                                    /* Motor Pole Pairs (change to integer) */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (8000U)                                   /* Max Speed of User Motor*/
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM / 30U)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (1000U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (1000U)
#define USER_RATIO_S                                (1U)                                     /* Minimum ramp up and down ratio for S-curve profile */

/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (100U)                                    /* threshold Speed to transit from Open loop to closed loop */
#define USER_STARTUP_VF_OFFSET_V                    (0.3f)                                    /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.05f)                                   /* V/F start up slew rate in V/Hz */

/*      --------------------------------------------------- PI Controller Parameters ---------------------------------------- */
#define USER_PI_SPEED_KP                                 ((uint16_t)1U<<15U)                        /* (1<<15). Proportional gain Kp, uint16_t. */
#define USER_PI_SPEED_KI                                 ((uint16_t)2)                              /* (1<<3). Integral gain Ki, uint16_t. */
#define USER_PI_SPEED_SCALE_KPKI                         (10)                             /* Angle/speed resolution increase from 16 bit.*/
#define USER_PI_TORQUE_KP                                (CALCULATED_DEFAULT_IQID_KP)                      /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_TORQUE_KI                                (CALCULATED_DEFAULT_IQID_KI >> 4)                 /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_TORQUE_SCALE_KPKI                        (CALCULATED_DEFAULT_SCALING_CURRENT_KPKI + 0)
#define USER_PI_FLUX_KP                                  (CALCULATED_DEFAULT_IQID_KP)                      /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_FLUX_KI                                  (CALCULATED_DEFAULT_IQID_KI >> 4)                 /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_FLUX_SCALE_KPKI                          (CALCULATED_DEFAULT_SCALING_CURRENT_KPKI + 0)
#define USER_PI_PLL_KP                                   ((uint16_t)1<<5)                            /* Proportional gain Kp, uint16_t. */
#define USER_PI_PLL_KI                                   ((uint16_t)1<<4)                             /* (1<<4). Integral gain Ki, uint16_t. */
#define USER_PI_PLL_SCALE_KPKI                           (18)

/*      --------------------------------------------------- Speed PI Controller Parameters ---------------------------------------- */
/* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
#define PI_SPEED_IK_LIMIT_MIN                       (-(((1<<14) * 3) >> 2))                     /* (-(1<<15)). I[k] output limit LOW. */
#define PI_SPEED_IK_LIMIT_MAX                       (((1<<14) * 3) >> 2)                        /* (1<<15). I[k] output limit HIGH. */
#define PI_SPEED_UK_LIMIT_MIN                       (16)                                        /* (-32767), 16. U[k] output limit LOW. */
#define PI_SPEED_UK_LIMIT_MAX                       (32767)                                     /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */

/*      --------------------------------------------------- Torque/Iq PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MIN                      (-32768)                                    /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MAX                      (32767)                                     /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MIN                      (-32768)                                    /* U[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MAX                      32767                                       /* U[k] output limit HIGH. Normally no need change. */

/*      --------------------------------------------------- Flux/Id PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MIN                        (-32768)                                    /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MAX                        (32767)                                     /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MIN                        (-32768)                                    /* U[k] output limit LOW. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MAX                        32767                                       /* U[k] output limit HIGH. Normally no need change. */

/*      --------------------------------------------------- PLL Estimator PI Controller Parameters ---------------------------------------- */
/* I[k] output limit LOW. */
#define PI_PLL_IK_LIMIT_MIN                         (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
#define PI_PLL_IK_LIMIT_MAX                         ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */
#define PI_PLL_UK_LIMIT_MIN                         ((uint32_t)SPEED_LOW_LIMIT >> 4)                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX                         (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT)            /* U[k] output limit HIGH.*/


/* ********************************************* CUSTOM_MOTOR *********************************************************************************************************/
#elif(MOTOR_TYPE == CUSTOM_MOTOR)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#if(float_uart == DISABLED)
#define  USER_MOTOR_R_PER_PHASE_OHM                 (0.9f)        /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (4500.0f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (4.0f)          /* Motor Pole Pairs */
#endif
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#if((Remote_Control == ENABLED) || (IR_Remote_Control == ENABLED))
#if(float_uart == DISABLED)
#define USER_SPEED_HIGH_LIMIT_RPM                   (350U)      /* Max Speed of User Motor, */
#endif
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM /50U)/* This speed *3 is the FOC_EXIT_SPEED */
//#define USER_SPEED_RAMPUP_RPM_PER_S                 (30U)    /* Default 30 */
//#define USER_SPEED_RAMPDOWN_RPM_PER_S               (80U)    /* Default 30 */
#define USER_RATIO_S                                (1U)     // Minimum ramp up and down ratio for S-curve profile

#define USER_VF_SPEED_RAMPUP_RATE_RPM_PER_S         (6U)		   /*!< V/F open loop - speed ramp up rate */
#define USER_VF_SPEED_RAMPUP_SLEWRATE              (float)((float)USER_CCU8_PWM_FREQ_HZ * USER_SPEED_HIGH_LIMIT_RPM/((float)USER_VF_SPEED_RAMPUP_RATE_RPM_PER_S * 32767) + 1.0f)

//#define USER_SPEED_RAMPUPDOWN_RATE_RPM_PER_S         (80U)		   /*!< V/F open loop - speed ramp up rate */
//#define USER_SPEED_RAMPUPDOWN_SLEWRATE              (float)((float)USER_CCU8_PWM_FREQ_HZ * USER_SPEED_HIGH_LIMIT_RPM/((float)USER_SPEED_RAMPUPDOWN_RATE_RPM_PER_S * 32767) + 1.0f)

/*Speed Ramp Rate Configuration*/
#define USER_SPEED_RAMPUP_RATE_RPM_PER_S         (80U)
#define USER_SPEED_RAMPUP_SLEWRATE              (float)((float)USER_CCU8_PWM_FREQ_HZ * USER_SPEED_HIGH_LIMIT_RPM/((float)USER_SPEED_RAMPUP_RATE_RPM_PER_S * 32767) + 1.0f)

#define USER_SPEED_RAMPDOWN_RATE_RPM_PER_S       (20U)
#define USER_SPEED_RAMPDOWN_SLEWRATE              (float)((float)USER_CCU8_PWM_FREQ_HZ * USER_SPEED_HIGH_LIMIT_RPM/((float)USER_SPEED_RAMPDOWN_RATE_RPM_PER_S * 32767) + 1.0f)
/**/
#else

#define USER_SPEED_HIGH_LIMIT_RPM                   (350U)      /* Max Speed of User Motor, */ /* changed 105 to 4000 */
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM / 30U)  // This speed *3 is the FOC_EXIT_SPEED
#define USER_SPEED_RAMPUP_RPM_PER_S                 (40U)    // orignal 30
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (50U)    // original 30
#define USER_RATIO_S                                (1U)     // Minimum ramp up and down ratio for S-curve profile

#define USER_VF_SPEED_RAMPUP_RATE_RPM_PER_S         (6U)		   /*!< V/F open loop - speed ramp up rate */
#define USER_VF_SPEED_RAMPUP_SLEWRATE              (float)((float)USER_CCU8_PWM_FREQ_HZ * USER_SPEED_HIGH_LIMIT_RPM/((float)USER_VF_SPEED_RAMPUP_RATE_RPM_PER_S * 32767) + 1.0f)

#define USER_SPEED_RAMPUPDOWN_RATE_RPM_PER_S         (30U)		   /*!< V/F open loop - speed ramp up rate */
#define USER_SPEED_RAMPUPDOWN_SLEWRATE              (float)((float)USER_CCU8_PWM_FREQ_HZ * USER_SPEED_HIGH_LIMIT_RPM/((float)USER_SPEED_RAMPUPDOWN_RATE_RPM_PER_S * 32767) + 1.0f)


#define USER_SPEED_STARTUP_RPM                      (100U)    // Start up speed ref(REFERENCE_SPEED_STARTUP_RPM)
#endif

/*      --------------------------------------------------- Start Up Torque Parameters ----------------------------------*/
#if(Startup_Torque_Control == ENABLED)
#if((Remote_Control == ENABLED) || (IR_Remote_Control == ENABLED))
#define Startup_Torque_Exit_rpm         (80U)    /* speed to exit from startup Torque control */
#if(UART_INTERFACE == DISABLED)
#define Startup_Torque_Ref_Iq           (5000)//(5000_2024)  /* Iq reference in the torque control */
#define Startup_Torque_Ref_Iq_max       (10000)//(10000_2024)/* Iq reference in the torque control max value during ramp up */
#define Startup_slew_rate               (5)//(5U_2024)    /*For startup torque ref_iq increment*/
#endif/*#if(UART_INTERFACE == DISABLED)*/
#define Startup_Stall_Spd_Check_Time    (24000U)  /*After this timing to check on the below rpm & subsequent rpm | 1.5 | 1.7|27200*/
#define Startup_Stall_Spd_Check_Rpm     (0U)     /*To check of this rpm to determine stall - 1st check*/

#define Startup_Stall_Spd_Check_Time_2  (40000U)  /*After this timing to check on the below rpm & subsequent rpm | 2.2*/
#define Startup_Stall_Spd_Check_Rpm_2   (0U)    /*To check of this rpm to determine stall- 2nd check*/
#else
#define Startup_Torque_Exit_rpm         (120U)   /* speed from Torque to speed control */
#define Startup_Torque_Ref_Iq           (14500)  /* Iq reference in the torque control (4000-3shunt) - (3600-single shunt) 14500/32768 = 1.99A */
#endif/*(Remote_Control == ENABLED)*/
#endif/*(Startup_Torque_Control == ENABLED)*/

/*      ------------------------------------ Gradual Speed Feed Forward Speed Check ----------------------------------*/
#define Gradual_Stall_Spd_Check_Time    (24000U)
#define Gradual_Stall_Spd_Check_Rpm     (0U)

/*      -------------------------------------------Catch Free Run Startup Torque Parameters & brake time ----------------------------------*/
#if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)
#define CFR_Startup_exit_rpm                (140U)    /* Condition to exit CFR startup control */
#if(UART_INTERFACE == DISABLED)
#define CFR_Startup_Ref_Iq                  (500)     /* 0.274/ 4000~0.549 / 3640*/
#define CFR_Startup_Torque_Ref_Iq_max       (10000)
#define CFR_Startup_slew_rate               (4U)      /* For CFR startup torque ref_iq increment*/
#endif/*#if(UART_INTERFACE == DISABLED)*/
#define CFR_Post_Exit_Rpm_Check_Time        (16000U)  /* Timing not to check rpm exit condition | period * time = actual timing. 32000 = ~2s|16000 = 1s */
#define CFR_Startup_Stall_Spd_Check_Time    (32000U)  /*After this timing to check on the below rpm & subsequent rpm*/
#define CFR_Startup_Stall_Spd_Check_Rpm     (0U)     /*To check of this rpm to determine stall - 1st check*/
#endif

/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (30U)    /* Threshold Speed to transit from Open loop to closed loop 15U*/
//#define USER_STARTUP_VF_OFFSET_V                   (float) (USER_VDC_LINK_V * 0.05f)          /* V/F startup offset (0.05-6) in V */
//#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ          (float) (USER_VDC_LINK_V /(ELECTRICAL_SPEED_FREQ_HZ *2))
//#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.5f)//(0.479f)       //0.5f crompton motor    /* V/F start up slew rate in V/Hz */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.5f)

#if(float_uart == DISABLED)
#define USER_STARTUP_VF_OFFSET_V                    (1.5f)   /* V/F startup offset in V  default 1.0f - Pre-alignment volt*/
#endif

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/*      --------------------------------------------------- PI Controller Parameters (Latest) ---------------------------------------- */
#if(UART_INTERFACE == DISABLED)
#if((Remote_Control == ENABLED) || (IR_Remote_Control == ENABLED))

#define USER_PI_SPEED_KP                                 (12000U)          /* (1<<15). Proportional gain Kp, uint16_t. */
#define USER_PI_SPEED_KI                                 ((uint16_t)2U)                /* (1<<3). Integral gain Ki, uint16_t. */
#define USER_PI_SPEED_SCALE_KPKI                         (12U)                         /* RES_INC: Angle/speed resolution increase from 16 bit.*/
//#define USER_PI_SPEED_KP                                 (30000U)          /* (1<<15). Proportional gain Kp, uint16_t. */
//#define USER_PI_SPEED_KI                                 ((uint16_t)3U)                /* (1<<3). Integral gain Ki, uint16_t. */
//#define USER_PI_SPEED_SCALE_KPKI                         (13U)                         /* RES_INC: Angle/speed resolution increase from 16 bit.*/
//#define USER_PI_SPEED_KP                                 (2000U)          /* (1<<15). Proportional gain Kp, uint16_t. */
//define USER_PI_SPEED_KI                                 ((uint16_t)2U)                /* (1<<3). Integral gain Ki, uint16_t. */
//#define USER_PI_SPEED_SCALE_KPKI                         (14U)                         /* Angle/speed resolution increase from 16 bit.*/
#define USER_PI_TORQUE_KP                                (1544U)
#define USER_PI_TORQUE_KI                                (19U)
#define USER_PI_TORQUE_SCALE_KPKI                        (13U)
#define USER_PI_FLUX_KP                                  (1544U)
#define USER_PI_FLUX_KI                                  (19U)
#define USER_PI_FLUX_SCALE_KPKI                          (13U)
#define USER_PI_PLL_KP                                   (20U)
#define USER_PI_PLL_KI                                   (1U)
#define USER_PI_PLL_SCALE_KPKI                           (9U)

/*Spd_KP - */
/*SPd_KI - higher, faster speed ramp/down but higher overshoot*/
/*TorFLux_KP - higher, cause current straighten side line but more pointy at peak. too low, irregular form.*/
/*TorFLux_KP - Lower, more side curvy, more distroted current wave*/
/*TorFLux_KI - higher, cause more distortion for the duty cycle of Va | more curvy line for current wave*/
/*PLL_KP - higher, more inconsistent of motor speed, wider jump range of speed.*/
/*PLL_KI - higher, irregular wave form, faster/stronger startup | lower, better current waveform and smoother/softer startup*/

#else
#define USER_PI_SPEED_KP                                 (6000U)          /* (1<<15). Proportional gain Kp, uint16_t. */
#define USER_PI_SPEED_KI                                 ((uint16_t)2U)                /* (1<<3). Integral gain Ki, uint16_t. */
#define USER_PI_SPEED_SCALE_KPKI                         (12U)                         /* RES_INC: Angle/speed resolution increase from 16 bit.*/
//#define USER_PI_SPEED_KP                                 (30000U)          /* (1<<15). Proportional gain Kp, uint16_t. */
//#define USER_PI_SPEED_KI                                 ((uint16_t)3U)                /* (1<<3). Integral gain Ki, uint16_t. */
//#define USER_PI_SPEED_SCALE_KPKI                         (13U)                         /* RES_INC: Angle/speed resolution increase from 16 bit.*/
//#define USER_PI_SPEED_KP                                 (2000U)          /* (1<<15). Proportional gain Kp, uint16_t. */
//#define USER_PI_SPEED_KI                                 ((uint16_t)2U)                /* (1<<3). Integral gain Ki, uint16_t. */
//#define USER_PI_SPEED_SCALE_KPKI                         (14U)                         /* Angle/speed resolution increase from 16 bit.*/
#define USER_PI_TORQUE_KP                                (1544U)
#define USER_PI_TORQUE_KI                                (19U)
#define USER_PI_TORQUE_SCALE_KPKI                        (13U)
#define USER_PI_FLUX_KP                                  (1544U)
#define USER_PI_FLUX_KI                                  (19U)
#define USER_PI_FLUX_SCALE_KPKI                          (13U)
#define USER_PI_PLL_KP                                   (20U)
#define USER_PI_PLL_KI                                   (1U)
#define USER_PI_PLL_SCALE_KPKI                           (9U)
#endif/*if(Remote_Control == ENABLED)*/
#endif/*#if(UART_INTERFACE == DISABLED)*/
/*      --------------------------------------------------- Speed PI Controller Parameters ---------------------------------------- */
/* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
#define PI_SPEED_IK_LIMIT_MIN                       (-16000)      /* (-(1<<15)). I[k] output limit LOW. */
#define PI_SPEED_IK_LIMIT_MAX                       (16000)         /* (1<<15). I[k] output limit HIGH. */
#define PI_SPEED_UK_LIMIT_MIN                       (0)            /* (-32767), 16. U[k] output limit LOW. */
#define PI_SPEED_UK_LIMIT_MAX                       (12500)         /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. 32767*/
/*      --------------------------------------------------- Torque/Iq PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MIN                      (-36384)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MAX                      (36384)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */
//#define PI_TORQUE_UK_LIMIT_MIN                      (-31129)      /* U[k] output limit LOW. Normally no need change. */
//#define PI_TORQUE_UK_LIMIT_MAX                      31129         /* U[k] output limit HIGH. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MIN                      (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MAX                      32767         /* U[k] output limit HIGH. Normally no need change. */

/*      --------------------------------------------------- Flux/Id PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MIN                       (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MAX                       (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MIN                       (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MAX                       32767         /* U[k] output limit HIGH. Normally no need change. */

/*      --------------------------------------------------- PLL Estimator PI Controller Parameters ---------------------------------------- */
/* I[k] output limit LOW. */
#define PI_PLL_IK_LIMIT_MIN                        (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
#define PI_PLL_IK_LIMIT_MAX                        ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */
#define PI_PLL_UK_LIMIT_MIN                        ((uint32_t)SPEED_LOW_LIMIT >> 0)                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX                        32767            /* U[k] output limit HIGH.*/

#else/*#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)*/
/*      --------------------------------------------------- PI Controller Parameters (Latest-Spd) ---------------------------------------- */
#define USER_PI_SPEED_KP                                 (10000U)          /* (1<<15). Proportional gain Kp, uint16_t. */
#define USER_PI_SPEED_KI                                 ((uint16_t)1U)                /* (1<<3). Integral gain Ki, uint16_t. */
#define USER_PI_SPEED_SCALE_KPKI                         (13U)                         /* Angle/speed resolution increase from 16 bit.*/
#define USER_PI_TORQUE_KP                                (5000U)
#define USER_PI_TORQUE_KI                                (50U)
#define USER_PI_TORQUE_SCALE_KPKI                        (12U)
#define USER_PI_FLUX_KP                                  (5000U)
#define USER_PI_FLUX_KI                                  (50U)
#define USER_PI_FLUX_SCALE_KPKI                          (12U)
#define USER_PI_PLL_KP                                   (500U)
#define USER_PI_PLL_KI                                   (3U)
#define USER_PI_PLL_SCALE_KPKI                           (19U)

/*      --------------------------------------------------- Speed PI Controller Parameters ---------------------------------------- */
/* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
#define PI_SPEED_IK_LIMIT_MIN                       (-2000)      /* (-(1<<15)). I[k] output limit LOW. */
#define PI_SPEED_IK_LIMIT_MAX                       (2000)         /* (1<<15). I[k] output limit HIGH. */
#define PI_SPEED_UK_LIMIT_MIN                       (16)            /* (-32767), 16. U[k] output limit LOW. */
#define PI_SPEED_UK_LIMIT_MAX                       (32767)         /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */
/*      --------------------------------------------------- Torque/Iq PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MIN                      (-131072)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MAX                      (131072)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MIN                      (-31129)      /* U[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MAX                      31129         /* U[k] output limit HIGH. Normally no need change. */

/*      --------------------------------------------------- Flux/Id PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MIN                       (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MAX                       (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MIN                       (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MAX                       32767         /* U[k] output limit HIGH. Normally no need change. */

/*      --------------------------------------------------- PLL Estimator PI Controller Parameters ---------------------------------------- */
/* I[k] output limit LOW. */
#define PI_PLL_IK_LIMIT_MIN                        (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
#define PI_PLL_IK_LIMIT_MAX                        ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */
#define PI_PLL_UK_LIMIT_MIN                        ((uint32_t)SPEED_LOW_LIMIT >> 0)                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX                        ((SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT) << 1)            /* U[k] output limit HIGH.*/
#endif


/* ********************************************* VORNADO_FAN_MOTOR *********************************************************************************************************/
#elif(MOTOR_TYPE == VORNADO_FAN_MOTOR)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (0.55f)        /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (1440.0f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (4.0f)          /* Motor Pole Pairs */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (2200U)      /* Max Speed of User Motor*/
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM / 30U)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (500U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (500U)
#define USER_RATIO_S                                (1U)                                     /* Minimum ramp up and down ratio for S-curve profile */
/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (50U)            /* threshold Speed to transit from Open loop to closed loop */
//#define USER_STARTUP_VF_OFFSET_V                   (float) (USER_VDC_LINK_V * 0.05f)                                  /* V/F startup offset in V */
//#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ          (float) (USER_VDC_LINK_V /ELECTRICAL_SPEED_FREQ_HZ)
#define USER_STARTUP_VF_OFFSET_V                    (0.4f)            /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.07f)           /* V/F start up slew rate in V/Hz */

/*      --------------------------------------------------- PI Controller Parameters ---------------------------------------- */
#define USER_PI_SPEED_KP                                 ((uint16_t)1U << 15U)          /* (1<<15). Proportional gain Kp, uint16_t. */
#define USER_PI_SPEED_KI                                 ((uint16_t)3U)                /* (1<<3). Integral gain Ki, uint16_t. */
#define USER_PI_SPEED_SCALE_KPKI                         (10U)                         /* Angle/speed resolution increase from 16 bit.*/
#define USER_PI_TORQUE_KP                                (CALCULATED_DEFAULT_IQID_KP >> 0)
#define USER_PI_TORQUE_KI                                (CALCULATED_DEFAULT_IQID_KI >> 1)
#define USER_PI_TORQUE_SCALE_KPKI                        (CALCULATED_DEFAULT_SCALING_CURRENT_KPKI)
#define USER_PI_FLUX_KP                                  (CALCULATED_DEFAULT_IQID_KP >> 0)
#define USER_PI_FLUX_KI                                  (CALCULATED_DEFAULT_IQID_KI >> 1)
#define USER_PI_FLUX_SCALE_KPKI                          (CALCULATED_DEFAULT_SCALING_CURRENT_KPKI)
#define USER_PI_PLL_KP                                   ((uint16_t)1<<8)
#define USER_PI_PLL_KI                                   ((uint16_t)1<<6)
#define USER_PI_PLL_SCALE_KPKI                           (14U)

/*      --------------------------------------------------- Speed PI Controller Parameters ---------------------------------------- */
/* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
#define PI_SPEED_IK_LIMIT_MIN                       (-32768)      /* (-(1<<15)). I[k] output limit LOW. */
#define PI_SPEED_IK_LIMIT_MAX                       (32768)         /* (1<<15). I[k] output limit HIGH. */
#define PI_SPEED_UK_LIMIT_MIN                       (16)            /* (-32767), 16. U[k] output limit LOW. */
#define PI_SPEED_UK_LIMIT_MAX                       (32767)         /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */
/*      --------------------------------------------------- Torque/Iq PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MIN                      (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MAX                      (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MIN                      (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MAX                      32767         /* U[k] output limit HIGH. Normally no need change. */

/*      --------------------------------------------------- Flux/Id PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MIN                       (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MAX                       (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MIN                       (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MAX                       32767         /* U[k] output limit HIGH. Normally no need change. */

/*      --------------------------------------------------- PLL Estimator PI Controller Parameters ---------------------------------------- */
/* I[k] output limit LOW. */
#define PI_PLL_IK_LIMIT_MIN                        (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
#define PI_PLL_IK_LIMIT_MAX                        ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */
#define PI_PLL_UK_LIMIT_MIN                        ((uint32_t)SPEED_LOW_LIMIT >> 4)                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX                        ((SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT) << 1)            /* U[k] output limit HIGH.*/

/* ********************************************* NANOTEC_MOTOR **************************************************************************************************************/
#elif(MOTOR_TYPE == NANOTEC_MOTOR)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (1.0f)        /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (1050.0f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (4.0f)          /* Motor Pole Pairs */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (4000U)
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM / 30U)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (1000U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (1000U)
/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (200U)            /* threshold Speed to transit from Open loop to closed loop */
//#define USER_STARTUP_VF_OFFSET_V                   (float) (USER_VDC_LINK_V * 0.05f)                                  /* V/F startup offset in V */
//#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ          (float) (USER_VDC_LINK_V /ELECTRICAL_SPEED_FREQ_HZ)
#define USER_STARTUP_VF_OFFSET_V                    (0.3f)            /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.05f)           /* V/F start up slew rate in V/Hz */

/*      --------------------------------------------------- Speed PI Controller Parameters ---------------------------------------- */
#define PI_SPEED_KP                                 ((uint16_t)1U<<15U)          /* (1<<15). Proportional gain Kp, uint16_t. */
#define PI_SPEED_KI                                 ((uint16_t)2)                /* (1<<3). Integral gain Ki, uint16_t. */
#define PI_SPEED_SCALE_KPKI                         (10)               /* Angle/speed resolution increase from 16 bit.*/

/* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
#define PI_SPEED_IK_LIMIT_MIN                       (-(((1<<11) * 3) >> 2))      /* (-(1<<15)). I[k] output limit LOW. */
#define PI_SPEED_IK_LIMIT_MAX                       (((1<<11) * 3) >> 2)         /* (1<<15). I[k] output limit HIGH. */

#define PI_SPEED_UK_LIMIT_MIN                       (16)            /* (-32767), 16. U[k] output limit LOW. */
#define PI_SPEED_UK_LIMIT_MAX                       (4000)         /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */

/*      --------------------------------------------------- Torque/Iq PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define PI_TORQUE_KP                                (USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define PI_TORQUE_KI                                (USER_DEFAULT_IQID_KI >> 1)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define PI_TORQUE_SCALE_KPKI                        (SCALING_CURRENT_KPKI + 0)

#define PI_TORQUE_IK_LIMIT_MIN                      (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MAX                      (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

#define PI_TORQUE_UK_LIMIT_MIN                      (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MAX                      32767         /* U[k] output limit HIGH. Normally no need change. */

/*      --------------------------------------------------- Flux/Id PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define PI_FLUX_KP                                  (USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define PI_FLUX_KI                                  (USER_DEFAULT_IQID_KI >> 1)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define PI_FLUX_SCALE_KPKI                          (SCALING_CURRENT_KPKI + 0)

#define PI_FLUX_IK_LIMIT_MIN                        (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MAX                        (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

#define PI_FLUX_UK_LIMIT_MIN                        (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MAX                        32767         /* U[k] output limit HIGH. Normally no need change. */

/*      --------------------------------------------------- PLL Estimator PI Controller Parameters ---------------------------------------- */
#define PI_PLL_KP                                   ((uint16_t)(1<<6))              /* Proportional gain Kp, uint16_t. */
#define PI_PLL_KI                                   ((uint16_t)(1<<4))              /* (1<<4). Integral gain Ki, uint16_t. */
#define PI_PLL_SCALE_KPKI                           (17)

/* I[k] output limit LOW. */
#define PI_PLL_IK_LIMIT_MIN                         (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
#define PI_PLL_IK_LIMIT_MAX                         ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */

#define PI_PLL_UK_LIMIT_MIN                         ((uint32_t)SPEED_LOW_LIMIT >> 4)                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX                       (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT)            /* U[k] output limit HIGH.*/

/* ********************************************* MAXON_MOTOR ***************************************************************************************************************/
#elif (MOTOR_TYPE == MAXON_MOTOR)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (6.8f)        /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (3865.0f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (4.0f)          /* Motor Pole Pairs */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (4530.0f)
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM / 30U)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (500U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (500U)
#define USER_RATIO_S                                (1U)

/*      --------------------------------------------------- Torque2speed Start Up Parameters ----------------------------------*/
#define Startup_Torque_Exit_rpm                 (120U)   // speed from Torque to speed control
#define Startup_Torque_Ref_Iq                    (4000U)    // Iq reference in the torque control (4000-3shunt) - (3600-single shunt)

/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (200U)            /* threshold Speed to transit from Open loop to closed loop */
//#define USER_STARTUP_VF_OFFSET_V                   (float) (USER_VDC_LINK_V * 0.05f)                                  /* V/F startup offset in V */
//#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ          (float) (USER_VDC_LINK_V /ELECTRICAL_SPEED_FREQ_HZ)
#define USER_STARTUP_VF_OFFSET_V                    (1.0f)            /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.1f)           /* V/F start up slew rate in V/Hz */

/*      --------------------------------------------------- Speed PI Controller Parameters ---------------------------------------- */
#define USER_PI_SPEED_KP                             ((uint16_t)1U<<15U)          /* (1<<15). Proportional gain Kp, uint16_t. */
#define USER_PI_SPEED_KI                             ((uint16_t)3)                /* (1<<3). Integral gain Ki, uint16_t. */
#define USER_PI_SPEED_SCALE_KPKI                     (10)               /* Angle/speed resolution increase from 16 bit.*/
#define USER_PI_TORQUE_KP                            (CALCULATED_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_TORQUE_KI                            (CALCULATED_DEFAULT_IQID_KI >> 0)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_TORQUE_SCALE_KPKI                    (CALCULATED_DEFAULT_SCALING_CURRENT_KPKI + 0)
#define USER_PI_FLUX_KP                              (CALCULATED_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_FLUX_KI                              (CALCULATED_DEFAULT_IQID_KI >> 0)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_FLUX_SCALE_KPKI                      (CALCULATED_DEFAULT_SCALING_CURRENT_KPKI + 0)
#define USER_PI_PLL_KP                               ((uint16_t)(1<<8))              /* Proportional gain Kp, uint16_t. */
#define USER_PI_PLL_KI                               ((uint16_t)(1<<6))              /* (1<<4). Integral gain Ki, uint16_t. */
#define USER_PI_PLL_SCALE_KPKI                       (19)
/*      --------------------------------------------------- Speed PI Controller Parameters ---------------------------------------- */
/* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
#define PI_SPEED_IK_LIMIT_MIN                       (-(((1<<15) * 3) >> 2))      /* (-(1<<15)). I[k] output limit LOW. */
#define PI_SPEED_IK_LIMIT_MAX                       (((1<<15) * 3) >> 2)         /* (1<<15). I[k] output limit HIGH. */
#define PI_SPEED_UK_LIMIT_MIN                       (16)            /* (-32767), 16. U[k] output limit LOW. */
#define PI_SPEED_UK_LIMIT_MAX                       (32767)          /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */
/*      --------------------------------------------------- Torque/Iq PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */

#define PI_TORQUE_IK_LIMIT_MIN                      (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MAX                      (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MIN                      (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MAX                      32767         /* U[k] output limit HIGH. Normally no need change. */
/*      --------------------------------------------------- Flux/Id PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */

#define PI_FLUX_IK_LIMIT_MIN                        (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MAX                        (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MIN                        (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MAX                        32767         /* U[k] output limit HIGH. Normally no need change. */
/*      --------------------------------------------------- PLL Estimator PI Controller Parameters ---------------------------------------- */
/* I[k] output limit LOW. */
#define PI_PLL_IK_LIMIT_MIN                         (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
#define PI_PLL_IK_LIMIT_MAX                         ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */
#define PI_PLL_UK_LIMIT_MIN                         ((uint32_t)SPEED_LOW_LIMIT >> 4)                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX                         (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT)            /* U[k] output limit HIGH.*/
/* ********************************************* BEKO_WM_MOTOR ***************************************************************************************************************/
#elif (MOTOR_TYPE == BEKO_WM_MOTOR)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (2.5f)        /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (14000.0f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (4.0f)          /* Motor Pole Pairs */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (4800.0f)
#define USER_SPEED_LOW_LIMIT_RPM                    (50.0f)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (500U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (400U)

/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (100U)            /* threshold Speed to transit from Open loop to closed loop */
#define USER_STARTUP_VF_OFFSET_V                    (5.0f)            /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.75f)           /* V/F start up slew rate in V/Hz */

/*      --------------------------------------------------- Speed PI Controller Parameters ---------------------------------------- */
#define PI_SPEED_KP                                 ((uint16_t)1U<<16U - 1)          /* (1<<15). Proportional gain Kp, uint16_t. */
#define PI_SPEED_KI                                 ((uint16_t)1U << 8U)                /* (1<<3). Integral gain Ki, uint16_t. */
#define PI_SPEED_SCALE_KPKI                         (8)               /* Angle/speed resolution increase from 16 bit.*/
/* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
#define PI_SPEED_IK_LIMIT_MIN                       (-(1<<13))      /* (-(1<<15)). I[k] output limit LOW. */
#define PI_SPEED_IK_LIMIT_MAX                       (1<<13)         /* (1<<15). I[k] output limit HIGH. */
#define PI_SPEED_UK_LIMIT_MIN                       (4)            /* (-32767), 16. U[k] output limit LOW. */
#define PI_SPEED_UK_LIMIT_MAX                       (32767)         /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */

/*      --------------------------------------------------- Torque/Iq PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define PI_TORQUE_KP                                (USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define PI_TORQUE_KI                                (USER_DEFAULT_IQID_KI >> 0)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define PI_TORQUE_SCALE_KPKI                        (SCALING_CURRENT_KPKI + 0)
#define PI_TORQUE_IK_LIMIT_MIN                      (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MAX                      (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MIN                      (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MAX                      32767         /* U[k] output limit HIGH. Normally no need change. */

/*      --------------------------------------------------- Flux/Id PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define PI_FLUX_KP                                  (USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define PI_FLUX_KI                                  (USER_DEFAULT_IQID_KI >> 0)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define PI_FLUX_SCALE_KPKI                          (SCALING_CURRENT_KPKI + 0)
#define PI_FLUX_IK_LIMIT_MIN                        (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MAX                        (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MIN                        (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MAX                        32767         /* U[k] output limit HIGH. Normally no need change. */

/*      --------------------------------------------------- PLL Estimator PI Controller Parameters ---------------------------------------- */
#define PI_PLL_KP                                   ((uint16_t)(2000))              /* Proportional gain Kp, uint16_t. */
#define PI_PLL_KI                                   ((uint16_t)(1<<5))              /* (1<<4). Integral gain Ki, uint16_t. */
#define PI_PLL_SCALE_KPKI                           (17)
/* I[k] output limit LOW. */
#define PI_PLL_IK_LIMIT_MIN                         (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
#define PI_PLL_IK_LIMIT_MAX                         ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */
#define PI_PLL_UK_LIMIT_MIN                         ((uint32_t)SPEED_LOW_LIMIT >> 0)                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX                         (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT)            /* U[k] output limit HIGH.*/

/* ********************************************* EBM_PAPST_VENTI_FAN_MOTOR ******************************************************************************************************/
#elif (MOTOR_TYPE == EBM_PAPST_VENTI_FAN_MOTOR)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (9.8f)        /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (96000.0f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (3.0f)          /* Motor Pole Pairs */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (1200U)      /* Max Speed of User Motor*/
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM / 20U)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (1000U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (1000U)
#define USER_RATIO_S                                (1U)
/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (200U)            /* threshold Speed to transit from Open loop to closed loop */
//#define USER_STARTUP_VF_OFFSET_V                   (float) (USER_VDC_LINK_V * 0.05f)                                  /* V/F startup offset in V */
//#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ          (float) (USER_VDC_LINK_V /ELECTRICAL_SPEED_FREQ_HZ)
#define USER_STARTUP_VF_OFFSET_V                    (6.0f)            /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (3.0f)           /* V/F start up slew rate in V/Hz */

/*      --------------------------------------------------- Speed PI Controller Parameters ---------------------------------------- */
#define USER_PI_SPEED_KP                            ((uint16_t)(1U<<15U) - 1)          /* (1<<15). Proportional gain Kp, uint16_t. */
#define USER_PI_SPEED_KI                            ((uint16_t)1U<<3)                /* (1<<3). Integral gain Ki, uint16_t. */
#define USER_PI_SPEED_SCALE_KPKI                    (12)               /* Angle/speed resolution increase from 16 bit.*/
#define USER_PI_TORQUE_KP                           (CALCULATED_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_TORQUE_KI                           (CALCULATED_DEFAULT_IQID_KI >> 0)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_TORQUE_SCALE_KPKI                   (CALCULATED_DEFAULT_SCALING_CURRENT_KPKI + 0)
#define USER_PI_FLUX_KP                             (CALCULATED_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_FLUX_KI                             (CALCULATED_DEFAULT_IQID_KI >> 0)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_FLUX_SCALE_KPKI                     (CALCULATED_DEFAULT_SCALING_CURRENT_KPKI + 0)
#define USER_PI_PLL_KP                              ((uint16_t)(1<<8))              /* Proportional gain Kp, uint16_t. */
#define USER_PI_PLL_KI                              ((uint16_t)(1<<4))              /* (1<<4). Integral gain Ki, uint16_t. */
#define USER_PI_PLL_SCALE_KPKI                      (20)

/*      --------------------------------------------------- Speed PI Controller Parameters ---------------------------------------- */
/* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
#define PI_SPEED_IK_LIMIT_MIN                       (-(1<<14))      /* (-(1<<15)). I[k] output limit LOW. */
#define PI_SPEED_IK_LIMIT_MAX                       (1<<14)         /* (1<<15). I[k] output limit HIGH. */
#define PI_SPEED_UK_LIMIT_MIN                       (4)            /* (-32767), 16. U[k] output limit LOW. */
#define PI_SPEED_UK_LIMIT_MAX                       (32767)         /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */

/*      --------------------------------------------------- Torque/Iq PI Controller Parameters ---------------------------------------- */
#define PI_TORQUE_IK_LIMIT_MIN                      (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MAX                      (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MIN                      (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MAX                      32767         /* U[k] output limit HIGH. Normally no need change. */
/*      --------------------------------------------------- Flux/Id PI Controller Parameters ---------------------------------------- */
#define PI_FLUX_IK_LIMIT_MIN                        (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MAX                        (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MIN                        (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MAX                        32767         /* U[k] output limit HIGH. Normally no need change. */
/*      --------------------------------------------------- PLL Estimator PI Controller Parameters ---------------------------------------- */
/* I[k] output limit LOW. */
#define PI_PLL_IK_LIMIT_MIN                         (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
#define PI_PLL_IK_LIMIT_MAX                         ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */
#define PI_PLL_UK_LIMIT_MIN                         ((uint32_t)SPEED_LOW_LIMIT >> 1)                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX                         (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT)            /* U[k] output limit HIGH.*/
#endif

#define USER_REFERENCE_SPEED_RPM                    (50U)            /* V/F open loop only Reference Speed 50->15U */
#define USER_SPEED_THRESHOLD_FW_RPM_PER_S           (500U)        /* Threshold speed to use Flux Weakening 10000->500*/

#endif /* PMSM_FOC_CONFIGURATION_PMSM_FOC_MOTOR_PARAMETERS_H_ */
