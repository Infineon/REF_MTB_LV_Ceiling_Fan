/**
 * @file pmsm_foc_user_config.h
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
 * @file pmsm_foc_user_config.h
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

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_USER_CONFIG_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_USER_CONFIG_H_

/**********************************************************************************************************************
 * HEADER FILES
 *********************************************************************************************************************/
#include "pmsm_foc_const_macro.h"
#include "math.h"

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#define  PMSM_FOC_HARDWARE_BOARD                    CUSTOM_KIT                             /*1. KIT_XMC1X_AK_MOTOR_001
                                                                                             2. KIT_XMC750WATT_MC_AK_V1
                                                                                             3. CUSTOM_KIT*/

/*      --------------------------------------------------- Current feedback Sensing Mechanism ---------------------------------------- */
#define  CURRENT_SENSING                            USER_SINGLE_SHUNT_CONV            /*1. USER_SINGLE_SHUNT_CONV
                                                                                              2. USER_THREE_SHUNT_ASSYNC_CONV
                                                                                              3. USER_THREE_SHUNT_SYNC_CONV*/
/*      --------------------------------------------------- FOC Control and Startup Scheme (Only Select 1 Scheme at one time) --------- */
#define MY_FOC_CONTROL_SCHEME                       SPEED_CONTROLLED_DIRECT_FOC              /*1. SPEED_CONTROLLED_VF_ONLY,
                                                                                             2. SPEED_CONTROLLED_VF_MET_FOC
                                                                                             3. SPEED_CONTROLLED_DIRECT_FOC
                                                                                             4. TORQUE_CONTROLLED_DIRECT_FOC
                                                                                             5. VQ_CONTROLLED_DIRECT_FOC */
/* --------------------------------- Torque Startup Control -------------------------------------*/
#if (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC)
#define Startup_Torque_Control                               DISABLED               /*1. ENABLED       2. DISABLED (Disable Torque start up to spd)*/
#endif

/****************************************************************************************************************************
 *                                   Motor STARTUP CONFIGURATIONS
 ****************************************************************************************************************************/
#define USER_ROTOR_IPD_METHOD                       (ROTOR_IPD_INDUCTIVE_SENSING)            /*!< 1. ROTOR_IPD_PRE_ALIGNMENT
                                                                                              2. ROTOR_IPD_INDUCTIVE_SENSING
                                                                                              3. ROTOR_IPD_NONE */

/************************************ Rotor Pre Alignment *******************************************************************/
#if (USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)
#define USER_ROTOR_IPD_IND_SENSE_PATTERN_ONTIME     (0.8f)/*(0.4f)*//*(0.06f)*/                               /*!< CCU8 pulse width in mSec. Phase patterns are energized in
                                                                                                 sequence for this time. Range: 0.00033 to 21.8 mSec */
#define USER_ROTOR_IPD_IND_SENSE_CURRENT_DECAY_TIME (0.9f)/*(0.5f)*//*(0.04f)*/                              /*!< Time in mSec for current to decay after pulse is applied.
                                                                                                 Range: 0.00033 to 21.8 mSec */
#else
#if(UART_INTERFACE == DISABLED)
#define USER_ROTOR_PREPOSITION_TIME_MS              (1000U)            /* Rotor startup pre alignment time in miliseconds */     /*orignal-100*/
#endif/*#if(UART_INTERFACE == DISABLED)*/
#endif

/*      --------------------------------------------------- Micrium uC Probe Enable/Disable  ---------------------------------------- */
#define uCPROBE_GUI_no_UART                         ENABLED                                  /*1. ENABLED
                                                                                               2. DISABLED (Disable KPKI parameters saving features support)*/

/*      --------------------------------------------------- Micrium uC Probe Oscilloscope Enable/Disable  ---------------------------------------- */
#define uCPROBE_GUI_OSCILLOSCOPE                    ENABLED              /*1. ENABLED       2. DISABLED  (Disable Probe Sampling in PWM IRQ )*/

/*      --------------------------------------------------- Reference Speed Adjustment Method ---------------------------------------- */
#define SETTING_TARGET_SPEED                        MICRIUM_UC_ONLY                                 /*1. MICRIUM_UC_ONLY
                                                                                                     2. BY_POT_ONLY  */

/*      --------------------------------------------------- Constant Torque Control Mode (Used when Constant Torque Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- IQ_REF-limit low < IQ_REF < IQ_REF-limit high  ---------------------------------------- */
#define USER_IQ_CURRENT_ALLOWED_A                   (2.0f)                                                                          /* 0 < USER_IQ_CURRENT_ALLOWED_A < I_MAX_A*/
#define USER_IQ_REF_LOW_LIMIT                       (0U)
//#define USER_IQ_REF_HIGH_LIMIT                      (uint32_t) (32768 * USER_IQ_CURRENT_ALLOWED_A /I_MAX_A)                          /*  I_MAX_A = (VAREF_V/(USER_R_SHUNT_OHM * OP_GAIN_FACTOR)) / 2.0f), IFX_XMC_LVPB_R3 - 13.6A */
#define USER_IQ_REF_HIGH_LIMIT                      (12000U)
#define USER_IQ_RAMPUP                              (10U)
#define USER_IQ_RAMPDOWN                            (10U)
#define USER_IQ_RAMP_SLEWRATE                       (50U)                                                                           /* USER_IQ_RAMP_SLEWRATE x PWM period, every cycle increase USER_IQ_RAMPUP or USER_IQ_RAMPDOWN */

/*      --------------------------------------------------- Constant VQ Control Mode (Used when Constant VQ Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- VQ_REF-limit low < VQ_REF < VQ_REF-limit high  ---------------------------------------- */
#if(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
#define USER_VQ_VOLTAGE_ALLOWED_V                   (13U)                                                                            /* 0 < USER_VQ_VOLTAGE_ALLOWED_V < VREF_MAX_V          VREF_MAX_V =  (USER_VDC_LINK_V / USER_SQRT_3_CONSTANT)*/
#define USER_VQ_REF_LOW_LIMIT                       (0U)
#define USER_VQ_REF_HIGH_LIMIT                      (uint32_t)(32768U * USER_VQ_VOLTAGE_ALLOWED_V/VREF_MAX_V)
#define USER_VQ_RAMPUP                              (2U)
#define USER_VQ_RAMPDOWN                            (2U)
#define USER_VQ_RAMP_SLEWRATE                       (10U)         /* USER_VQ_RAMP_SLEWRATE x PWM period, every cycle increase USER_VQ_RAMPUP or USER_VQ_RAMPDOWN */
#endif

/*      --------------------------------------------------- SVM Switching Sequences ---------------------------------------- */
#define SVM_SWITCHING_SCHEME                        STANDARD_SVM_7_SEGMENT                        /*1. STANDARD_SVM_7_SEGMENT
                                                                                                    2. STANDARD_SVM_5_SEGMENT*/

#define ADC_ALTERNATE_REFERENCE                     DISABLED                                      /*1. ENABLED       2. DISABLED*/
/*      --------------------------------------------------- Advance Motor Stop Conditional Handling  ---------------------------------------- */
#define ADVANCE_CONDITIONAL_MOTOR_STOP              ENABLED                                       /*1. ENABLED       2. DISABLED*/
/*      --------------------------------------------------- Recommended Configuration (Strongly influenced by Execution Time) ---------------------------------------- */
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/*      --------------------------------------------------- Reference Speed Adjustment Method ---------------------------------------- */
#define SETTING_TARGET_SPEED                        MICRIUM_UC_ONLY
/*      --------------------------------------------------- Add d-q voltage decoupling components ---------------------------------------- */
#define DQ_DECOUPLING                               DISABLED                                      /*1. ENABLED       2. DISABLED*/
/*      --------------------------------------------------- Watch Dog Timer Activation ---------------------------------------- */
#define WATCH_DOG_TIMER                             DISABLED                                      /*1. ENABLED       2. DISABLED*/
/*      --------------------------------------------------- FOC Control Safety Protection ---------------------------------------- */
#define VDC_UNDER_OVERVOLTAGE_PROTECTION            ENABLED                                      /*1. ENABLED       2. DISABLED*/
#define OVERCURRENT_PROTECTION                      DISABLED                                      /*1. ENABLED       2. DISABLED*/
#define ACMP_OCP                                    ENABLED                              /*1. ENABLED       2. DISABLED*/

#else
/*      --------------------------------------------------- Add d-q voltage decoupling components ---------------------------------------- */
#define DQ_DECOUPLING                               ENABLED                                         /*1. ENABLED       2. DISABLED*/
#define WATCH_DOG_TIMER                             ENABLED                                         /*1. ENABLED       2. DISABLED*/
/*      --------------------------------------------------- FOC Control Safety Protection ---------------------------------------- */
#define VDC_UNDER_OVERVOLTAGE_PROTECTION            ENABLED                                         /*1. ENABLED       2. DISABLED*/
#define OVERCURRENT_PROTECTION                      DISABLED                                         /*1. ENABLED       2. DISABLED*/
#endif

/*------------------------------------ Speed Profile (Used when SPEED_CONTROLLED_VF_ONLY|SPEED_CONTROLLED_VF_MET_FOC|SPEED_CONTROLLED_DIRECT_FOC is enabled) ----------------------------------- */
#if(SETTING_TARGET_SPEED == BY_UART_ONLY)
#if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_ONLY)||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC))
#define MOTOR_SPEED_0                               (0U)
#define MOTOR_SPEED_1                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.1f)
#define MOTOR_SPEED_2                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.2f)
#define MOTOR_SPEED_3                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.3f)
#define MOTOR_SPEED_4                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.4f)
#define MOTOR_SPEED_5                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.5f)
#define MOTOR_SPEED_6                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.6f)
#define MOTOR_SPEED_7                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.7f)
#define MOTOR_SPEED_8                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.8f)
#define MOTOR_SPEED_9                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.9f)
#define MOTOR_SPEED_A                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 1.0f)

#define FOR_MOTOR_SPEED_0                           (0U)
#define FOR_MOTOR_SPEED_1                           (uint32_t)(((MOTOR_SPEED_1 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_2                           (uint32_t)(((MOTOR_SPEED_2 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_3                           (uint32_t)(((MOTOR_SPEED_3 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_4                           (uint32_t)(((MOTOR_SPEED_4 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_5                           (uint32_t)(((MOTOR_SPEED_5 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_6                           (uint32_t)(((MOTOR_SPEED_6 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_7                           (uint32_t)(((MOTOR_SPEED_7 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_8                           (uint32_t)(((MOTOR_SPEED_8 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_9                           (uint32_t)(((MOTOR_SPEED_9 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_A                           (uint32_t)(((MOTOR_SPEED_A - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))

#define ADC_STEP_INC_EACHRPM                        (uint32_t)(50/(USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM) * (1<<12))

/*------------------------------------ Speed Profile (Used when TORQUE_CONTROLLED_DIRECT_FOC is enabled) ------------------------------------------------------------------------------------ */
#elif(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
#define MOTOR_SPEED_0                               (0U)
#define MOTOR_SPEED_1                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.1f)
#define MOTOR_SPEED_2                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.2f)
#define MOTOR_SPEED_3                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.3f)
#define MOTOR_SPEED_4                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.4f)
#define MOTOR_SPEED_5                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.5f)
#define MOTOR_SPEED_6                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.6f)
#define MOTOR_SPEED_7                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.7f)
#define MOTOR_SPEED_8                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.8f)
#define MOTOR_SPEED_9                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.9f)
#define MOTOR_SPEED_A                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 1.0f)
#define FOR_MOTOR_SPEED_0                           (0U)
#define FOR_MOTOR_SPEED_1                           (uint32_t)(((MOTOR_SPEED_1 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_2                           (uint32_t)(((MOTOR_SPEED_2 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_3                           (uint32_t)(((MOTOR_SPEED_3 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_4                           (uint32_t)(((MOTOR_SPEED_4 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_5                           (uint32_t)(((MOTOR_SPEED_5 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_6                           (uint32_t)(((MOTOR_SPEED_6 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_7                           (uint32_t)(((MOTOR_SPEED_7 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_8                           (uint32_t)(((MOTOR_SPEED_8 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_9                           (uint32_t)(((MOTOR_SPEED_9 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_A                           (uint32_t)(((MOTOR_SPEED_A - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))

#define ADC_STEP_INC_EACHRPM                        (uint32_t)(50/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT) * (1<<12))

/*------------------------------------ Speed Profile (Used when VQ_CONTROLLED_DIRECT_FOC is enabled) ------------------------------------------------------------------------------------ */
#elif(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
#define MOTOR_SPEED_0                               (0U)
#define MOTOR_SPEED_1                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.1f)
#define MOTOR_SPEED_2                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.2f)
#define MOTOR_SPEED_3                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.3f)
#define MOTOR_SPEED_4                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.4f)
#define MOTOR_SPEED_5                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.5f)
#define MOTOR_SPEED_6                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.6f)
#define MOTOR_SPEED_7                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.7f)
#define MOTOR_SPEED_8                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.8f)
#define MOTOR_SPEED_9                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.9f)
#define MOTOR_SPEED_A                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 1.0f)
#define FOR_MOTOR_SPEED_0                           (0U)
#define FOR_MOTOR_SPEED_1                           (uint32_t)(((MOTOR_SPEED_1 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_2                           (uint32_t)(((MOTOR_SPEED_2 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_3                           (uint32_t)(((MOTOR_SPEED_3 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_4                           (uint32_t)(((MOTOR_SPEED_4 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_5                           (uint32_t)(((MOTOR_SPEED_5 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_6                           (uint32_t)(((MOTOR_SPEED_6 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_7                           (uint32_t)(((MOTOR_SPEED_7 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_8                           (uint32_t)(((MOTOR_SPEED_8 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_9                           (uint32_t)(((MOTOR_SPEED_9 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_A                           (uint32_t)(((MOTOR_SPEED_A - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define ADC_STEP_INC_EACHRPM                        (uint32_t)(50/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT) * (1<<12))
#endif
#endif/*#if(SETTING_TARGET_SPEED == BY_UART_ONLY)*/

#if(PMSM_FOC_HARDWARE_BOARD == KIT_XMC1X_AK_MOTOR_001)
#define MCUCARD_TYPE                                 KIT_XMC13_BOOT_001
#define INVERTERCARD_TYPE                            PMSM_LV15W
#define MOTOR_TYPE                                   MAXON_MOTOR
#elif(PMSM_FOC_HARDWARE_BOARD == KIT_XMC750WATT_MC_AK_V1)
#define MCUCARD_TYPE                                 KIT_XMC1300_DC_V1
#define INVERTERCARD_TYPE                            POWERINVERTER_750W
#define MOTOR_TYPE                                   EBM_PAPST_VENTI_FAN_MOTOR
#elif(PMSM_FOC_HARDWARE_BOARD == CUSTOM_KIT)
#define MCUCARD_TYPE                                 CUSTOM_MCU
#define INVERTERCARD_TYPE                            CUSTOM_INVERTER
#define MOTOR_TYPE                                   CUSTOM_MOTOR
#endif

#define TH_POT_ADC                                    50                            /* 50. Threshold POT ADC that motor can enter or exit motor idle state */
#define SYSTEM_BE_IDLE                                (ADC.ADC_POT < TH_POT_ADC)    /* POT ADC is too low */

#if(ADC_ALTERNATE_REFERENCE == ENABLED)
  #if((CURRENT_SENSING == USER_THREE_SHUNT_ASSYNC_CONV) || (CURRENT_SENSING == USER_THREE_SHUNT_SYNC_CONV))
      #define ADC_ALTERNATE_REF_PHASEUVW            ENABLED
      #define ADC_ALTERNATE_REF_SINGLESHUNT         DISABLED
  #elif(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      #define ADC_ALTERNATE_REF_PHASEUVW            DISABLED
      #define ADC_ALTERNATE_REF_SINGLESHUNT         ENABLED
  #endif
#elif(ADC_ALTERNATE_REFERENCE == DISABLED)
      #define ADC_ALTERNATE_REF_PHASEUVW            DISABLED
      #define ADC_ALTERNATE_REF_SINGLESHUNT         DISABLED
#endif

#define START_UP_MODE_1                             DISABLED                                 /*1. ENABLED       2. DISABLED*/
/****************************************************************************************************************************/
/* ------------------------------------------ *Uart Interface* ------------------------------------------------- */
#define UART_INTERFACE                              DISABLED    /* 1. ENABLED 2. DISABLED*/
#if (UART_INTERFACE == ENABLED)
#define float_uart                                  DISABLED    /* 1. ENABLED 2. DISABLED*/
#else
#define float_uart                                  DISABLED
#endif
/* ------------------------------------------ Checking & Testing Purpose* ------------------------------------------------- */
#define Test_point                                  ENABLED              /*Assign a pin as test point*/
#define MCU_Load_time_test                          DISABLED               /*Assign a pin to check on mcu load time*/
#define Protection_Indicator                        ENABLED               /*LED light up indicate protection occurs.*/
                                                    /*1. ENABLED       2. DISABLED*/

/* ------------------------------------------ Auto Startup upon power back on during power disruption* ------------------------------------------------- */
#define auto_start                                  ENABLED    /* 1. ENABLED 2. DISABLED*/


#if(UART_INTERFACE == ENABLED)
/* ------------------------------------------ RF Remote Control Feature* ------------------------------------------------- */
#define Remote_Control                              ENABLED              /*1. ENABLED       2. DISABLED*/

/* ------------------------------------------ IR Remote Control Feature* ------------------------------------------------- */
#define IR_Remote_Control                           ENABLED              /*Remember to set remote type through UART*/
#else
#define Remote_Control                              ENABLED              /*1. ENABLED       2. DISABLED (Disable remote control function)*/
#define IR_Remote_Control                           DISABLED              /* ONLY ONE Remote function can be ENABLED. Pick One.*/
#endif

/* ---------------------------------------- *Remote Control Configuration* ------------------------------------------------- */
#if((IR_Remote_Control == ENABLED) || (Remote_Control == ENABLED))

#if(Remote_Control == ENABLED)
#define remote_counter_time                         25U        /* Time to determine button pressed. 250 ~ 1s timing 0.1/1*250=*/
#define start_delay_time                            0U         /* Delay time before going into motor start*/
#define stop_delay_time                             0U         /* Delay time before going into motor stop*/
#endif

/*If using the ADC_POT to control fan speed*/
/*Currently this is use for adc pot display in GUI. spd1_pot value is use for motor start.*/
#define spd_1_adc_pot                               1753U /*150rpm - 0.428*4096, each increment 0.125*/
#define spd_2_adc_pot                               2339U /*200rpm - 0.571*4096*/
#define spd_3_adc_pot                               2925U /*250rpm - 0.714*4096*/
#define spd_4_adc_pot                               3510U /*300rpm - 0.857*4096*/
#define spd_5_adc_pot                               4095U /*350rpm - 4095*/

/* ----------------------------------------- Gradual Speed Feedforward ------------------------------------------------------*/
#define gradual_counter_time     68U    /*Number of cycle before it will increment the Iq_ref. Prev:68*/
#define gradual_slew_rate        10U    /*How much of an increment value for his each increment. Prev:15*/
#define gradual_stop_slew_rate   18U    /*Lower value, long*/

/* ----------------------------------- Conditional advanced stop ---------------------------------------------------- */
#define Advanced_stop_exit_speed  50U  /* Exiting rpm speed to Motor stop state*/
#endif /*#if((IR_Remote_Control == ENABLED) || (Remote_Control == ENABLED))*/

/* ----------------------------------------- BMI Reset ------------------------------------------------------*/
#if(UART_INTERFACE == ENABLED)
#define BMIresetting                                ENABLED         /*1. ENABLED       2. DISABLED*/
#endif
/* ------------------------------------------ Stall Detection Feature* ------------------------------------------------- */
#define MOTOR_STALL_DETECTION                       ENABLED         /*1. ENABLED       2. DISABLED*/

#if(MOTOR_STALL_DETECTION == ENABLED)

#if(float_uart == DISABLED)
#define STALL_RATIO                                 (2.1f) /* 2.1f STALL Ratio, >=1; */
#define STALL_CURRENT_A                             (2.5f)//(0.091f_2024)  0.3/* Motor STALL current in Amps, x7217 to int */
#endif

#define STALL_THRESHOLD_TIME_ms                     (2000.0f) /* Delay between stall condition occurrence to STALL declare */

//#define STALL_MAX_RETRY_COUNT                       (3U) /* Number of retry attempts before complete stop */
#endif /*#if(MOTOR_STALL_DETECTION == ENABLED)*/

/* --------------------------------------------- Fault Reset --------------------------------------------------------------------------- */
#define FAULT_RESET                                 ENABLED         /*1. ENABLED       2. DISABLED*/

#if (FAULT_RESET == ENABLED)
#if(UART_INTERFACE == DISABLED)
#define FAULT_MAX_RETRY_COUNT                       (3U)  /* Number of retry attempts before complete stop */
#endif
#define fault_timing                                25   /*The timing before reset function occur. period_freq*64*250=~1s*/
#endif

/* --------------------------------------------- Sleep MODE --------------------------------------------------------------------------- */
#define MCU_Sleep_Mode                              DISABLED         /*1. ENABLED       2. DISABLED*/

#if (MCU_Sleep_Mode == ENABLED)
#if(UART_INTERFACE == DISABLED)
#define sleepmode_countertime                       15000 /*About 3s(800),16s(4000), 20s(5333) 60s(15000) to go sleep mode| 16kHz - 64 counter*/
#endif

#endif

/* ------------------------------------------ *Catch Free Run* ------------------------------------------------- */
#define CATCH_FREE_RUNNING_WITH_BEMF                ENABLED /* 1. ENABLED 2. DISABLED*/

#if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)

#if(float_uart == DISABLED)
#define BEMF_THRESHOLD_V                            (13.0f)//(13.0f) /*Threshold BEMF in volts to enter into catch free 13~0.23V(real scope)*/
#endif

#define TOTAL_CATCH_TIME                            (8000U) /* Determines total time spent in catch free-running motor, x SVMPWM period.* (4 ~ 64)*/
#define BOOTSTRAP_PRE_CHARGE                        DISABLED /* 1. ENABLED 2. DISABLED*/
#endif

/* -------------------------------------------- Power Limit ------------------------------------------------------*/
#define MAX_POWER_LIMIT_EN_DIS						ENABLED 	/* 1. ENABLED 2. DISABLED*/
#if(MAX_POWER_LIMIT_EN_DIS == ENABLED)
#define TORQUE_CONST_VP_RAD_S 						(0.2f)		/*Torque const = Voltage peak / rad_s*/
#define POWER_LIMIT_MAX_W							(30U)			/*Power limit max in watt*/
//#define BRAKE_MIN_SPEED_RAD_S						(50U)
#endif

#endif /* PMSM_FOC_CONFIGURATION_PMSM_FOC_USER_CONFIG_H_ */

