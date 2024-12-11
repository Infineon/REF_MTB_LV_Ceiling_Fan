/**
 * @file pmsm_foc_invertercard_parameters.h
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
 * @file pmsm_foc_invertercard_parameters.h
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

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_INVERTERCARD_PARAMETERS_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_INVERTERCARD_PARAMETERS_H_

#include "pmsm_foc_user_config.h"

/* ********************************************* PMSM_LV15W ******************************************************************************************************/
#if(INVERTERCARD_TYPE == PMSM_LV15W)
#define INTERNAL_OP_GAIN                            DISABLED               /*1. ENABLED       2. DISABLED (Please configure OP-Gain manually) */
#define USER_VDC_LINK_V                             (24.0f)                 /* Hardware Inverter VDC link voltage in V  */
#define USER_DEAD_TIME_US                           (0.75f)                 /* deadtime, rise(left) and fall values in us  */
#define USER_CCU8_PWM_FREQ_HZ                       (20000U)              /* CCU8 PWM Switching Frequency in Hz*/
#define USER_BOOTSTRAP_PRECHARGE_TIME_MS            (20U)                 /* Initial Bootstrap precharging time in ms */
#define USER_DC_LINK_DIVIDER_RATIO                  (float)(5.1f/(5.1f+47.0f))           /* R1/(R2+R1) ratio for DC link MCU ADC */
#define USER_VBEMF_RATIO                            (float)(5.2f/(5.2f+47.0f))           /* R1/(R2+R1) ratio for BEMF Voltage sensing circuit ratio */
#define USER_CURRENT_TRIP_THRESHOLD_A               (3.0f)               /* threshold current for trip detection in Ampere*/
#define USER_TRIP_THRESHOLD_TIME_MS                 (100U)               /* threshold time for trip detection in ms */
#define USER_MAX_RETRY_MOTORSTARTUP_TRIP            (3U)                /* Max retry of motor startup if trip  */
/*      --------------------------------------------------- Motor Phase Current Measurement ---------------------------------------- */
#define USER_R_SHUNT_OHM                            (0.05f)               /* Phase shunt resistor in ohm */
#define USER_DC_SHUNT_OHM                           (0.05f)               /* DC link shunt current resistor in ohm */
#define USER_RIN_PHASECURRENT_KOHM                  (1.0f)                 /* R_IN (of equivalent amplifier) kohm */
#define USER_R_PHASECURRENT_FEEDBACK_KOHM           (16.4f)                  /* R_FEEDBACK (of equivalent amplifier) kohm */
#define USER_RIN_DCCURRENT_KOHM                     (10.0f)                  /* Rf for dc current sensing */
#define USER_R_DCCURRENT_FEEDBACK_KOHM              (75.0f)                   /* Rin for dc current sensing */
#define USER_MAX_ADC_VDD_V                          (5.0f)                   /* VDD5, maximum voltage at ADC */
#define G_OPAMP_PER_PHASECURRENT                    (USER_R_PHASECURRENT_FEEDBACK_KOHM / USER_RIN_PHASECURRENT_KOHM)
#define I_MAX_A                                     ((VAREF_V/(USER_R_SHUNT_OHM * OP_GAIN_FACTOR)) / 2U)               /* For IFX_XMC_LVPB_R3, I_MAX_A = 13.16A */

#if(INTERNAL_OP_GAIN == ENABLED)
#define OP_GAIN_FACTOR                             (3U)                       /* Different HW Board has different OP Gain factor, XMC13 built-in Gain Factor available 1, 3, 6 and 12 only*/
#elif(INTERNAL_OP_GAIN == DISABLED)
#define OP_GAIN_FACTOR                              G_OPAMP_PER_PHASECURRENT
#endif

#define GATE_DRIVER_INPUT_LOGIC                    PASSIVE_LOW                   /*1. PASSIVE_HIGH       2. PASSIVE_LOW (Please refer the gate driver datasheet) */
#if(GATE_DRIVER_INPUT_LOGIC == PASSIVE_HIGH)
#define MOTOR_COASTING_HIGH_SIDE                    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#define MOTOR_COASTING_LOW_SIDE                     XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW

#define MOTOR_RUN_HIGH_SIDE                         XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#define MOTOR_RUN_LOW_SIDE                          XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#elif(GATE_DRIVER_INPUT_LOGIC == PASSIVE_LOW)
#define MOTOR_COASTING_HIGH_SIDE                    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW
#define MOTOR_COASTING_LOW_SIDE                     XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH

#define MOTOR_RUN_HIGH_SIDE                         XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW
#define MOTOR_RUN_LOW_SIDE                          XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW
#endif

//#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH /*OCP*/

#define INVERTER_ENABLE_PIN    (1U)             /* 1 = Active High, 0 Active Low*/

#if(INVERTER_ENABLE_PIN == 0U)
  #define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
  #define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#elif(INVERTER_ENABLE_PIN == 1U)
  #define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
  #define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#endif

#define IU_ADC_BIAS                               (2087)
#define IV_ADC_BIAS                               (2078)
#define IW_ADC_BIAS                               (2061)

/* ********************************************* POWERINVERTER_750W ******************************************************************************************************/
#elif(INVERTERCARD_TYPE == POWERINVERTER_750W)
#define INTERNAL_OP_GAIN                            DISABLED               /*1. ENABLED       2. DISABLED (Please configure OP-Gain manually) */
#define USER_VDC_LINK_V                             (320.0f)                 /* Hardware Inverter VDC link voltage in V  */
#define USER_DEAD_TIME_US                           (0.75f)                 /* deadtime, rise(left) and fall values in us  */
#define USER_CCU8_PWM_FREQ_HZ                       (16000U)              /* CCU8 PWM Switching Frequency in Hz*/
#define USER_BOOTSTRAP_PRECHARGE_TIME_MS            (20U)                 /* Initial Bootstrap precharging time in ms */
#define USER_DC_LINK_DIVIDER_RATIO                  (float)(10.0f/(10.0f+990.0f))           /* R1/(R2+R1) ratio for DC link MCU ADC */
#define USER_VBEMF_RATIO                            (float)(12.0f/(12.0f+990.0f))           /* R1/(R2+R1) ratio for BEMF Voltage sensing circuit ratio */
#define USER_CURRENT_TRIP_THRESHOLD_A               (4.0f)               /* threshold current for trip detection in Ampere*/
#define USER_TRIP_THRESHOLD_TIME_MS                 (100U)               /* threshold time for trip detection in ms */
#define USER_MAX_RETRY_MOTORSTARTUP_TRIP            (3U)                /* Max retry of motor startup if trip  */
/*      --------------------------------------------------- Motor Phase Current Measurement ---------------------------------------- */
#define USER_R_SHUNT_OHM                            (0.05f)               /* Phase shunt resistor in ohm */
#define USER_DC_SHUNT_OHM                           (0.05f)               /* DC link shunt current resistor in ohm */
#define USER_RIN_PHASECURRENT_KOHM                  (1.0f)                 /* R_IN (of equivalent amplifier) kohm */
#define USER_R_PHASECURRENT_FEEDBACK_KOHM           (12.0f)                  /* R_FEEDBACK (of equivalent amplifier) kohm */
#define USER_RIN_DCCURRENT_KOHM                     (10.0f)                  /* Rf for dc current sensing */
#define USER_R_DCCURRENT_FEEDBACK_KOHM              (75.0f)                   /* Rin for dc current sensing */
#define USER_MAX_ADC_VDD_V                          (5.0f)                   /* VDD5, maximum voltage at ADC */
#define G_OPAMP_PER_PHASECURRENT                    (USER_R_PHASECURRENT_FEEDBACK_KOHM / USER_RIN_PHASECURRENT_KOHM)
#define I_MAX_A                                     ((VAREF_V/(USER_R_SHUNT_OHM * OP_GAIN_FACTOR)) / 2U)               /* For IFX_XMC_LVPB_R3, I_MAX_A = 13.16A */

#if(INTERNAL_OP_GAIN == ENABLED)
#define OP_GAIN_FACTOR                             (3U)                       /* Different HW Board has different OP Gain factor, XMC13 built-in Gain Factor available 1, 3, 6 and 12 only*/
#elif(INTERNAL_OP_GAIN == DISABLED)
#define OP_GAIN_FACTOR                              G_OPAMP_PER_PHASECURRENT
#endif

#define GATE_DRIVER_INPUT_LOGIC                    PASSIVE_HIGH                   /*1. PASSIVE_HIGH       2. PASSIVE_LOW (Please refer the gate driver datasheet) */
#if(GATE_DRIVER_INPUT_LOGIC == PASSIVE_HIGH)
#define MOTOR_COASTING_HIGH_SIDE                    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#define MOTOR_COASTING_LOW_SIDE                     XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW

#define MOTOR_RUN_HIGH_SIDE                         XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#define MOTOR_RUN_LOW_SIDE                          XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#elif(GATE_DRIVER_INPUT_LOGIC == PASSIVE_LOW)
#define MOTOR_COASTING_HIGH_SIDE                    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW
#define MOTOR_COASTING_LOW_SIDE                     XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH

#define MOTOR_RUN_HIGH_SIDE                         XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW
#define MOTOR_RUN_LOW_SIDE                          XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW
#endif

#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
//#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH

#define INVERTER_ENABLE_PIN    (0U)             /* 1 = Active High, 0 Active Low*/

#if(INVERTER_ENABLE_PIN == 0U)
  #define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
  #define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#elif(INVERTER_ENABLE_PIN == 1U)
  #define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
  #define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#endif

#define IU_ADC_BIAS                               (2087)
#define IV_ADC_BIAS                               (2078)
#define IW_ADC_BIAS                               (2061)

/* ********************************************* IFX_XMC_LVPB_R3 ******************************************************************************************************/
#elif(INVERTERCARD_TYPE == CUSTOM_INVERTER)
#define INTERNAL_OP_GAIN                            ENABLED               /*1. ENABLED       2. DISABLED (Please configure OP-Gain manually) */

#if(float_uart == DISABLED)
#define USER_VDC_LINK_V                             (24.7f)                 /* Hardware Inverter VDC link voltage in V  */
#endif

#define USER_CCU8_PWM_FREQ_HZ                       (16000U)              /* CCU8 PWM Switching Frequency in Hz*/
#define USER_DEAD_TIME_US                           (0.3f)   //(2f)              /* deadtime, rise(left) and fall values in us  */

#if(UART_INTERFACE == DISABLED)
#define USER_BOOTSTRAP_PRECHARGE_TIME_MS            (200U)                 /* Initial Bootstrap precharging time in ms */
#endif
//#define USER_BOOTSTRAP_PRECHARGE_TIME_MS            (2000U)                 /* Initial Bootstrap precharging time in ms */
#if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)
#define CFR_Spd_Status_Time                         (3000U)/* In Cycle count. THis value had to be below the total catch time value set in user_config */
#define CFR_BRAKE_TIME_3_MS                         (3000U) /* user time/period. 3s = 4/0.000625=64000 */
#define CFR_BRAKE_TIME_2_MS                         (2000U) /* user time/period. 3s = 4/0.000625=64000 */
#if(UART_INTERFACE == DISABLED)
#define CFR_BRAKE_TIME_1_MS                         (1000U)  /* user time/period. 3s = 4/0.000625=64000 */
#define CFR_ADC_BEMF_UVW_HIGH_LIMIT                 (300U)  /*ADC BEMF high speed threshold to determine bootstrap delay*/
#endif/*#if(UART_INTERFACE == DISABLED)*/
#endif

#if(float_uart == DISABLED)
#define USER_DC_LINK_DIVIDER_RATIO                  (float)(1.6f/(1.6f+10.0f))           /* R1/(R2+R1) ratio for DC link MCU ADC - 1.6(2 stage)*/
//#define USER_DC_LINK_DIVIDER_RATIO                  (float)(2.0f/(2.0f+10.0f))           /* R1/(R2+R1) ratio for DC link MCU ADC  - 2.0(1 stage)*/
#endif

#define USER_VBEMF_RATIO                            (float)(1.6f/(1.6f+10.0f))           /* R1/(R2+R1) ratio for BEMF Voltage sensing circuit ratio */
#define USER_CURRENT_TRIP_THRESHOLD_A               (3.5f)               /* threshold current for trip detection in Ampere*/
#define USER_TRIP_THRESHOLD_TIME_MS                 (0.5U)               /* threshold time for trip detection in ms, changed100to10 */
#define USER_MAX_RETRY_MOTORSTARTUP_TRIP            (3U)                /* Max retry of motor startup if trip  */
/*      --------------------------------------------------- Motor Phase Current Measurement ---------------------------------------- */
#define USER_R_SHUNT_OHM                            (0.1f)               /* Phase shunt resistor in ohm */
#define USER_DC_SHUNT_OHM                           (0.1f)               /* DC link shunt current resistor in ohm */
#define USER_RIN_PHASECURRENT_KOHM                  (1.0f)               /* R_IN (of equivalent amplifier) kohm */
#define USER_R_PHASECURRENT_FEEDBACK_KOHM           (11.0f)                  /* R_FEEDBACK (of equivalent amplifier) kohm */
#define USER_RIN_DCCURRENT_KOHM                     (1.0f)                  /* Rf for dc current sensing */
#define USER_R_DCCURRENT_FEEDBACK_KOHM              (11.0f)                   /* Rin for dc current sensing */
#define USER_MAX_ADC_VDD_V                          (5.0f)                   /* VDD5, maximum voltage at ADC */
#define G_OPAMP_PER_PHASECURRENT                    (USER_R_PHASECURRENT_FEEDBACK_KOHM / (USER_RIN_PHASECURRENT_KOHM+USER_R_PHASECURRENT_FEEDBACK_KOHM))

#if(INTERNAL_OP_GAIN == ENABLED)
#define OP_GAIN_FACTOR                             (6U)                       /* Different HW Board has different OP Gain factor, XMC13 built-in Gain Factor available 1, 3, 6 and 12 only*/
#elif(INTERNAL_OP_GAIN == DISABLED)
#define OP_GAIN_FACTOR                              G_OPAMP_PER_PHASECURRENT
#endif

#define GATE_DRIVER_INPUT_LOGIC                    PASSIVE_LOW                   /*1. PASSIVE_HIGH       2. PASSIVE_LOW (Please refer the gate driver datasheet) */

#if(GATE_DRIVER_INPUT_LOGIC == PASSIVE_HIGH)
#define MOTOR_COASTING_HIGH_SIDE                    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#define MOTOR_COASTING_LOW_SIDE                     XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW

#define MOTOR_RUN_HIGH_SIDE                         XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#define MOTOR_RUN_LOW_SIDE                          XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#elif(GATE_DRIVER_INPUT_LOGIC == PASSIVE_LOW)
#define MOTOR_COASTING_HIGH_SIDE                    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#define MOTOR_COASTING_LOW_SIDE                     XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW

#define MOTOR_RUN_HIGH_SIDE                         XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW
#define MOTOR_RUN_LOW_SIDE                          XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW
#endif

//#define CCU8_INPUT_TRAP_LEVEL                       XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
#define CCU8_INPUT_TRAP_LEVEL                     XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH /*OCP*/

#define INVERTER_ENABLE_PIN                         (0U)             /* 1 = Active High, 0 Active Low*/

#if(INVERTER_ENABLE_PIN == 0U)
#define ENABLE_LEVEL                                XMC_GPIO_OUTPUT_LEVEL_LOW
#define DISABLE_LEVEL                               XMC_GPIO_OUTPUT_LEVEL_HIGH
#elif(INVERTER_ENABLE_PIN == 1U)
#define ENABLE_LEVEL                                XMC_GPIO_OUTPUT_LEVEL_HIGH
#define DISABLE_LEVEL                               XMC_GPIO_OUTPUT_LEVEL_LOW
#endif

#define IU_ADC_BIAS                               (2048)
#define IV_ADC_BIAS                               (2048)
#define IW_ADC_BIAS                               (2048)
#endif

#endif /* PMSM_FOC_CONFIGURATION_PMSM_FOC_INVERTERCARD_PARAMETERS_H_ */
