/**
 * @file pmsm_foc_const_macro.h
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
 * @file pmsm_foc_const_macro.h
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

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_CONST_MARCOS_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_CONST_MARCOS_H_

/*******************************************************************************
 * MACROS
 *******************************************************************************/

/* Hardware kit. Refer to the header file description above */
#define   KIT_XMC1X_AK_MOTOR_001                    (1U)
#define   KIT_XMC750WATT_MC_AK_V1                   (2U)
#define   CUSTOM_KIT                                (8U)
#define   IFCN_XMC_LVPB_R3                          (5U)

/* MCU Card. Refer to the header file description above */
#define   KIT_XMC13_BOOT_001                        (1U)
#define   KIT_XMC1300_DC_V1                         (2U)
#define   CUSTOM_MCU                                (6U)

/* Inverter Card. Refer to the header file description above */
#define   PMSM_LV15W                                (1U)
#define   POWERINVERTER_750W                        (2U)
#define   CUSTOM_INVERTER                           (8U)

/* Motor Type. Refer to the header file description above  */
#define  VORNADO_FAN_MOTOR                          (3U)
#define  NANOTEC_MOTOR                              (4U)
#define  MAXON_MOTOR                                (5U)
#define  EBM_PAPST_VENTI_FAN_MOTOR                  (7U)
#define  CUSTOM_MOTOR                               (8U)

#define  STANDARD_SVM_7_SEGMENT                     (1U)
#define  STANDARD_SVM_5_SEGMENT                     (2U)

/* Current Sensing Feedback Scheme */
#define  USER_THREE_SHUNT_ASSYNC_CONV                (1U)
#define  USER_THREE_SHUNT_SYNC_CONV                  (2U)
#define  USER_SINGLE_SHUNT_CONV                      (3U)
/*      --------------------------------------------------- FOC Control and Startup Scheme  ---------------------------------------- */
#define  SPEED_CONTROLLED_VF_ONLY                   (1U)
#define  SPEED_CONTROLLED_VF_MET_FOC                (2U)
#define  SPEED_CONTROLLED_DIRECT_FOC                (3U)
#define  TORQUE_CONTROLLED_DIRECT_FOC               (4U)
#define  VQ_CONTROLLED_DIRECT_FOC                   (5U)
/*      -----------------------------Motor Startup Configurations  -------------------------------------------------*/
#define ROTOR_IPD_NONE                              (0U)   /* Motor starts without initial position reference */
#define ROTOR_IPD_PRE_ALIGNMENT                     (1U)   /* Rotor pre-alignment and then go to Direct FOC startup */
#define ROTOR_IPD_INDUCTIVE_SENSING                 (2U)   /* Inductive sensing detects initial rotor position */
/*      -----------------------------Over current / Over Voltage Protection Scheme  -------------------------------- */
#define ENABLED                                      (1U)
#define DISABLED                                     (0U)
/*      -----------------------------Gate driver Input logic definition  -------------------------------- */
#define PASSIVE_HIGH                                 (0U)
#define PASSIVE_LOW                                  (1U)
/*      -----------------------------UART USIC Channel Configuration  -------------------------------- */
#define USIC_DISABLED_ALL                            (0U)
#define USIC0_CH0_P1_4_P1_5                          (1U)
#define USIC0_CH1_P1_2_P1_3                          (2U)

#define FALSE                                        (0U)
#define TRUE                                         (1U)
/*      ----------------------------- Reference Speed Adjustment Method  -------------------------------- */
#define MICRIUM_UC_ONLY                              (1U)
#define BY_POT_ONLY                                  (2U)
#define BY_UART_ONLY                                 (3U)                           /* Baudrate: 460800, Data: 8-bit, Parity: None, Stop: 1-bit*/

/*      --------------------------------------------------- SVM with Pseudo Zero Vectors ---------------------------------------- */
#define USER_INVERSE_SVM_LAMDA                      (float)(20.0)
/*      --------------------------------------------------- MCU Parameters ---------------------------------------- */
#define USER_MCLK_FREQ_MHz                          (32U)       /* CPU Clock in Mhz*/
#define USER_PCLK_FREQ_MHz                          (64U)       /* Peripheral CLK frequency = double of CPU Clock */
#define USER_CCU8_PRESCALER                         (1U)
#define USER_CORDIC_MPS                             (2.0f)      /* CORDIC module MPS Setting value */
#define CORDIC_K                                    (1.646760258f)                  /* CORDIC SCALE (Inherent Gain Factor) */
#define CORDIC_SHIFT                                (14U)             /* 8 ~ 16. Shift for CORDIC input / output registers, whose [7:0] are 0x00. Normally no need change.*/
#define CORDIC_CIRCULAR_VECTORING_MODE (0x62) /* CORDIC: Circular Vectoring Mode (default). MPS: Divide by 2 (default).*/
#define CORDIC_CIRCULAR_ROTATION_MODE (0x6A) /* CORDIC: Circular Rotation Mode. MPS: Divide by 2 (default).*/
/*      --------------------------------------------------- Parameters for Startup Lock / Stall Detection ---------------------------------------- */
#define USER_MAX_RETRY_MOTORSTARTUP_STALL           (3U)           /* Max retry of motor startup if stall  */
/*      --------------------------------------------------- MISC Constant ---------------------------------------- */
#define USER_SQRT_3_CONSTANT                        (1.7320508075f)
#define USER_CCU4_DEBUG_KHZ                         (160U)
#define USER_PI                                     (3.1415926536f)
#define SPEED_TO_RPM_SCALE                          (15U)
#define SHIFT_BIAS_LPF                              (3U)                    /* Shift times for unity gain LPF: Y[n] = Y[n-1] + (X[n]-Y[n-1])>>SHIFT_BIAS_LPF. */

#define SQRT3                                       (1.732050807569F)       /* √3 */
#define DIV_SQRT3                                   (591)                  /* ((int16_t)((1/SQRT3) * (1<<SCALE_SQRT3))) */
#define DIV_SQRT3_Q14                               (9459U)
#define SCALE_DIV_3                                 (14U)                   /* For 1/3 scaling. */
#define DIV_3                                       (5461U)                 /* ((int16_t)((1/3) * (1<<SCALE_DIV_3))) */

#define DEGREE_90                                   (4194304U << 8U)        /* 90° angle (0 ~ 2^23 represent electrical angle 0° ~ 180° in CORDIC) */
#define DEGREE_X                                    (DEGREE_90 * 1U)        /* X = 0°, 90°, 180°, or 270° */
#define DEGREE_SHIFT                                (652448U << 8U)         /* 14° angle shift */

#define CORDIC_VECTORING_MODE                       (0x62)                  /* CORDIC: Circular Vectoring Mode (default). MPS: Divide by 2 (default).*/
#define CORDIC_ROTATION_MODE                        (0x6A)                  /*  CORDIC: Circular Rotation Mode. MPS: Divide by 2 (default).*/

#define DIRECTION_INC                               0                       /* Motor rotation direction - rotor angle increasing */

#define ADJUST_NOT_DONE                             0xAB                    /* Parameter adjustment has not been done */
#define ADJUST_DONE                                 0                       /* Parameter adjustment has been done */

#define ACTIVE_HIGH                                 0
#define ACTIVE_LOW                                  1

#define CFR_PHASE_U                                 0
#define CFR_PHASE_V                                 1
#define CFR_PHASE_W                                 2

/*----------------------------------- ROTOR_IPD_INDUCTIVE_SENSING -------------------------------------------*/
#define PMSM_FOC_ANGLE_000_DEGREE_Q31               (0)        			        /* 0° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_030_DEGREE_Q31               (357913941)             /* 30° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_060_DEGREE_Q31               (715827883) 			      /* 60° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_090_DEGREE_Q31               (1073741824)   			    /* 90° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_120_DEGREE_Q31               (1431655765)            /* 120° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_150_DEGREE_Q31               (1789569707)            /* 150° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_180_DEGREE_Q31               (2147483647)            /* 180° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_210_DEGREE_Q31               (-1789569707)           /* 210° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_240_DEGREE_Q31               (-1431655765)           /* 240° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_270_DEGREE_Q31               (-1073741824)           /* 270° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_300_DEGREE_Q31               (-715827883)            /* 300° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_330_DEGREE_Q31               (-357913941)            /* 330° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_INVALID                      (555)

#define PMSM_FOC_ANGLE_045_DEGREE_Q31               (536870912)             /* 45° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */

/*      --------------------------------------------------- UCProbe Update Button  ---------------------------------------- */
#define UCPROBE_BUTTON_PRESSED                      1234
#define UCPROBE_BUTTON_CLEAR                        0

#endif /* PMSM_FOC_CONFIGURATION_PMSM_FOC_CONST_MARCOS_H_ */

