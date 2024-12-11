/**
 * @file pmsm_foc_mcucard_parameters.h
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
 * @file pmsm_foc_mcucard_parameters.h
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

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_MCUCARD_PARAMETERS_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_MCUCARD_PARAMETERS_H_

#include "pmsm_foc_user_config.h"
#include <xmc_vadc.h>
#include <xmc_ccu4.h>
#include <xmc_ccu8.h>
#include <xmc_scu.h>
#include <xmc_gpio.h>
#include <xmc_math.h>
#include <xmc_wdt.h>
#include <xmc_acmp.h>         /*OCP importing xmc acmp function*/
#include "xmc1_gpio_map.h"

/* ********************************************* MINITKIT_XMC1300_V1 ******************************************************************************************************/
#if(MCUCARD_TYPE == MINITKIT_XMC1300_V1)
/*********************************************************************************************************************
 * MINITKIT_XMC1300_V1
 * GPIO Resources Configuration
 ********************************************************************************************************************/
#define TRAP_PIN               P0_12
#define INVERTER_EN_PIN        P0_11

#define PHASE_U_HS_PIN         P0_0
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P0_1
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P0_2
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7

#define PHASE_V_LS_PIN         P0_3
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7

#define PHASE_W_HS_PIN         P0_8
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P0_9
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define TEST_PIN           P1_4

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ********************************************************************************************************************/
#define CCU8_MODULE          CCU80
#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU80_CC81
#define CCU8_MODULE_PHASE_W  CCU80_CC82
#define CCU8_MODULE_ADC_TR   CCU80_CC83

/* TRAP LED blinking period. */
#define LED_BLINK_PRS   (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)

/*********************************************************************************************************************
 * VADC Resources Configuration
 ********************************************************************************************************************/
#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)

/*ADC Asynchrononus Conversion */
#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_RESULT_REG    (3U)

#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (5U)       /* P2.3, VADC group1 channel 5 */
#define VADC_IW_RESULT_REG    (5U)

/*ADC_Synchrononus Conversion */
/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_G0_CHANNEL    (4U)        /* P2.11, VADC group0 channel 4 */

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (5U)       /* P2.3, VADC group1 channel 5 */
#define VADC_IW_G0_CHANNEL    (0xFU)       /* No group 0 for this pin  */

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/* single shunt current VADC define */
#define VADC_ISS_GROUP        VADC_G1
#define VADC_ISS_GROUP_NO     (1U)
#define VADC_ISS_CHANNEL      (4U)       /* P2.9 VADC group1 channel 4 */
#define VADC_ISS_RESULT_REG   (15U)
#endif

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (6U)      /* P2.4 VADC group1 channel 6 */
#define VADC_VDC_RESULT_REG   (6U)

/* DC link current VADC define */
#define VADC_IDC_GROUP        VADC_G0
#define VADC_IDC_GROUP_NO     (0U)
#define VADC_IDC_CHANNEL      (2U)       /* P2.9 VADC group0 channel 2 */
#define VADC_IDC_RESULT_REG   (2U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (7U)      /* P2.5 VADC group1 channel 7 */
#define VADC_POT_RESULT_REG   (7U)

/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_IDC_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_G1_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL

/*********************************************************************************************************************
 * DEBUGGING ENABLE Resources Configuration
 ********************************************************************************************************************/
#define DEBUG_PWM_0_ENABLE      (1U)        /* 1 = Enable Debug PWM P1.0, 0 = Disable Debug PWM */
#define DEBUG_PWM_1_ENABLE      (1U)        /* 1 = Enable Debug PWM P0.4, 0 = Disable Debug PWM */

/*********************************************************************************************************************
 * UART Resources Configuration
 ********************************************************************************************************************/
#if(SETTING_TARGET_SPEED == BY_UART_ONLY)
#define UART_ENABLE             USIC0_CH1_P1_2_P1_3             /* 1. USIC0_CH0_P1_4_P1_5
                                                                   2. USIC0_CH1_P1_2_P1_3 */
#elif(SETTING_TARGET_SPEED == BY_POT_ONLY)
#define UART_ENABLE             USIC_DISABLED_ALL               /* 1. USIC_DISABLED_ALL -- which enable POT ADC speed adjustment */
#endif

/* ********************************************* XMC_PINUS_MCU_V2 ******************************************************************************************************/
#elif(MCUCARD_TYPE == XMC_PINUS_MCU_V2)
/*********************************************************************************************************************
 * IFX_XMC_PINUS_V2
 * GPIO Resources Configuration
 ********************************************************************************************************************/
#define TRAP_PIN               P0_12
#define INVERTER_EN_PIN        P0_11

#define PHASE_U_HS_PIN         P0_0
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P0_1
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P0_2
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7

#define PHASE_V_LS_PIN         P0_3
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7

#define PHASE_W_HS_PIN         P0_8
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P0_9
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define TEST_PIN           P1_4

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ********************************************************************************************************************/
#define CCU8_MODULE          CCU80
#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU80_CC81
#define CCU8_MODULE_PHASE_W  CCU80_CC82
#define CCU8_MODULE_ADC_TR   CCU80_CC83

/* TRAP LED blinking period. */
#define LED_BLINK_PRS   (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)

/*********************************************************************************************************************
 * VADC Resources Configuration
 ********************************************************************************************************************/

#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)


/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_G0_CHANNEL    (4U)        /* P2.11, VADC group0 channel 4 */

#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_RESULT_REG    (3U)

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (4U)       /* P2.9, VADC group1 channel 4 */
#define VADC_IW_G0_CHANNEL    (2U)       /* P2.9, VADC group0 channel 2 */

#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (4U)       /* P2.9, VADC group1 channel 4 */
#define VADC_IW_RESULT_REG    (4U)

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G0
#define VADC_VDC_GROUP_NO     (0U)
#define VADC_VDC_CHANNEL      (1U)      /* P2.4 VADC group1 channel 6 */
#define VADC_VDC_RESULT_REG   (1U)

/* DC link current VADC define */
#define VADC_IDC_GROUP        VADC_G0
#define VADC_IDC_GROUP_NO     (0U)
#define VADC_IDC_CHANNEL      (2U)       /* No use */
#define VADC_IDC_RESULT_REG   (2U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G0
#define VADC_POT_GROUP_NO     (0U)
#define VADC_POT_CHANNEL      (7U)      /* No use */
#define VADC_POT_RESULT_REG   (7U)

/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_VDC_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL

/*********************************************************************************************************************
 * DEBUGGING ENABLE Resources Configuration
 ********************************************************************************************************************/
#define DEBUG_PWM_0_ENABLE      (0U)        /* 1 = Enable Debug PWM P1.0, 0 = Disable Debug PWM */
#define DEBUG_PWM_1_ENABLE      (0U)        /* 1 = Enable Debug PWM P0.4, 0 = Disable Debug PWM */

/*********************************************************************************************************************
 * UART Resources Configuration
 ********************************************************************************************************************/
#if(SETTING_TARGET_SPEED == BY_UART_ONLY)
#define UART_ENABLE             USIC0_CH0_P1_4_P1_5             /* 1. USIC_DISABLED_ALL, -- which enable POT ADC speed adjustment
                                                                   2. USIC0_CH0_P1_4_P1_5
                                                                   3. USIC0_CH1_P1_2_P1_3 */
#endif
/* ********************************************* EVAL_M1_1302 ******************************************************************************************************/
#elif(MCUCARD_TYPE == EVAL_M1_1302)
/*********************************************************************************************************************
 * IFX_MADK_EVAL_M1_05_65D_V1
 * GPIO Resources Configuration
 ********************************************************************************************************************/
#define TRAP_PIN               P0_12
#define INVERTER_EN_PIN        P0_11

#define PHASE_U_HS_PIN         P0_0
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P0_1
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P0_7
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_LS_PIN         P0_6
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_HS_PIN         P0_8
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P0_9
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define TEST_PIN           P0_5

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ********************************************************************************************************************/
#define CCU8_MODULE          CCU80
#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU80_CC81
#define CCU8_MODULE_PHASE_W  CCU80_CC82
#define CCU8_MODULE_ADC_TR   CCU80_CC83

/* TRAP LED blinking period. */
#define LED_BLINK_PRS     (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)

/*********************************************************************************************************************
 * VADC Resources Configuration
 ********************************************************************************************************************/

#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)

/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (4U)        /* P2.9, VADC group1 channel 4 */
#define VADC_IU_G0_CHANNEL    (2U)        /* P2.9, VADC group0 channel 2 */

#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (4U)        /* P2.9, VADC group1 channel 4 */
#define VADC_IU_RESULT_REG    (4U)

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (3U)       /* P2.11, VADC group1 channel 3 */
#define VADC_IW_G0_CHANNEL    (4U)       /* P2.11, VADC group0 channel 4 */

#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (3U)       /* P2.11, VADC group1 channel 3 */
#define VADC_IW_RESULT_REG    (3U)

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/* single shunt current VADC define */
#define VADC_ISS_GROUP        VADC_G1
#define VADC_ISS_GROUP_NO     (1U)
#define VADC_ISS_CHANNEL      (1U)       /* P2.7 VADC group1 channel 1 */
#define VADC_ISS_RESULT_REG   (15U)
#endif

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (5U)      /* P2.3 VADC group1 channel 5 */
#define VADC_VDC_RESULT_REG   (5U)

/* DC link average current VADC define */
#define VADC_IDC_GROUP        VADC_G0
#define VADC_IDC_GROUP_NO     (0U)
#define VADC_IDC_CHANNEL      (6U)       /* P2.1 VADC group0 channel 6 */
#define VADC_IDC_RESULT_REG   (6U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (7U)      /* P2.5 VADC group1 channel 7 */
#define VADC_POT_RESULT_REG   (7U)

/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_IDC_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_G1_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL

/*********************************************************************************************************************
 * DEBUGGING ENABLE Resources Configuration
 ********************************************************************************************************************/
#define DEBUG_PWM_0_ENABLE      (0U)        /* 1 = Enable Debug PWM P1.0, 0 = Disable Debug PWM */
#define DEBUG_PWM_1_ENABLE      (0U)        /* 1 = Enable Debug PWM P0.4, 0 = Disable Debug PWM */
/*********************************************************************************************************************
 * UART Resources Configuration
 ********************************************************************************************************************/
#if(SETTING_TARGET_SPEED == BY_UART_ONLY)
#define UART_ENABLE             USIC0_CH1_P1_2_P1_3              /*1. USIC_DISABLED_ALL -- which enable POT ADC speed adjustment
                                                                   2. USIC0_CH0_P1_4_P1_5
                                                                   3. USIC0_CH1_P1_2_P1_3 */
#endif
/* ********************************************* KIT_XMC13_BOOT_001 ******************************************************************************************************/
#elif(MCUCARD_TYPE == KIT_XMC13_BOOT_001)
/*********************************************************************************************************************
 * KIT_XMC1X_AK_MOTOR_001
 * GPIO Resources Configuration
 ********************************************************************************************************************/
//#define TRAP_PIN               P0_12
//#define INVERTER_EN_PIN        P0_11
#define USE_INVERTER_EN_PIN    0    // means use enable pin or not

#define PHASE_U_HS_PIN         P0_0
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P0_1
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P0_7
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_LS_PIN         P0_6
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_HS_PIN         P0_8
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P0_9
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define TEST_PIN           P0_4
#define adc_pin             P0_10

#if(Remote_Control == ENABLED)
/*For the Remote Control Button Pin */
//#define button_D P0_10
#define button_C               P0_11
#define button_B               P0_12
#define button_A               P0_13
#endif

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ********************************************************************************************************************/
#define CCU8_MODULE          CCU80
#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU80_CC81
#define CCU8_MODULE_PHASE_W  CCU80_CC82
#define CCU8_MODULE_ADC_TR   CCU80_CC83


/* TRAP LED blinking period. */
#define LED_BLINK_PRS     (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)


/*********************************************************************************************************************
 * VADC Resources Configuration
 ********************************************************************************************************************/
#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)

/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_G0_CHANNEL    (4U)        /* P2.11, VADC group0 channel 4 */

#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_RESULT_REG    (3U)

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (4U)       /* P2.9, VADC group1 channel 4 */
#define VADC_IW_G0_CHANNEL    (2U)       /* P2.9, VADC group0 channel 2 */

#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (4U)       /* P2.9, VADC group1 channel 4 */
#define VADC_IW_RESULT_REG    (4U)

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/* single shunt current VADC define */
#define VADC_ISS_GROUP        VADC_G1
#define VADC_ISS_GROUP_NO     (1U)
#define VADC_ISS_CHANNEL      (1U)       /* P2.7 VADC group1 channel 1 */
#define VADC_ISS_RESULT_REG   (15U)
#endif

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (5U)      /* P2.3 VADC group1 channel 5 */
#define VADC_VDC_RESULT_REG   (5U)

/* DC link current VADC define */
#define VADC_IDC_GROUP        VADC_G1
#define VADC_IDC_GROUP_NO     (1U)
#define VADC_IDC_CHANNEL      (6U)       /* P2.4 VADC group1 channel 6 */
#define VADC_IDC_RESULT_REG   (6U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (7U)      /* P2.5 VADC group1 channel 7 */
#define VADC_POT_RESULT_REG   (7U)

/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_IDC_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_G1_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL

#if(ACMP_OCP == ENABLED)
/****** (ACMP Instances Parameters) ---OCP *******/
/*Setting the use of which acmp_x instance: 0,1,2*/
/* P2.8 acmp0.n | P2.9 acmp0.p*/
/* P2.6 acmp1.n | P2.7 acmp1.p*/
/* P2.2 acmp2.n | P2.1 acmp2.p*/
#define OC_ACMP_instance  2
#endif

/*********************************************************************************************************************
 * DEBUGGING ENABLE Resources Configuration
 ********************************************************************************************************************/
#define DEBUG_PWM_0_ENABLE      (0U)        /* 1 = Enable Debug PWM P1.0, 0 = Disable Debug PWM */
#define DEBUG_PWM_1_ENABLE      (0U)        /* 1 = Enable Debug PWM P0.4, 0 = Disable Debug PWM */

/*********************************************************************************************************************
 * UART Resources Configuration
 ********************************************************************************************************************/
#if(SETTING_TARGET_SPEED == BY_UART_ONLY)
#define UART_ENABLE             USIC0_CH1_P1_2_P1_3             /* 1. USIC0_CH0_P1_4_P1_5
                                                                   2. USIC0_CH1_P1_2_P1_3 */
#elif(SETTING_TARGET_SPEED == BY_POT_ONLY)
#define UART_ENABLE             USIC_DISABLED_ALL               /* 1. USIC_DISABLED_ALL -- which enable POT ADC speed adjustment */
#endif
/* ********************************************* KIT_XMC1300_DC_V1 ******************************************************************************************************/
#elif(MCUCARD_TYPE == KIT_XMC1300_DC_V1)
/*********************************************************************************************************************
 * KIT_XMC1300_DC_V1
 * GPIO Resources Configuration
 ********************************************************************************************************************/
#define TRAP_PIN               P0_12
#define INVERTER_EN_PIN        P0_11

#define PHASE_U_HS_PIN         P0_0
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P0_1
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P0_7
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_LS_PIN         P0_6
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_HS_PIN         P0_8
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P0_9
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define TEST_PIN           P0_4

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ********************************************************************************************************************/
#define CCU8_MODULE          CCU80
#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU80_CC81
#define CCU8_MODULE_PHASE_W  CCU80_CC82
#define CCU8_MODULE_ADC_TR   CCU80_CC83

/* TRAP LED blinking period. */
#define LED_BLINK_PRS     (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)


/*********************************************************************************************************************
 * VADC Resources Configuration
 ********************************************************************************************************************/
#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)

/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (4U)        /* P2.9, VADC group1 channel 4 */
#define VADC_IU_G0_CHANNEL    (2U)        /* P2.9, VADC group0 channel 2 */

#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (4U)        /* P2.9, VADC group1 channel 4 */
#define VADC_IU_RESULT_REG    (4U)

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (3U)       /* P2.11, VADC group1 channel 3 */
#define VADC_IW_G0_CHANNEL    (4U)       /* P2.11, VADC group0 channel 4 */

#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (3U)       /* P2.11, VADC group1 channel 3 */
#define VADC_IW_RESULT_REG    (3U)

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/* single shunt current VADC define */
#define VADC_ISS_GROUP        VADC_G1
#define VADC_ISS_GROUP_NO     (1U)
#define VADC_ISS_CHANNEL      (1U)       /* P2.7 VADC group1 channel 1 */
#define VADC_ISS_RESULT_REG   (15U)
#endif

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (5U)      /* P2.3 VADC group1 channel 5 */
#define VADC_VDC_RESULT_REG   (5U)

/* DC link average current VADC define */
#define VADC_IDC_GROUP        VADC_G0
#define VADC_IDC_GROUP_NO     (0U)
#define VADC_IDC_CHANNEL      (6U)       /* P2.1 VADC group0 channel 6 */
#define VADC_IDC_RESULT_REG   (6U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (7U)      /* P2.5 VADC group1 channel 7 */
#define VADC_POT_RESULT_REG   (7U)


/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_IDC_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_G1_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL

/*********************************************************************************************************************
 * DEBUGGING ENABLE Resources Configuration
 ********************************************************************************************************************/
#define DEBUG_PWM_0_ENABLE      (0U)        /* 1 = Enable Debug PWM P1.0, 0 = Disable Debug PWM */
#define DEBUG_PWM_1_ENABLE      (0U)        /* 1 = Enable Debug PWM P0.4, 0 = Disable Debug PWM */
/*********************************************************************************************************************
 * UART Resources Configuration
 ********************************************************************************************************************/
#if(SETTING_TARGET_SPEED == BY_UART_ONLY)
#define UART_ENABLE             USIC0_CH1_P1_2_P1_3              /* 1. USIC_DISABLED_ALL, -- which enable POT ADC speed adjustment
                                                                   2. USIC0_CH0_P1_4_P1_5
                                                                   3. USIC0_CH1_P1_2_P1_3 */
#endif
/* ********************************************* CUSTOM_MCU ******************************************************************************************************/
#elif(MCUCARD_TYPE == CUSTOM_MCU)
/*********************************************************************************************************************
 * CUSTOM_MCU
 * GPIO Resources Configuration
 ********************************************************************************************************************/
//#define TRAP_PIN               P0_12    /*NOT used in Design*/
//#define INVERTER_EN_PIN        P0_11    /*NOT used in Design*/
#define USE_INVERTER_EN_PIN    0    // means use enable pin or not

#define PHASE_U_HS_PIN         P0_0
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P0_1
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P0_8
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_LS_PIN         P0_9
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_HS_PIN         P0_7
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P0_6
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

/*********************************************************************************************************************
 * Testing Purpose
 ********************************************************************************************************************/
#if (Test_point == ENABLED)
#define TEST_PIN           P0_13
#endif

#if (Protection_Indicator ==  ENABLED)
#define LED_protection     P0_3
#endif

#if (MCU_Load_time_test == ENABLED)
#define mcu_load           P0_13
#endif
/*********************************************************************************************************************
 * Remote Control PIN Configuration
 ********************************************************************************************************************/
#if(IR_Remote_Control == ENABLED)
/*For the IR Remote Control Receive Pin */
//#define recvpin        P0_2
//#define IR_LED_Testing     P0_1
//#define IR_LED_Testing2    P0_7
//#define IR_LED_Testing3    P0_9

#endif

#if(Remote_Control == ENABLED)
/*For the Remote Control Button Pin */
//#define button_D               P0_13
#define button_C               P0_12
#define button_B               P0_11
#define button_A               P0_10
#endif

/*********************************************************************************************************************
 * Sleep Mode Pin Configuration
 ********************************************************************************************************************/
#if(MCU_Sleep_Mode == ENABLED)
#define ERU_sleepmode          P2_9
#endif
/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ********************************************************************************************************************/
#if(USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)
#define CCU8_MODULE             CCU80
#define CCU8_MODULE_NUM         (0U)

#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_SLICE_PHASE_U_NUM  (0U)

#define CCU8_MODULE_PHASE_V  CCU80_CC82
#define CCU8_SLICE_PHASE_V_NUM  (2U)

#define CCU8_MODULE_PHASE_W  CCU80_CC81
#define CCU8_SLICE_PHASE_W_NUM  (1U)

#define CCU8_MODULE_ADC_TR   CCU80_CC83
#define CCU8_SLICE_ADC_TR_NUM   (3U)
#else
#define CCU8_MODULE          CCU80
#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU80_CC82
#define CCU8_MODULE_PHASE_W  CCU80_CC81
#define CCU8_MODULE_ADC_TR   CCU80_CC83
#endif


/* TRAP LED blinking period. */
#define LED_BLINK_PRS     (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)


/*********************************************************************************************************************
 * VADC Resources Configuration
 ********************************************************************************************************************/
// Remapping ADC
/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (4U)        /* P2.9, VADC group1 channel 4 */
#define VADC_IW_G0_CHANNEL    (2U)        /* P2.9, VADC group0 channel 2 */

#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (4U)        /* P2.9, VADC group1 channel 4 */
#define VADC_IW_RESULT_REG    (4U)

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (3U)       /* P2.11, VADC group1 channel 3 */
#define VADC_IU_G0_CHANNEL    (4U)       /* P2.11, VADC group0 channel 4 */

#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (3U)       /* P2.11, VADC group1 channel 3 */
#define VADC_IU_RESULT_REG    (3U)

// remap the adc
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/* single shunt current VADC define */
#define VADC_ISS_GROUP        VADC_G1
#define VADC_ISS_GROUP_NO     (1U)
#define VADC_ISS_CHANNEL      (5U)      /* P2.3 VADC group1 channel 5 */
#define VADC_ISS_RESULT_REG   (5U)
#endif

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (7U)        /* P2.5 VADC group1 channel 7 */
#define VADC_VDC_RESULT_REG   (7U)

/* DC link average current VADC define */
#define VADC_IDC_GROUP        VADC_G0
#define VADC_IDC_GROUP_NO     (0U)        /* This pin is used for OCP OCth */
#define VADC_IDC_CHANNEL      (6U)        /* P2.1 VADC group0 channel 6 */
#define VADC_IDC_RESULT_REG   (6U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (6U)        /* P2.4 VADC group1 channel 6 */
#define VADC_POT_RESULT_REG   (6U)

#if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)
/*************************** BEMF detection *************************************************************************/
/* Make sure all are from same group. If not then scan trigger both the groups for conversion */
#define new_board_2023                ENABLED /* 1. ENABLED 2. DISABLED*/
#if(new_board_2023 ==ENABLED)
/*New Board 2023(integrated gate driver)*/
/* BEMF_U VADC define*/
#define VADC_BEMF_U_GROUP         VADC_G0 /* #P2.8 - Grp 0, Ch 1*/
#define VADC_BEMF_U_CHANNEL       (1U)
#define VADC_BEMF_U_RESULT_REG    (1U)

/* BEMF_V VADC define*/
#define VADC_BEMF_V_GROUP         VADC_G0 /* #P2.6 - Grp 0, Ch 0*/
#define VADC_BEMF_V_CHANNEL       (0U)
#define VADC_BEMF_V_RESULT_REG    (0U)

/* BEMF_W VADC define*/
#define VADC_BEMF_W_GROUP         VADC_G1 /* #P2.7 - Grp 1, Ch 1*/
#define VADC_BEMF_W_CHANNEL       (1U)
#define VADC_BEMF_W_RESULT_REG    (1U)

#else
/*Old CF Board 2022*/
/* BEMF_U VADC define*/
#define VADC_BEMF_U_GROUP         VADC_G0 /* P2.2 VADC group0 channel 7 #P2.8*/
#define VADC_BEMF_U_CHANNEL       (1U)
#define VADC_BEMF_U_RESULT_REG    (1U)

/* BEMF_V VADC define*/
#define VADC_BEMF_V_GROUP         VADC_G1 /* P2.1 VADC group0 channel 6 #P2.7*/
#define VADC_BEMF_V_CHANNEL       (1U)
#define VADC_BEMF_V_RESULT_REG    (1U)

/* BEMF_W VADC define*/
#define VADC_BEMF_W_GROUP         VADC_G0 /* P2.0 VADC group0 channel 5 #P2.6*/
#define VADC_BEMF_W_CHANNEL       (0U)
#define VADC_BEMF_W_RESULT_REG    (0U)
#endif
#endif//#if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)

/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_IU_G0_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_G1_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IDC_CHANNEL

#if(ACMP_OCP == ENABLED)
/****** (ACMP Instances Parameters) ---OCP *******/
/*Setting the use of which acmp_x instance: 0,1,2*/
/* P2.8 acmp0.n | P2.9 acmp0.p*/
/* P2.6 acmp1.n | P2.7 acmp1.p*/
/* P2.2 acmp2.n | P2.1 acmp2.p*/
#define OC_ACMP_instance  2
#endif

/*********************************************************************************************************************
 * DEBUGGING ENABLE Resources Configuration
 ********************************************************************************************************************/
#define DEBUG_PWM_0_ENABLE      (0U)        /* 1 = Enable Debug PWM P1.0, 0 = Disable Debug PWM */
#define DEBUG_PWM_1_ENABLE      (0U)        /* 1 = Enable Debug PWM P0.4, 0 = Disable Debug PWM */
/*********************************************************************************************************************
 * UART Resources Configuration
 ********************************************************************************************************************/
#if(SETTING_TARGET_SPEED == BY_UART_ONLY)
#define UART_ENABLE             USIC0_CH1_P1_2_P1_3              /* 1. USIC_DISABLED_ALL, -- which enable POT ADC speed adjustment
                                                                   2. USIC0_CH0_P1_4_P1_5
                                                                   3. USIC0_CH1_P1_2_P1_3 */
#elif(SETTING_TARGET_SPEED == BY_POT_ONLY)
#define UART_ENABLE             USIC_DISABLED_ALL               /* 1. USIC_DISABLED_ALL -- which enable POT ADC speed adjustment */
#endif
#endif

/* NVIC Interrupt Resources Configuration */
/* ********************************************************************************************************************/
#define pmsm_foc_controlloop_isr                        CCU80_0_IRQHandler
#define pmsm_foc_trap_protection_irq                    CCU80_1_IRQHandler
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  #if (VADC_ISS_GROUP_NO == 0)
    #define pmsm_foc_vadc_source_irqhandler              VADC0_G0_1_IRQHandler
  #else
    #define pmsm_foc_vadc_source_irqhandler              VADC0_G1_1_IRQHandler
  #endif
#endif
/* ********************************************************************************************************************/
/* NVIC Interrupt Priority Configuration */
/* ********************************************************************************************************************/
/* Interrupt priority configurations - 0 is the highest priority and 3 is the lowest */
#if (USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)
#define PMSM_FOC_ADC_NVIC_PRIO                          (1U)  /*!< ADC interrupt priority */
#define PMSM_FOC_FCL_NVIC_PRIO                          (2U)  /*!< FAST Control loop - Executed every PWM period */
#define PMSM_FOC_SCL_NVIC_PRIO                          (3U)  /*!< Slow Control loop  - SysTick */
#define PMSM_FOC_CTRAP_NVIC_PRIO                        (0U)  /*!< CTRAP   */
#define PMSM_FOC_FAULT_NVIC_PRIO                        (1U)  /*!< nFault from 6EDL7141 */
#endif
/* ********************************************************************************************************************/
/** POSIF module */
/* ********************************************************************************************************************/
#if (USER_ROTOR_IPD_METHOD == ROTOR_IPD_INDUCTIVE_SENSING)
#define PMSM_FOC_POSIF_MODULE                          (POSIF0)
#endif
/*********************************************************************************************************************
 * CCU4 DAC Debug
 ********************************************************************************************************************/
#if (DEBUG_PWM_0_ENABLE == 1U)|| (DEBUG_PWM_1_ENABLE == 1U)
#define DEBUG_PWM_CCU4_MODULE   CCU40


  /* Debug Period Value controls the resolution of the PWM.
   * This is the value that goes into the PWM period register.
   */
  #define DEBUG_PWM_PERIOD_CNTS (400U)

  /* Initial Duty Cycle of Debug PWM Channels */
  #define DEBUG_PWM_50_PERCENT_DC_CNTS  ((uint16_t)(DEBUG_PWM_PERIOD_CNTS >> 1))

#if (DEBUG_PWM_0_ENABLE == 1U)
  #define DEBUG_PWM_0_SLICE                         CCU40_CC40
  #define DEBUG_PWM_0_SLICE_NUM                     (0U)
  #define DEBUG_PWM_0_SLICE_SHADOW_TRANS_ENABLE_Msk XMC_CCU4_SHADOW_TRANSFER_SLICE_0
  #define DEBUG_PWM_0_PORT                          XMC_GPIO_PORT1
  #define DEBUG_PWM_0_PIN                           (0U)
  #define DEBUG_PWM_0_ALT_OUT                       XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2
#endif  /*DEBUG_PWM_0_ENABLE == 1*/

#if (DEBUG_PWM_1_ENABLE == 1U)
  #define DEBUG_PWM_1_SLICE                         CCU40_CC41
  #define DEBUG_PWM_1_SLICE_NUM                     (1U)
  #define DEBUG_PWM_1_SLICE_SHADOW_TRANS_ENABLE_Msk XMC_CCU4_SHADOW_TRANSFER_SLICE_1
  #define DEBUG_PWM_1_PORT                          XMC_GPIO_PORT0
  #define DEBUG_PWM_1_PIN                           (4U)
  #define DEBUG_PWM_1_ALT_OUT                       XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT4
#endif  /*DEBUG_PWM_1_ENABLE == 1 */

#endif /*(DEBUG_PWM_0_ENABLE == 1U)|| (DEBUG_PWM_1_ENABLE == 1U) */

  /* Tmp_CRS = 0 or (- Tmp_CRS) if Tmp_CRS < 0. */
  #define REVERSE_CRS_OR_0  (- Tmp_CRS)

#endif /* PMSM_FOC_CONFIGURATION_PMSM_FOC_MCUCARD_PARAMETERS_H_ */
