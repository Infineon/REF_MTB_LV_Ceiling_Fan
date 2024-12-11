/**
 * @file pmsm_foc_uart.c
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
 * @file pmsm_foc_debug.c
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
#include "pmsm_foc_uart.h"
#include "xmc1_flash.h"
#include "serial.h"
#include "shell.h"
#include <stdlib.h>               /*For strtol | strtof*/
#include "..\Configuration\pmsm_foc_variables_scaling.h"

/*===================== Uart Interface ================*/
#if(UART_INTERFACE == ENABLED)
extern void serial_init(void);

/*All variable declaration*/
char *remaining;
uint32_t Recv_2nd_Str;
uint32_t user_para_pos;
uint32_t flash_update;
uint32_t value_range_error;
uint32_t check_variable;
float check_variable_2;
uint32_t uart_para_limit[70][2];          /*Flash Array size - depend how many parameter needed to be stored in flash*/

/*=========================================== ALL TUNING PARAMETER COMMAND ===============================================*/
void Update_Para_cmd(int32_t argc, char **argv)
{
  shell_println("Updating Parameters. System reset.");
  __disable_irq();
  uint32_t *MotorCONF_Address = MotorConfig_Addr;
  uint32_t *UserConfig_Address;
  UserConfig_Address = &User_Para[0];
  /*This function erases, writes and verifies the page indicated by the Address parameter. */
  XMC_FLASH_ProgramVerifyPage(MotorCONF_Address,UserConfig_Address);
  NVIC_SystemReset();
}

void Para_cmd(int32_t argc, char **argv)
{
  if (argc == 2)
  {
	user_para_pos = strtol(argv[0], &remaining, 10);
    Recv_2nd_Str = strtol(argv[1], &remaining, 10);
    if((Recv_2nd_Str >= uart_para_limit[user_para_pos][0]) && (Recv_2nd_Str <= uart_para_limit[user_para_pos][1]) && user_para_pos!=58)
    {
        flash_update = 0;
        shell_println("Previous %s value: %d", argv[0], User_Para[user_para_pos]);
        shell_println("Int value received: %d", Recv_2nd_Str);
        User_Para[user_para_pos] = Recv_2nd_Str;
    }
    else if((Recv_2nd_Str >= uart_para_limit[user_para_pos][0]) && (Recv_2nd_Str <= uart_para_limit[user_para_pos][1]) && user_para_pos==58)
    {
        flash_update = 0;
        shell_println("Previous %s value: %d", argv[0], User_Para[user_para_pos]);
        shell_println("Int value received: %d", Recv_2nd_Str);
        User_Para[user_para_pos] = Recv_2nd_Str;
        shell_println("Updating Parameters. System reset.");
        NVIC_DisableIRQ(CCU80_0_IRQn);
        NVIC_DisableIRQ(VADC0_G1_1_IRQn);  //VADC interrupt
        uint32_t *MotorCONF_Address = MotorConfig_Addr;
        uint32_t *UserConfig_Address;
        UserConfig_Address = &User_Para[0];
        XMC_FLASH_ProgramVerifyPage(MotorCONF_Address,UserConfig_Address);
        NVIC_SystemReset();
    }
    else
    {
        value_range_error = 0;
        shell_println("Argument not supported, please re-enter value within range");
    }
  }
}

/*======================================= END ALL TUNING PARAMETER COMMAND ===============================================*/
void help_cmd(int32_t argc, char **argv)
{
  (void)argc;
  (void)argv;

  shell_help();
}

#if(BMIresetting == ENABLED)
void BMI_cmd(int32_t argc, char **argv)
{
  if (strcmp(argv[0], "resetBMI") == 0)
  {
    shell_println("Resetting BMI...");

    XMC1000_BmiInstallationReq(0xFFC0); // BMI = ASC_BSL
  }
}
#endif

void display_cmd(int32_t argc, char **argv)
{
  if (strcmp(argv[0], "displaypi") == 0)
  {
    shell_println("\r\nPI PARAMETER----------------------------");
    shell_println("Spdkp : %d, spdki : %d, spdscale : %d", User_Para[1], User_Para[2], User_Para[3]);
    shell_println("Torkp : %d, Torki : %d, Torscale : %d", User_Para[4], User_Para[5], User_Para[6]);
    shell_println("Fluxkp: %d, Fluxki: %d, Fluxscale: %d", User_Para[7], User_Para[8], User_Para[9]);
    shell_println("PLLkp : %d, PLLki : %d, PLLscale : %d", User_Para[10], User_Para[11], User_Para[12]);
  }
  if (strcmp(argv[0], "displaycfr") == 0)
  {
    shell_println("\r\nCATCH FREE RUN--------------------------");
    shell_println("CFR_ADC_BEMF_UVW_HIGH_LIMIT: %d",User_Para[13]);
  }
  if (strcmp(argv[0], "displaybrake") == 0)
  {
    shell_println("\r\nBRAKE BOOTSTRAP-------------------------");
    shell_println("Brake1: %d, Bootstrap: %d", User_Para[14],User_Para[15]);
  }
  if (strcmp(argv[0], "displaypre") == 0)
  {
    shell_println("\r\nPRE-ALIGNMENT---------------------------");
    shell_println("Pre time: %d, inc_step: %d", User_Para[16],User_Para[17]);
  }
  if (strcmp(argv[0], "displaynorstart") == 0)
  {
    shell_println("\r\nNORMAL STARTUP--------------------------");
    shell_println("Ref_iq: %d, Iq_max: %d, slew_rate: %d", User_Para[18],User_Para[19],User_Para[20]);
  }
  if (strcmp(argv[0], "displaycfrstart") == 0)
  {
    shell_println("\r\nCFR STARTUP-----------------------------");
    shell_println("Ref_iq: %d, Iq_max: %d, slew_rate: %d", User_Para[21],User_Para[22],User_Para[23]);
  }
  if (strcmp(argv[0], "displayrc") == 0)
  {
    shell_println("\r\nRemote Control-----------------------------");
    shell_println("Current_Ref_iq: %d", User_Para[24]);
    shell_println("Spd 1_Ref_iq: %d, Spd 2_Ref_iq: %d, Spd 3_Ref_iq: %d", User_Para[25],User_Para[26],User_Para[27]);
    shell_println("Spd 4_Ref_iq: %d, Spd 5_Ref_iq: %d, Spd 6_Ref_iq: %d", User_Para[28],User_Para[29],User_Para[30]);
  }
  if (strcmp(argv[0], "displayocp") == 0)
  {
    shell_println("\r\nOver Current Protection-----------------------------");
    shell_println("OCP: %d", User_Para[31]);
  }

#if(auto_start == ENABLED)
  if (strcmp(argv[0], "displaypower") == 0)
  {
    shell_println("\r\nPower Disruption Parameter-----------------------------");
    shell_println("Power Disruption Flag Status: %d", User_Para[60]);
    shell_println("Last Fan Speed:               %d", User_Para[61]);
  }
#endif

  if (strcmp(argv[0], "displayrctype") == 0)
  {
    shell_println("\r\nRemote Control Type-----------------------------");
    shell_println("RC - TYPE(0 - IR | 1 - RF) : %d", User_Para[58]);
  }

  if (strcmp(argv[0], "displaymisc") == 0)
  {
    shell_println("\r\nMISC -----------------------------");
    shell_println("Fault retry : %d", User_Para[56]);
    shell_println("Sleeptime count : %d", User_Para[57]);
    shell_println("Motor Max Speed : %d", User_Para[59]);
  }
}

/* ---------------------------------- cmd Table -------------------------------------------------- */
/* *name      - Command name (null-terminated string).
 *  min_args    - Minimum number of arguments the command accepts.
 *  max_args    - Maximum number of arguments the command accepts.
 *  cmd_ptr     - pointer to the actual command function defined by: the @ref shell_cmd_function_t type
 *  *description  - Brief description of the command (null-terminated string): This field is used by the @ref shell_help() function.@n
 *  *syntax     - * This field is used by the @ref shell_help() function.
          * The standard command line syntax information
          * which will be helpful to describe the possible command
          * line parameters in a help display is:
          * - @c [] = When a parameter is surrounded with square
          * brackets, this means the parameter is optional.
          * - @c <> = When a parameter is surrounded with angle
          * brackets, this means the parameter is required for
          * normal operations of command.
          * - @c | = The vertical bar means a choice between
          * parameter value is acceptable.
 *  */
const shell_command_t cmd_table[] =
{
  /*command name | min_args | max_args | cmd_ptr | description | syntax*/
  {"help"             , 0u, 0u, help_cmd        , "Display this help message", ""},
  {"update"           , 0u, 0u, Update_Para_cmd , "Update parameter value to flash and system reset", ""},
  {"displaypi"        , 0u, 0u, display_cmd , "Display PI parameter value", ""},
  {"displaycfr"       , 0u, 0u, display_cmd , "Display CFR parameter value", ""},
  {"displaybrake"     , 0u, 0u, display_cmd , "Display Bootstrap brake time parameter value", ""},
  {"displaypre"       , 0u, 0u, display_cmd , "Display pre-alignment parameter value", ""},
  {"displaynorstart"  , 0u, 0u, display_cmd , "Display normal startup parameter value", ""},
  {"displaycfrstart"  , 0u, 0u, display_cmd , "Display cfr startup parameter value", ""},
  {"displayrc"        , 0u, 0u, display_cmd , "Display remote control parameter value", ""},
  {"displayocp"       , 0u, 0u, display_cmd , "Display ocp parameter value", ""},
  {"displaypower"     , 0u, 0u, display_cmd , "Display power disruption / fan level parameter value", ""},
  {"displayrctype"    , 0u, 0u, display_cmd , "Display Remote Control Type status", ""},
  {"displaymisc"      , 0u, 0u, display_cmd , "Display Fault_retry | Sleepmode time | Motor Max Speed", ""},
  /*=== BMI reset ===*/
  {"resetBMI"   , 0u, 0u, BMI_cmd , "BMI reset to Default",""},
  /*=== PI Parameter ===*/
  {"1"      , 1u, 1u, Para_cmd  , "PI_SPEED_KP"         , "1 to 65535"},
  {"2"      , 1u, 1u, Para_cmd  , "PI_SPEED_KI"         , "1 to 65535"},
  {"3"   , 1u, 1u, Para_cmd  , "PI_SPEED_SCALE_KPKI" , "1 to 25"},
  {"4"      , 1u, 1u, Para_cmd  , "PI_TORQUE_KP"        , "1 to 65535"},
  {"5"      , 1u, 1u, Para_cmd  , "PI_TORQUE_KI"        , "1 to 65535"},
  {"6"   , 1u, 1u, Para_cmd  , "PI_TORQUE_SCALE_KPKI", "1 to 25"},
  {"7"     , 1u, 1u, Para_cmd  , "PI_FLUX_KP"          , "1 to 65535"},
  {"8"     , 1u, 1u, Para_cmd  , "PI_FLUX_KI"          , "1 to 65535"},
  {"9"  , 1u, 1u, Para_cmd  , "PI_FLUX_SCALE_KPKI"  , "1 to 25"},
  {"10"      , 1u, 1u, Para_cmd  , "PI_PLL_KP"           , "1 to 65535"},
  {"11"      , 1u, 1u, Para_cmd  , "PI_PLL_KI"           , "1 to 65535"},
  {"12"   , 1u, 1u, Para_cmd  , "PI_PLL_SCALE_KPKI"   , "1 to 25"},
  /*=== Startup Parameter ===*/
  /*Catch Free Run (1)*/
  {"13" , 1u, 1u, Para_cmd  , "CFR_ADC_BEMF_UVW_HIGH_LIMIT"     , "1 to 12000"},
  /*Brake Bootstrap Brake time (2)*/
  {"14"     , 1u, 1u, Para_cmd  , "CFR_BRAKE_TIME_3_MS"             , "1 to 12000"},
  {"15"     , 1u, 1u, Para_cmd  , "USER_BOOTSTRAP_PRECHARGE_TIME_MS", "1 to 12000"},
  /*Pre-Alignment (2)*/
  {"16"    , 1u, 1u, Para_cmd  , "USER_ROTOR_PREPOSITION_TIME_MS"  , "1 to 12000"},
  {"17"    , 1u, 1u, Para_cmd  , "VREF_INC_STEP"                   , "1 to 50"},
  /*Normal Startup (3)*/
  {"18"    , 1u, 1u, Para_cmd  , "Startup_Torque_Ref_Iq"           , "1 to 32767"},
  {"19" , 1u, 1u, Para_cmd  , "Startup_Torque_Ref_Iq_max"       , "1 to 32767"},
  {"20"  , 1u, 1u, Para_cmd  , "Startup_slew_rate "              , "1 to 50"},
  /*CFR Startup (3)*/
  {"21"      , 1u, 1u, Para_cmd  , "CFR_Startup_Ref_Iq"              , "1 to 32767"},
  {"22"   , 1u, 1u, Para_cmd  , "CFR_Startup_Torque_Ref_Iq_max"   , "1 to 32767"},
  {"23"    , 1u, 1u, Para_cmd  , "CFR_Startup_slew_rate"           , "1 to 50"},
  /*=== Remote Control Parameter ===*/
  /*Remote Control (7)*/
  {"24"       , 1u, 1u, Para_cmd  , "torque_current_ref_iq" , "1 to 32767"},
  {"25"     , 1u, 1u, Para_cmd  , "torque_spd_1_ref_iq  " , "1 to 32767"},
  {"26"     , 1u, 1u, Para_cmd  , "torque_spd_2_ref_iq  " , "1 to 32767"},
  {"27"     , 1u, 1u, Para_cmd  , "torque_spd_3_ref_iq  " , "1 to 32767"},
  {"28"     , 1u, 1u, Para_cmd  , "torque_spd_4_ref_iq  " , "1 to 32767"},
  {"29"     , 1u, 1u, Para_cmd  , "torque_spd_5_ref_iq  " , "1 to 32767"},
  {"30"     , 1u, 1u, Para_cmd  , "torque_spd_6_ref_iq  " , "1 to 32767"},
  /*=== MISC Parameter ===*/
  /*Over Current Protection (1)*/
  {"31"                , 1u, 1u, Para_cmd  , "I_OCP_SW"               , "1 to 32767"},
  /*Extra paremeters for fine tuning*/
  {"56"  , 1u, 1u, Para_cmd  , "FAULT_MAX_RETRY_COUNT"  , "1 to 25"},
  {"57"     , 1u, 1u, Para_cmd  , "sleepmode_countertime"  , "1 to 32767"},
  {"58"             , 1u, 1u, Para_cmd  , "Remote Control Type"    , "0 to 1"},  /*(User_Para[58]: 1 RF | 0 IR)*/
  {"59"      , 1u, 1u, Para_cmd  , "Motor Max Speed"    , "0 to 1000"},
  {0, 0u, 0u, 0, 0, 0}
};

/*--------------- Shell Display Start Menu shown on Terminal Program ------------------------------*/
void my_shell_init(void)
{
	shell_println("\r\n Please input string command. Enter 'help' for command list.");
}

void uart_config_init(void)
{
      /*Working on Flash*/
      uint32_t *MotorCONF_Addr = MotorConfig_Addr;
      User_Para[0] = *MotorCONF_Addr;
      User_Para[1] = *(MotorCONF_Addr+1); /*spd kp*/
      User_Para[2] = *(MotorCONF_Addr+2); /*spd ki*/
      User_Para[3] = *(MotorCONF_Addr+3); /*spd scale*/
      User_Para[4] = *(MotorCONF_Addr+4); /*tor kp*/
      User_Para[5] = *(MotorCONF_Addr+5); /*tor ki*/
      User_Para[6] = *(MotorCONF_Addr+6); /*tor scale*/
      User_Para[7] = *(MotorCONF_Addr+7); /*flux kp*/
      User_Para[8] = *(MotorCONF_Addr+8); /*flux ki*/
      User_Para[9] = *(MotorCONF_Addr+9); /*flux scale*/
      User_Para[10] = *(MotorCONF_Addr+10); /*pll kp*/
      User_Para[11] = *(MotorCONF_Addr+11); /*pll ki*/
      User_Para[12] = *(MotorCONF_Addr+12); /*pll scale*/
      User_Para[13] = *(MotorCONF_Addr+13); /*cfr adc bemf*/
      User_Para[14] = *(MotorCONF_Addr+14); /*brake1*/
      User_Para[15] = *(MotorCONF_Addr+15); /*brake0*/
      User_Para[16] = *(MotorCONF_Addr+16); /*pretime*/
      User_Para[17] = *(MotorCONF_Addr+17); /*prestep*/
      User_Para[18] = *(MotorCONF_Addr+18); /*startiq*/
      User_Para[19] = *(MotorCONF_Addr+19); /*startiqmax*/
      User_Para[20] = *(MotorCONF_Addr+20); /*startslew*/
      User_Para[21] = *(MotorCONF_Addr+21); /*cfriq*/
      User_Para[22] = *(MotorCONF_Addr+22); /*cfriqmax*/
      User_Para[23] = *(MotorCONF_Addr+23); /*cfrslew*/
      User_Para[24] = *(MotorCONF_Addr+24); /*rciq*/
      User_Para[25] = *(MotorCONF_Addr+25); /*rcspd1*/
      User_Para[26] = *(MotorCONF_Addr+26); /*rcspd2*/
      User_Para[27] = *(MotorCONF_Addr+27); /*rcspd3*/
      User_Para[28] = *(MotorCONF_Addr+28); /*rcspd4*/
      User_Para[29] = *(MotorCONF_Addr+29); /*rcspd5*/
      User_Para[30] = *(MotorCONF_Addr+30); /*rcspd6*/
      User_Para[31] = *(MotorCONF_Addr+31); /*OCP*/
      User_Para[32] = *(MotorCONF_Addr+32); /*STARTUP_CURRENT_A*/
      User_Para[33] = *(MotorCONF_Addr+33); /*CALCULATED_DEFAULT_IQID_KI*/
      User_Para[34] = *(MotorCONF_Addr+34); /*DEFAULT_SCALE_OF_R*/
      User_Para[35] = *(MotorCONF_Addr+35); /*DEFAULT_R_SCALEUP*/
      User_Para[36] = *(MotorCONF_Addr+36); /*DEFAULT_SCALE_OF_L*/
      User_Para[37] = *(MotorCONF_Addr+37); /*MOTOR_SCALE_OF_LQ*/
      User_Para[38] = *(MotorCONF_Addr+38); /*MOTOR_LQ_SCALEUP*/
      User_Para[39] = *(MotorCONF_Addr+39); /*CALCULATED_DEFAULT_SCALING_CURRENT_KPKI*/
      User_Para[40] = *(MotorCONF_Addr+40); /*CALCULATED_DEFAULT_IQID_KP*/
      User_Para[41] = *(MotorCONF_Addr+41); /*L_OMEGALI*/
      User_Para[42] = *(MotorCONF_Addr+42); /*STARTUP_SPEED*/
      User_Para[43] = *(MotorCONF_Addr+43); /*STARTUP_SPEED_THRESHOLD*/
      User_Para[44] = *(MotorCONF_Addr+44); /*STARTUP_VF_SLEWRATE*/
      User_Para[45] = *(MotorCONF_Addr+45); /*SPEED_LOW_LIMIT_RPM*/
      User_Para[46] = *(MotorCONF_Addr+46); /*REFERENCE_SPEED_USER*/
      User_Para[47] = *(MotorCONF_Addr+47); /*SPEED_HIGH_LIMIT_RPM*/
      User_Para[48] = *(MotorCONF_Addr+48); /*RAMP_UP_SPEED*/
      User_Para[49] = *(MotorCONF_Addr+49); /*RAMP_DOWN_SPEED*/
      User_Para[50] = *(MotorCONF_Addr+50); /*BEMF_THRESHOLD_VALUE*/
      User_Para[51] = *(MotorCONF_Addr+51); /*STARTUP_VF_OFFSET*/
      User_Para[52] = *(MotorCONF_Addr+52); /*STALL_RATIO_SCALEUP*/
      User_Para[53] = *(MotorCONF_Addr+53); /*STALL_CURRENT*/
      User_Para[54] = *(MotorCONF_Addr+54); /*24.7V*/
      User_Para[55] = *(MotorCONF_Addr+55); /*BEMF_MAG_SCALING*/
      User_Para[56] = *(MotorCONF_Addr+56); /*FAULT_MAX_RETRY_COUNT*/
      User_Para[57] = *(MotorCONF_Addr+57); /*sleepmode_countertime*/
      User_Para[58] = *(MotorCONF_Addr+58); /*rc type*/
      User_Para[59] = *(MotorCONF_Addr+59); /*USER_SPEED_HIGH_LIMIT_RPM*/
      User_Para[60] = *(MotorCONF_Addr+60); /*Flag indicator for Power disruption*/
      User_Para[61] = *(MotorCONF_Addr+61); /*Fan Speed Level*/
      /*1 page can hold 62 variables. Take note if using more than 62*/

      /*Checking invalid flash value*/
      uint32_t count;
      uint32_t invalid_para = 0;
      for(count = 0; count < 62; count++)
      {
        if(User_Para[count] > 65535U)
        {
          invalid_para = 1U;
        }
      }

      uart_para_limit_config();

      /*shell init move to here for shell print to work during input default value*/
      serial_init();
      shell_init(cmd_table, my_shell_init);

      /*Preload parameters if from new mcu. Default value is -1*/
      if((invalid_para == 1U) || (User_Para[0] != 1U))
      {
        shell_println("Inputting default value... system reset");
        User_Para[0] = 1U;
        User_Para[1] = 12000U;   /*spd kp*/
        User_Para[2] = 2U;       /*spd ki*/
        User_Para[3] = 12U;      /*spd scale*/
        User_Para[4] = 1544U;    /*tor kp*/
        User_Para[5] = 19U;     /*tor ki*/
        User_Para[6] = 13U;      /*tor scale*/
        User_Para[7] = 1544U;    /*flux kp*/
        User_Para[8] = 19U;     /*flux ki*/
        User_Para[9] = 13U;      /*flux scale*/
        User_Para[10] = 20U;   /*pll kp*/
        User_Para[11] = 1U;     /*pll ki*/
        User_Para[12] = 9U;     /*pll scale*/
        User_Para[13] = 300U;    /*cfr adc bemf*/
        User_Para[14] = 500U;    /*brake1*/
        User_Para[15] = 200U;    /*brake0*/
        User_Para[16] = 0;//1000U;      /*pretime*/
        User_Para[17] = 1U;      /*prestep*/
        User_Para[18] = 5000U;//10000U;  /*startiq*/
        User_Para[19] = 10000;//14500U;  /*startiqmax*/
        User_Para[20] = 5U;//15U;     /*startslew*/
        User_Para[21] = 500U;    /*cfriq*/
        User_Para[22] = 10000U;  /*cfriqmax*/
        User_Para[23] = 4U;      /*cfrslew*/
        User_Para[24] = 3000U;   /*rciq*/
        User_Para[25] = 3200U;   /*rcspd1*/
        User_Para[26] = 4820U;   /*rcspd2*/
        User_Para[27] = 6850U;   /*rcspd3*/
        User_Para[28] = 9200U;   /*rcspd4*/
        User_Para[29] = 11080U;//11080U;  /*rcspd5*/
        User_Para[30] = 13000U;//11080U;  /*rcspd6*/
        User_Para[31] = 30000U;  /*OCP*/
        User_Para[32] = 1U;      /*STARTUP_CURRENT_A*/
        User_Para[33] = 205U;    /*CALCULATED_DEFAULT_IQID_KI*/
        User_Para[34] = 13U;     /*DEFAULT_SCALE_OF_R*/
        User_Para[35] = 2611U;   /*DEFAULT_R_SCALEUP*/
        User_Para[36] = 17U;     /*DEFAULT_SCALE_OF_L*/
        User_Para[37] = 21U;     /*MOTOR_SCALE_OF_LQ*/
        User_Para[38] = 2941U;   /*MOTOR_LQ_SCALEUP*/
        User_Para[39] = 15U;     /*CALCULATED_DEFAULT_SCALING_CURRENT_KPKI*/
        User_Para[40] = 14434U;  /*CALCULATED_DEFAULT_IQID_KP*/
        User_Para[41] = 422U;    /*L_OMEGALI*/
        User_Para[42] = 0U;      /*STARTUP_SPEED*/
        User_Para[43] = 81U;     /*STARTUP_SPEED_THRESHOLD*/
        User_Para[44] = 431U;    /*STARTUP_VF_SLEWRATE*/
        User_Para[45] = 21U;     /*SPEED_LOW_LIMIT_RPM*/
        User_Para[46] = 136U;    /*REFERENCE_SPEED_USER*/
        User_Para[47] = 1092U;    /*SPEED_HIGH_LIMIT_RPM*/
        User_Para[48] = 195U;    /*RAMP_UP_SPEED*/
        User_Para[49] = 73U;     /*RAMP_DOWN_SPEED*/
        User_Para[50] = 367U;    /*BEMF_THRESHOLD_VALUE*/
        User_Para[51] = 2297U;   /*STARTUP_VF_OFFSET*/
        User_Para[52] = 2150U;   /*STALL_RATIO_SCALEUP*/
        User_Para[53] = 655U;   /*STALL_CURRENT*/
        User_Para[54] = 2790U;   /*24.7V*/
        User_Para[55] = 6941U;   /*BEMF_MAG_SCALING*/
        User_Para[56] = 3U;      /*FAULT_MAX_RETRY_COUNT*/
        User_Para[57] = 30000U;  /*sleepmode_countertime*/
        User_Para[58] = 1U;      /*rc type*/
        User_Para[59] = 400U;   /*USER_SPEED_HIGH_LIMIT_RPM*/
        User_Para[60] = 0U;      /*Flag indicator for Power disruption*/
        User_Para[61] = 1U;      /*Fan Speed Level*/

        uint32_t *MotorCONF_Address = MotorConfig_Addr;
        uint32_t *UserConfig_Address;
        UserConfig_Address = &User_Para[0];
        XMC_FLASH_ProgramVerifyPage(MotorCONF_Address,UserConfig_Address);
        NVIC_SystemReset();
      }
      /********End of Pre-load parameters *********/
}

void uart_para_limit_config(void)
{
    uart_para_limit[1][0]=1;
    uart_para_limit[1][1]=65535;
    uart_para_limit[2][0]=1;
    uart_para_limit[2][1]=65535;
    uart_para_limit[3][0]=1;
    uart_para_limit[3][1]=25;
    uart_para_limit[4][0]=1;
    uart_para_limit[4][1]=65535;
    uart_para_limit[5][0]=1;
    uart_para_limit[5][1]=65535;
    uart_para_limit[6][0]=1;
    uart_para_limit[6][1]=25;
    uart_para_limit[7][0]=1;
    uart_para_limit[7][1]=65535;
    uart_para_limit[8][0]=1;
    uart_para_limit[8][1]=65535;
    uart_para_limit[9][0]=1;
    uart_para_limit[9][1]=25;
    uart_para_limit[10][0]=1;
    uart_para_limit[10][1]=65535;
    uart_para_limit[11][0]=1;
    uart_para_limit[11][1]=65535;
    uart_para_limit[12][0]=1;
    uart_para_limit[12][1]=25;
    uart_para_limit[13][0]=1;
    uart_para_limit[13][1]=12000;
    uart_para_limit[14][0]=1;
    uart_para_limit[14][1]=12000;
    uart_para_limit[15][0]=1;
    uart_para_limit[15][1]=12000;
    uart_para_limit[16][0]=0;
    uart_para_limit[16][1]=12000;
    uart_para_limit[17][0]=1;
    uart_para_limit[17][1]=50;
    uart_para_limit[18][0]=1;
    uart_para_limit[18][1]=32767;
    uart_para_limit[19][0]=1;
    uart_para_limit[19][1]=32767;
    uart_para_limit[20][0]=1;
    uart_para_limit[20][1]=50;
    uart_para_limit[21][0]=1;
    uart_para_limit[21][1]=32767;
    uart_para_limit[22][0]=1;
    uart_para_limit[22][1]=32767;
    uart_para_limit[23][0]=1;
    uart_para_limit[23][1]=50;
    uart_para_limit[24][0]=1;
    uart_para_limit[24][1]=32767;
    uart_para_limit[25][0]=1;
    uart_para_limit[25][1]=32767;
    uart_para_limit[26][0]=1;
    uart_para_limit[26][1]=32767;
    uart_para_limit[27][0]=1;
    uart_para_limit[27][1]=32767;
    uart_para_limit[28][0]=1;
    uart_para_limit[28][1]=32767;
    uart_para_limit[29][0]=1;
    uart_para_limit[29][1]=32767;
    uart_para_limit[30][0]=1;
    uart_para_limit[30][1]=32767;
    uart_para_limit[31][0]=1;
    uart_para_limit[31][1]=32767;
    uart_para_limit[56][0]=1;
    uart_para_limit[56][1]=25;
    uart_para_limit[57][0]=1;
    uart_para_limit[57][1]=32767;
    uart_para_limit[58][0]=0;
    uart_para_limit[58][1]=1;
    uart_para_limit[59][0]=1;
    uart_para_limit[59][1]=1000;
}

void uart_shell_state_machine(void)
{
	shell_state_machine();
}


#endif/*#if(UART_INTERFACE == ENABLED)*/
/*=====================================================*/

