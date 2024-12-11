/**
 * @file mcuinit.c
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
 * @file mcuinit.c
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
#include "mcuinit.h"
#include "xmc_eru.h"  /*ERU Sleep Mode*/

#define PMSM_FOC_SETTLING_TIME    0x7FFFF

/* Motor control information */
extern MotorControlType Motor;

#if(START_UP_MODE_1 == ENABLED)
extern CFR_type CFR;
#endif

#if(MCU_Sleep_Mode == ENABLED)
//static const XMC_ERU_ETL_CONFIG_t MCU_sleep_mode_generator_config =
XMC_ERU_ETL_CONFIG_t MCU_sleep_mode_generator_config =
{
  .input = ERU0_ETL3_INPUTB_P2_9, /*ERU0.3B0 */
  .source = XMC_ERU_ETL_SOURCE_B,
  .edge_detection = XMC_ERU_ETL_EDGE_DETECTION_RISING,
  .status_flag_mode = XMC_ERU_ETL_STATUS_FLAG_MODE_HWCTRL,
  .enable_output_trigger = true,
  .output_trigger_channel = XMC_ERU_ETL_OUTPUT_TRIGGER_CHANNEL1
};

//static const XMC_ERU_OGU_CONFIG_t MCU_sleep_mode_detection_config =
XMC_ERU_OGU_CONFIG_t MCU_sleep_mode_detection_config =
{
  .service_request = XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER
};

void ERU0_1_IRQHandler(void)
{
  if(Motor.sleepmode_status == 0x01)
  {
    /*A signal interrupt is to wake up MCU and re-enable the disabled interrupt back*/
    NVIC_EnableIRQ(CCU80_0_IRQn);     /*CCU8 interrupt*/
  #if(VADC_ISS_GROUP_NO == 0U)
    NVIC_EnableIRQ(VADC0_G0_1_IRQn);  /*VADC interrupt*/
  #else
    NVIC_EnableIRQ(VADC0_G1_1_IRQn);  /*VADC interrupt*/
  #endif

    SCU_CLK->PWRSVCR &= ~SCU_CLK_PWRSVCR_FPD_Msk; /*Flash*/
    Motor.State = STOP_MOTOR;

    for(int i=0; i<1000; i++)
    {

    }
  }
}
#endif /*#if(MCU_Sleep_Mode == ENABLED)*/

#if(IR_Remote_Control == ENABLED)
uint32_t pack32 (uint16_t x, uint16_t y) {
  return ((x << 8) | y);
}

uint32_t checksum (uint32_t node_address,
                   uint32_t command,
       uint32_t data_word_0,
       uint32_t data_word_1) {
  return (0xffff & (-1 * (pack32(command, node_address) + data_word_0 + data_word_1)));
}

void cswb (uint8_t *str) {
  uint16_t c = checksum(str[NODE_ADDRESS],
                        str[COMMAND],
      pack32(str[DATA_WORD_0_LOW], str[DATA_WORD_0_HIGH]),
      pack32(str[DATA_WORD_1_LOW], str[DATA_WORD_1_HIGH]));

  str[CHECKSUM_HIGH] = c & 0xff;
  str[CHECKSUM_LOW] = c >> 8;
}

volatile uint8_t one_ms_flag = 0U;

void one_ms_callback (void) {
  one_ms_flag = 1;
}

void IRrecv_DataReadyCallback(unsigned long data) {

//  IRsend_sendNEC(0xF00, 12);

  IRrecv_resume();
}
#endif /*#if(IR_Remote_Control == ENABLED)*/

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

/* API to initialize MCU and peripherals for motor control */
void pmsm_foc_init(void)
{

  volatile uint32_t delay_counter;

  /* Reset configuration, clock configuration */
  pmsm_foc_reset_clock_init();

  /* Hardware Settling down Timing*/
  for(delay_counter = 0; delay_counter < PMSM_FOC_SETTLING_TIME; delay_counter++);

  /* Init GPIOs -- Before ccu8 init*/
  pmsm_foc_gpio_Init();

#if(ACMP_OCP == ENABLED)
  /*Init ACMP, for overcurrent protection -- Before ccu8 init*/
  overcurrent_acmp();
#endif

#if(MCU_Sleep_Mode == ENABLED)
  sleep_mode_init();
#endif

#if(IR_Remote_Control == ENABLED)
  if(User_Para[58] == 0)
  {
    ir_remote_init();
  }

#endif

  /* Init CCU8 */
  pmsm_foc_ccu8_init();

  /* Init CCU4 for debug, PWM speed adjustment, or FG / RD */
  pmsm_foc_ccu4_init();

  #if(SETTING_TARGET_SPEED == BY_UART_ONLY)
  /* Init UART */
  pmsm_foc_uart_init();
  #endif
  /* Init MATH Unit (i.e.: CORDIC Coprocessor and Divider Unit DIV) */
  pmsm_foc_math_init();

  /*  Init variables for motor control. Before start motor, brake the motor in case it is running */
  pmsm_motorcontrol_init();

  /* Init ADC, for current sensing, ADC of DC link Vdc (and POT). Do at later stage of the init */
  pmsm_adc_module_init();

#if(START_UP_MODE_1 == DISABLED)
  pmsm_phasecurrent_init();
#endif
  pmsm_adc_dclink_init();
  pmsm_adc_pot_init();

  #if(WATCH_DOG_TIMER == ENABLED)
  /* Init WDT */
  pmsm_foc_wdt_init();
  #endif

  /* Synchronous start of CAPCOM modules, e.g.: CCU8x, and or CCU4x */
  CCUx_SynStart();

#if(SETTING_TARGET_SPEED == BY_UART_ONLY)
  UART_TX_String("\r\nInfineon FOC\r\n");
#endif

}

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/*###* Init for motor control ####
   * ---------------------------*/
void pmsm_motorcontrol_init (void)
{
#if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)
  Motor.State = CATCH_FREERUNNING;
//  Motor.State = MOTOR_COASTING;

#else
  Motor.State = BRAKE_BOOTSTRAP;/*
                                   * First brake the motor before motor startup.
                                   * Charge gate driver bootstrap capacitors (if any).
                                   */
#endif

#if(START_UP_MODE_1 == ENABLED)
    Motor.State = MOTOR_COASTING;
#endif
    Motor.Rotation_Dir = DIRECTION_INC; /* Motor rotation direction - rotor angle increasing. */
    pmsm_foc_variables_init ();                /* Init variables. */
    pmsm_foc_get_current_bias();

#if (Test_point == ENABLED)
//    XMC_GPIO_ToggleOutput(TEST_PIN);
#endif

} /* End of pmsm_motorcontrol_init () */

//#if(CATCH_FREE_RUNNING_WITH_BEMF == ENABLED)
//void start_cfr_motor(void)
//{
//  Motor.State = CATCH_FREERUNNING;
//}
//#endif

/** To init variables for catch free-running motor
  * -----------------------------------------------------*/
void pmsm_foc_init_variables_cfr_motor(void)
{

#if(START_UP_MODE_1 == ENABLED)
    CFR.motor_phase = CFR_PHASE_U;
    CFR.flag = NO_CATCH_FREE;
    CFR.counter1 = 0;
    CFR.counter2 = 0;
    CFR.counter3 = 0;
    CFR.counter4 = 0;
#endif

    ADC.BEMF_U = 0;
    ADC.BEMF_V = 0;
    ADC.BEMF_W = 0;
    ADC.BEMF_Max = 0;
    ADC.BEMF_UV_Threshold = 0;

    /* Init ADC bias */
    ADC.ADC_Bias = IU_ADC_BIAS;

    ADC.ADC_Bias_Iu = IU_ADC_BIAS;
    ADC.ADC_Bias_Iv = IV_ADC_BIAS;
    ADC.ADC_Bias_Iw = IW_ADC_BIAS;
}

#if(MCU_Sleep_Mode == ENABLED)
void sleep_mode_init(void)
{

  XMC_ERU_ETL_Init(ERU0_ETL3, &MCU_sleep_mode_generator_config);
  XMC_ERU_OGU_Init(ERU0_OGU1, &MCU_sleep_mode_detection_config); /**/

  NVIC_SetPriority(ERU0_1_IRQn, 0U);
  NVIC_EnableIRQ(ERU0_1_IRQn);
}
#endif

#if(IR_Remote_Control == ENABLED)
void ir_remote_init(void)
{
  /*IR_init*/
  IRrecv_IRrecvInit(&recvpin);
  IRrecv_enableIRIn(); // Start the receiver

  /*IR_Peripheral init*/
  DIGITAL_IO_Init(&recvpin);
  INTERRUPT_Init(&irrecv_isr_interrupt);
  TIMER_Init(&irrecv_isr);
}

#endif
