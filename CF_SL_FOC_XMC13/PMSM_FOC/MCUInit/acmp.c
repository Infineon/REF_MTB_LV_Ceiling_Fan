/*************************
 * For over current feature using ACMP
 * OCP
 *************************/

#include "acmp.h"
#include "xmc_gpio.h"
#include "xmc_eru.h"
#include "..\ControlModules\pmsm_foc_functions.h"

//extern MotorControlType Motor; /* Motor control information */

#define acmp2_output P0_5
//#define ERU_PDout P0_0
//#define Testing_port P1_4
//#define POT P2_5 /*Potentiometer Usage*/

/*---------------------------- ALL Pin Port Configuration -----------------------------------------*/

/* Initialize the port for acmp2 pin output if needed
 * 1) Mode refer to xmc1_gpio.h,
 * 2) Mode output_level; refer to I/o port table ******************
 *
 **/
XMC_GPIO_CONFIG_t port_config =
{
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT6,    /*For acmp pin port output.*/
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};

//XMC_GPIO_CONFIG_t pdout_config =
//{
//    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT1,    /*For eru pin port output.*/
//    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
//};

//XMC_GPIO_CONFIG_t port_interrupt =
//{
//    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,         /*For eru_interrupt testing port output.*/
//    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
//};

//XMC_GPIO_CONFIG_t POT_as_input =
//{
//    .mode = XMC_GPIO_MODE_INPUT_TRISTATE,           /*For potentiometer usage*/
////    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
//};

/*****************************************************************
 * *********   ACMP Slice API CONFIGURATION SETUP ****************
 */

/*Initializing the ACMP module. It configures the ANACMP register of the respective input.*/
XMC_ACMP_CONFIG_t ACMP2_init =
{
    .filter_disable = 0U,     /*0- filter spike is on | 1- filter off*/
    .output_invert = 1U,      /*0-non invert(V+>V-)High output | 1-Invert(V->V+)High output */
    .hysteresis = XMC_ACMP_HYSTERESIS_20 /*Off,10mV,15mV,20mV*/
};

/**********************************************************
 * *********   ERU API CONFIGURATION SETUP ****************
 */

/*P2.1(ref-acmp.p) to vss(gnd), P2.2(input-acmp.n) to vdd(high) with no invert */
/*V+(ref) > V-(input) --> Output should be high(1) */
/*V+(ref) < V-(input) --> Output should be low (0) --> 1 to 0(falling edge detection)*/

/*with invert   ((((Currently using this)))) */
/*V+(ref) > V- --> Output should be Low (0) */
/*V+(ref) < V- --> Output should be High(1) --> 0 to 1(raising edge detection)*/

/*Structure for initializing ERUx_ETLy (x = [0], y = [0..4]) module.*/
XMC_ERU_ETL_CONFIG_t button_event_generator_config =
{
    .input = ERU0_ETL2_INPUTA_ACMP2_OUT,      /*Selecting input base on peripheral selected. -> XMC_ERU_ETL_INPUT_A0 ->ERU0.2A0 */
    .source = XMC_ERU_ETL_SOURCE_A,       /*select (A) path as a event source (Service Requests by Event Sources)*/
    .edge_detection = XMC_ERU_ETL_EDGE_DETECTION_RISING,   /*detection of falling edge from input generates the event(Configure the event trigger edge(FE, RE))*/
    .status_flag_mode = XMC_ERU_ETL_STATUS_FLAG_MODE_HWCTRL,  /*Enables the status flag auto clear(LD) Pg136*/
    .enable_output_trigger = true,        /*Enables the generation of trigger pulse(PE), for the configured edge detection.*/
    .output_trigger_channel = XMC_ERU_ETL_OUTPUT_TRIGGER_CHANNEL0 /*Output channel select(OCS) for ETLx output trigger pulse. Trigger pulses are sent to OGU0*/
};

/*Structure for initializing ERUx_OGUy (x = [0], y = [0..4]) module. (Output Gating Unit)*/
/*ERU Output Config ( For PDout -- CCu8 input F ) --- Using this for the ctrap function */
XMC_ERU_OGU_CONFIG_t button_event_detection_config =
{
    .peripheral_trigger         = 0U,
    .enable_pattern_detection   = true, /* Enables generation of pattern match event */
    .service_request = XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER_AND_PATTERN_MATCH, /*Original 01 --> */
    .pattern_detection_input = XMC_ERU_OGU_PATTERN_DETECTION_INPUT2
};

/*--- (ERU_OGU config setup) For IOUT -- CCu8 input G / eru interrupt function*/
//XMC_ERU_OGU_CONFIG_t button_event_detection_config =
//{
////    .enable_pattern_detection   = true, /* Enables generation of pattern match event */
//    .service_request = XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER, /*Original 01 --> */
////    .pattern_detection_input = XMC_ERU_OGU_PATTERN_DETECTION_INPUT2
//};

/*--- (ERU_OGU config setup) From the Dave app reference ---------*/
//const XMC_ERU_OGU_CONFIG_t EVENT_GENERATOR_0_OGU_Config =
//{
//  .peripheral_trigger         = 0U, /* OGU input peripheral trigger(ISS) */
//  .enable_pattern_detection   = true, /* Enables generation of pattern match event(GEEN) */
//  .service_request            = XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER_AND_PATTERN_MATCH, /* Interrupt gating signal(GP) */
//  .pattern_detection_input    = XMC_ERU_OGU_PATTERN_DETECTION_INPUT2(IPENx)
//};

/******************************************************************************************
 * ********* Interrupt triggered function (ACMP -> ERU -> ERU_interrupt) ****************
 *
 * Off PWM Signal function here
 * Change motor state machine.
 *
 * */

// Disable as using the ccu8 interrupt. Enable if not using the ccu8 interrupt
void ERU0_0_IRQHandler(void)
{
//  XMC_ERU_ETL_ClearStatusFlag(ERU0_ETL2);   /*Clear the sticky flag(status_flag_mode=sw)*/

//  XMC_GPIO_ToggleOutput(Testing_port);      /*Pin 1_4 Testing interrupt function. It toggel only when triggered.*/

  /*Stop Function*/
//  Motor.Counter = 0;
//  Motor.State = STOP_MOTOR;                 /*State to stop motor--> pmsm_foc_stop_motor() */

//  Motor.State = TRAP_PROTECTION;
//  Motor.CCU8_Trap_Status = 0x01;

  /*#1-Way to stop motor ------------------------*/
//  NVIC_DisableIRQ(CCU80_0_IRQn);

  /* set a CTRAP to stop motor*/
  CCU80_CC80->INS &= (0xFEFFFFFF); //to set CTRAP trigger level EV2LM to active high bit24
  CCU80_CC81->INS &= (0xFEFFFFFF); //to set CTRAP trigger level EV2LM to active high bit24
  CCU80_CC82->INS &= (0xFEFFFFFF); //to set CTRAP trigger level EV2LM to active hign bit24
//  NVIC_SystemReset();

  /*#2-Way to stop motor ------------------------*/
  //  pmsm_foc_svpwm_update(0, Car2Polar.SVM_Angle16);

  /*#3-Way to stop motor ------------------------*/
// XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, DISABLE_LEVEL); /* Disable gate driver. */
//   XMC_GPIO_SetMode(PHASE_U_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
//   XMC_GPIO_SetMode(PHASE_U_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
//   XMC_GPIO_SetMode(PHASE_V_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
//   XMC_GPIO_SetMode(PHASE_V_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
//   XMC_GPIO_SetMode(PHASE_W_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
//   XMC_GPIO_SetMode(PHASE_W_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
//  Motor.Inverter_status = 0;
} /*End of interrupt function*/

/******************************************/
/*****All main initialization is here******/

void overcurrent_acmp(void)
{
#if(ACMP_OCP == ENABLED)
  /*Initializes an instance of analog comparator module
   * Configures the ANACMP resister with hysteresis, comparator filter and inverted comparator output.*/

  XMC_ACMP_Init(XMC_ACMP0, OC_ACMP_instance, &ACMP2_init);
//  XMC_ACMP_SetLowPowerMode();     /*Blanking.*/

/* Enables an instance of ACMP module.
 * Starts the comparator by setting CMP_EN bit of respective ANACMP \a instance register. The \a instance number
 * determines which analog comparator to be switched on. Call this API after the successful completion of the comparator
 * initilization and input selection.*/

  XMC_ACMP_EnableComparator(XMC_ACMP0, OC_ACMP_instance);

  /*Input method selection.*/
  /*XMC_ACMP_INP_SOURCE_STANDARD_PORT  = 0U,                                          < Input is connected to port
   *XMC_ACMP_INP_SOURCE_ACMP1_INP_PORT = (uint16_t)(COMPARATOR_ANACMP0_ACMP0_SEL_Msk) < Input is connected to port and ACMP1 INP */

  XMC_ACMP_SetInput(XMC_ACMP0, OC_ACMP_instance,XMC_ACMP_INP_SOURCE_STANDARD_PORT);

  /* Enable ref and Initialize ACMP Slice-2 */
  //XMC_ACMP_EnableReferenceDivider();

  /*----------------------*/

  /*Initialise the ERU for ETL & OGU*/
  XMC_ERU_ETL_Init(ERU0_ETL2, &button_event_generator_config);  /*according to the input into ERU*/
  XMC_ERU_OGU_Init(ERU0_OGU0, &button_event_detection_config);  /*according to the ERU OGU ouput used*/

  /*----------------------*/

  /*Initialise GPIO*/
  //XMC_GPIO_Init(LED, &led_config);
  XMC_GPIO_Init(acmp2_output, &port_config); /*For acmp output initializing hardward pin connection needed*/
//  XMC_GPIO_Init(ERU_PDout, &pdout_config);
//  XMC_GPIO_Init(Testing_port, &port_interrupt);
//  XMC_GPIO_Init(POT, &POT_as_input);

  /*----------------------*/

  /*Setting the interrupt routine*/
  // DIsable as using the ccu8 trap function. It have its own interrupt.
//  NVIC_SetPriority(ERU0_0_IRQn, 0U); /*The function sets the priority of an interrupt.*/
//  NVIC_EnableIRQ(ERU0_0_IRQn);    /*The function enables a device-specific interrupt in the NVIC interrupt controller.*/
#endif
} /*End of the acmp main initializing function */
