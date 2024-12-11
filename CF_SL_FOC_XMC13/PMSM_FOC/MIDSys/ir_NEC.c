#include "ir_system.h"
#include "IRremote.h"
#include "IRremoteInt.h"
#include <stdio.h>
#include <xmc_gpio.h>
#include "xmc1_gpio_map.h"

#include "..\ControlModules\pmsm_foc_interface.h"
#include "..\ControlModules\pmsm_foc_functions.h"
#include "..\MIDSys\pmsm_foc_stall_detection.h"

#if(IR_Remote_Control == ENABLED)

/* Motor control information */
extern MotorControlType Motor;
extern void motor_start(void);
extern void motor_stop(void);

//==============================================================================
//                           N   N  EEEEE   CCCC
//                           NN  N  E      C
//                           N N N  EEE    C
//                           N  NN  E      C
//                           N   N  EEEEE   CCCC
//==============================================================================

/**

 * @brief main() - Application entry point
 *
 * <b>Details of function</b><br>
 * This routine is the application entry point. It is invoked by the device startup code. It is responsible for
 * invoking the APP initialization dispatcher routine - DAVE_Init() and hosting the place-holder for user application
 * code.
 */

/* Sample structure holding motor parameters */


#define NEC_BITS          32
#define NEC_HDR_MARK    9000
#define NEC_HDR_SPACE   4500
#define NEC_BIT_MARK     560
#define NEC_ONE_SPACE   1690
#define NEC_ZERO_SPACE   560
#define NEC_RPT_SPACE   2250

//+=============================================================================
#if IR_SEND_NEC
void  IRsend_sendNEC (unsigned long data,  int nbits)
{
	// Set IR carrier frequency
	IRsend_enableIROut(38);

	// Header
	IRsend_mark(NEC_HDR_MARK);
	IRsend_space(NEC_HDR_SPACE);

	// Data
	for (unsigned long  mask = 1UL << (nbits - 1);  mask;  mask >>= 1) {
		if (data & mask) {
			IRsend_mark(NEC_BIT_MARK);
			IRsend_space(NEC_ONE_SPACE);
		} else {
			IRsend_mark(NEC_BIT_MARK);
			IRsend_space(NEC_ZERO_SPACE);
		}
	}

	// Footer
	IRsend_mark(NEC_BIT_MARK);
	IRsend_space(0);  // Always end with the LED off
}
#endif

//+=============================================================================
// NECs have a repeat only 4 items long

#if IR_DECODE_NEC
bool  IRrecv_decodeNEC (ir_decode_results *results)
{
	long  data   = 0;  // We decode in to here; Start with nothing
	int   offset = 1;  // Index in to results; Skip first entry!?

	// Check header "mark"
	if (!IR_MATCH_MARK(results->rawbuf[offset], NEC_HDR_MARK))  return false ;
	offset++;

	// Check for repeat
	if ( (irparams.rawlen == 4)
	    && IR_MATCH_SPACE(results->rawbuf[offset  ], NEC_RPT_SPACE)
	    && IR_MATCH_MARK (results->rawbuf[offset+1], NEC_BIT_MARK )
	   ) {
		results->bits        = 0;
		results->value       = REPEAT;
		results->decode_type = NEC;
		return true;
	}

	// Check we have enough data
	if (irparams.rawlen < (2 * NEC_BITS) + 4)  return false ;

	// Check header "space"
	if (!IR_MATCH_SPACE(results->rawbuf[offset], NEC_HDR_SPACE))  return false ;
	offset++;

	// Build the data
	for (int i = 0;  i < NEC_BITS;  i++) {
		// Check data "mark"
		if (!IR_MATCH_MARK(results->rawbuf[offset], NEC_BIT_MARK))  return false ;
		offset++;
        // Suppend this bit
		if      (IR_MATCH_SPACE(results->rawbuf[offset], NEC_ONE_SPACE ))  data = (data << 1) | 1 ;
		else if (IR_MATCH_SPACE(results->rawbuf[offset], NEC_ZERO_SPACE))  data = (data << 1) | 0 ;
		else                                                            return false ;
		offset++;
	}

	// Success
	results->bits        = NEC_BITS;
	results->value       = data;
	results->decode_type = NEC;

  switch (data)
  {

    /* -------------------------- Button ON & OFF - similar to RF remote Control -------------------------- */
    case 0xff22dd:
//      XMC_GPIO_SetOutputHigh(IR_LED_Testing);
      /*BUTTON ON - TURNING ON THE MOTOR --------------------*/
      if (Motor.motorstartstop == 0) /*0 - Button 'OFF' State*/
      {
        //Motor.remote_start_stop = 1;
        if((Motor.State != TRAP_PROTECTION) || (Motor.State != DCLINK_OVER_UNDER_VOLTAGE))
        {
          Motor.motorstartstop = 1;         /*1- Button 'ON State */

          motor_start();

        }/*if((Motor.State != TRAP_PROTECTION) || (Motor.State != DCLINK_OVER_UNDER_VOLTAGE))*/
      }

      /*BUTTON OFF - TURNING OFF THE MOTOR ---------------------*/
      else if(Motor.motorstartstop == 1) /*1 - Button 'ON' State*/
      {
        Motor.motorstartstop = 0;           /*0 - Button 'OFF' State*/
        Motor.fault_counter = 0; /*Motor retry function*/

        motor_stop();
      }
      break;

    /* -------------------------- Button A1: Speed Increase - Similar to RF remote Control -------------------------- */
    case 0xff02fd:
//      XMC_GPIO_SetOutputHigh(IR_LED_Testing2);

        if(Motor.Speed_lvl < 6)        /*Check for max speed lvl.*/
        {
          Motor.Speed_lvl ++;
        }
        else
        {
          Motor.Speed_lvl = 6;
        }

/*============================== Using of ADC_POT to control Speed ==================================*/
//        if(Motor.Speed_lvl == 1)
//        {
//          ADC.ADC_POT = spd_1_adc_pot;
//          Motor.motor_lastspeed = ADC.ADC_POT;
////                Motor.last_speed_lvl = Motor.Speed_lvl;
//        }
//        if(Motor.Speed_lvl == 2)
//        {
//          ADC.ADC_POT = spd_2_adc_pot;
//          Motor.motor_lastspeed = ADC.ADC_POT;
////              Motor.last_speed_lvl = Motor.Speed_lvl;
//        }
//        if(Motor.Speed_lvl == 3)
//        {
//          ADC.ADC_POT = spd_3_adc_pot;
//          Motor.motor_lastspeed = ADC.ADC_POT;
////              Motor.last_speed_lvl = Motor.Speed_lvl;
//        }
//        if(Motor.Speed_lvl == 4)
//        {
//          ADC.ADC_POT = spd_4_adc_pot;
//          Motor.motor_lastspeed = ADC.ADC_POT;
////              Motor.last_speed_lvl = Motor.Speed_lvl;
//        }
//        if(Motor.Speed_lvl == 5)
//        {
//          ADC.ADC_POT = spd_5_adc_pot;
//          Motor.motor_lastspeed = ADC.ADC_POT;
////              Motor.last_speed_lvl = Motor.Speed_lvl;
//        }
//        if(Motor.Speed_lvl == 6)
//        {
//          ADC.ADC_POT = spd_6_adc_pot;
//          Motor.motor_lastspeed = ADC.ADC_POT;
////              Motor.last_speed_lvl = Motor.Speed_lvl;
//        }
/*================================================================================================*/
      break;

    /* -------------------------- Button B2: Speed Decrease - Similar to RF remote Control -------------------------- */
    case 0xffc23d:
//      XMC_GPIO_SetOutputLow(IR_LED_Testing2);

        if(Motor.Speed_lvl > 1)        /*Check for min speed lvl.*/
        {
          Motor.Speed_lvl --;
        }
        if(Motor.Speed_lvl > 6)
        {
          Motor.Speed_lvl = 6;
        }
        if(Motor.Speed_lvl < 1)
        {
          Motor.Speed_lvl = 1;
        }

/*============================== Using of ADC_POT to control Speed ==================================*/
//        if(Motor.Speed_lvl == 1)
//        {
//          ADC.ADC_POT = spd_1_adc_pot;
//          Motor.motor_lastspeed = ADC.ADC_POT;
////                Motor.last_speed_lvl = Motor.Speed_lvl;
//        }
//        if(Motor.Speed_lvl == 2)
//        {
//          ADC.ADC_POT = spd_2_adc_pot;
//          Motor.motor_lastspeed = ADC.ADC_POT;
////              Motor.last_speed_lvl = Motor.Speed_lvl;
//        }
//        if(Motor.Speed_lvl == 3)
//        {
//          ADC.ADC_POT = spd_3_adc_pot;
//          Motor.motor_lastspeed = ADC.ADC_POT;
////              Motor.last_speed_lvl = Motor.Speed_lvl;
//        }
//        if(Motor.Speed_lvl == 4)
//        {
//          ADC.ADC_POT = spd_4_adc_pot;
//          Motor.motor_lastspeed = ADC.ADC_POT;
////              Motor.last_speed_lvl = Motor.Speed_lvl;
//        }
//        if(Motor.Speed_lvl == 5)
//        {
//          ADC.ADC_POT = spd_5_adc_pot;
//          Motor.motor_lastspeed = ADC.ADC_POT;
////              Motor.last_speed_lvl = Motor.Speed_lvl;
//        }
//        if(Motor.Speed_lvl == 6)
//        {
//          ADC.ADC_POT = spd_6_adc_pot;
//          Motor.motor_lastspeed = ADC.ADC_POT;
////              Motor.last_speed_lvl = Motor.Speed_lvl;
//        }
/*================================================================================================*/
      break;

      /* ------------------------------------------------------------------------- */
    case 0xffa25d: /*Button OFF*/
//      XMC_GPIO_SetOutputLow(IR_LED_Testing);
      break;
      /* ------------------------------------------------------------------------- */

    case 0xff906f: /*Button C3*/
//      XMC_GPIO_SetOutputHigh(IR_LED_Testing3);
      break;

    case 0xffa857: /*Button D4*/
//      XMC_GPIO_SetOutputLow(IR_LED_Testing3);
      break;

    default:
      //invoker_switch_index = MY_INVOKER_DEFAULT_VALUE; enable_invoker_switch = 1;
      break;
  }

  return true;
}

#endif

#endif /*#if(IR_Remote_Control == ENABLED)*/
