/*
************************************************************************************************************************
*                                                     Oscilloscope
*
*                                    (c) Copyright 2015; Micrium, Inc.; Weston, FL
*                           All rights reserved.  Protected by international copyright laws.
*
*
* File    : PROBE_SCOPE_CFG.H
* By      : JJL
* Version : V1.00.00
************************************************************************************************************************
*/


/*
*********************************************************************************************************
*                                              CONFIGURATION
*********************************************************************************************************
*/

#define  PROBE_SCOPE_MAX_CH                     3   /* The maximum number of channels: [1,4].             */
#define  PROBE_SCOPE_MAX_SAMPLES              200   /* The maximum number of samples per channel.         */
#define  PROBE_SCOPE_16_BIT_EN                  1   /* The maximum size of each sample is 16-bits: [0,1]. */
#define  PROBE_SCOPE_32_BIT_EN                  1   /* The maximum size of each sample is 32-bits: [0,1]. */
#if defined ( __ICCARM__ )
#define  PROBE_SCOPE_RAM_FUNC                     __ramfunc
#elif defined ( __GNUC__ )
#define  PROBE_SCOPE_RAM_FUNC                     __attribute__((long_call, section(".ram_code")))
#elif defined ( __CC_ARM )
#define  PROBE_SCOPE_RAM_FUNC                     __attribute__((section(".ram_code")))
#else
#warning "Compiler not known. Placement of functions in RAM disabled"
#endif
