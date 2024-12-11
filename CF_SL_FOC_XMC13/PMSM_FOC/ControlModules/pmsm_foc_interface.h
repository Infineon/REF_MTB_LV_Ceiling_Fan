/**
 * @file pmsm_foc_interface.h
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
 * @file pmsm_foc_interface.h
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

#ifndef PMSM_FOC_CONTROLMODULES_PMSM_FOC_INTERFACE_H_
#define PMSM_FOC_CONTROLMODULES_PMSM_FOC_INTERFACE_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

#include "..\MIDSys\pmsm_foc_current_threeshunt.h"
#include "..\MIDSys\pmsm_foc_current_singleshunt.h"
#if(START_UP_MODE_1 == ENABLED)
#include "..\MIDSys\pmsm_foc_catch_free_running.h"
#endif
#include "..\MIDSys\pmsm_foc_transitions.h"
#include "..\MIDSys\pmsm_foc_debug.h"
#include "..\MIDSys\pmsm_foc_pwmsvm.h"

/*********************************************************************************************************************
 * API PROTOTYPES
 ********************************************************************************************************************/
void pmsm_foc_stop_motor (void);
extern void pmsm_foc_get_adcphasecurrent(uint16_t Previous_SVM_SectorNo, uint16_t New_SVM_SectorNo, ADCType* const HandlePtr);
void pmsm_foc_brake_motor_bootstrap_charge(void);
void pmsm_foc_vf_openloop_rampup(void);
void pmsm_foc_motorstart(int32_t user_set_speed);
void pmsm_foc_motorstop(void);
void motor_start(void);
void motor_stop(void);
void pmsm_foc_pull_1phase_low(uint16_t phase_no);
void pmsm_foc_motor_coasting(void);
extern void pmsm_phasecurrent_init(void);
void pmsm_foc_disable_inverter(void);

void MCU_sleep_mode(void);/*Sleep Mode*/
void MCU_Wake_Up(void);/*Wake Up*/
void uInspector_update_tune_para(void); /*micro inspector GUI flash update*/

void user_para_init(void);

#if(START_UP_MODE_1 == ENABLED)
void pmsm_foc_cfr_precharge_bootstrap (void);
void pmsm_foc_cfr_transition_closedloop(CFR_type* const HandlePtr);
#endif

#endif /* PMSM_FOC_CONTROLMODULES_PMSM_FOC_INTERFACE_H_ */

