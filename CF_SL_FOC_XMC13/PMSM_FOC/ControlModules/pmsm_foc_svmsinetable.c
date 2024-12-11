/**
 * @file pmsm_foc_svmsinetable.c
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
 * @file pmsm_foc_svmsinetable.c
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

#include "..\Configuration\pmsm_foc_variables_scaling.h"

/*********************************************************************************************************************
 * MACROS
 *********************************************************************************************************************/

#define scale(C) ((uint16_t)(C * 1U))

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/

/*
 * This is the sine Look-Up Table (LUT) for SVM calculations. Instead, can use CORDIC for the same calculations
 * (save >0.4kbyte Flash).
 * It contains angles of 0� to 60�. Array size 256 or 1024. Angle resolution is 60�/256 = 0.234� or 60�/1024 = 0.0586�.
 */

const uint16_t Sinus60_tab[]=
{
           0U, scale(134U), scale(268U), scale(402U),
  scale(536U), scale(670U), scale(804U), scale(938U),
  scale(1072U), scale(1206U), scale(1340U), scale(1473U),
  scale(1607U), scale(1741U), scale(1875U), scale(2009U),
  scale(2143U), scale(2276U), scale(2410U), scale(2544U),
  scale(2677U), scale(2811U), scale(2944U), scale(3078U),
  scale(3211U), scale(3345U), scale(3478U), scale(3611U),
  scale(3744U), scale(3878U), scale(4011U), scale(4144U),
  scale(4277U), scale(4409U), scale(4542U), scale(4675U),
  scale(4808U), scale(4940U), scale(5073U), scale(5205U),
  scale(5337U), scale(5469U), scale(5602U), scale(5734U),
  scale(5866U), scale(5997U), scale(6129U), scale(6261U),
  scale(6392U), scale(6524U), scale(6655U), scale(6786U),
  scale(6917U), scale(7048U), scale(7179U), scale(7310U),
  scale(7440U), scale(7571U), scale(7701U), scale(7831U),
  scale(7961U), scale(8091U), scale(8221U), scale(8351U),
  scale(8480U), scale(8610U), scale(8739U), scale(8868U),
  scale(8997U), scale(9126U), scale(9255U), scale(9383U),
  scale(9512U), scale(9640U), scale(9768U), scale(9896U),
  scale(10023U), scale(10151U), scale(10278U), scale(10405U),
  scale(10532U), scale(10659U), scale(10786U), scale(10912U),
  scale(11039U), scale(11165U), scale(11291U), scale(11416U),
  scale(11542U), scale(11667U), scale(11793U), scale(11918U),
  scale(12042U), scale(12167U), scale(12291U), scale(12415U),
  scale(12539U), scale(12663U), scale(12787U), scale(12910U),
  scale(13033U), scale(13156U), scale(13278U), scale(13401U),
  scale(13523U), scale(13645U), scale(13767U), scale(13888U),
  scale(14010U), scale(14131U), scale(14251U), scale(14372U),
  scale(14492U), scale(14613U), scale(14732U), scale(14852U),
  scale(14971U), scale(15090U), scale(15209U), scale(15328U),
  scale(15446U), scale(15564U), scale(15682U), scale(15800U),
  scale(15917U), scale(16034U), scale(16151U), scale(16267U),
  scale(16384U), scale(16499U), scale(16615U), scale(16731U),
  scale(16846U), scale(16960U), scale(17075U), scale(17189U),
  scale(17303U), scale(17417U), scale(17530U), scale(17643U),
  scale(17756U), scale(17869U), scale(17981U), scale(18093U),
  scale(18204U), scale(18316U), scale(18427U), scale(18537U),
  scale(18648U), scale(18758U), scale(18868U), scale(18977U),
  scale(19086U), scale(19195U), scale(19303U), scale(19412U),
  scale(19519U), scale(19627U), scale(19734U), scale(19841U),
  scale(19947U), scale(20054U), scale(20159U), scale(20265U),
  scale(20370U), scale(20475U), scale(20579U), scale(20684U),
  scale(20787U), scale(20891U), scale(20994U), scale(21097U),
  scale(21199U), scale(21301U), scale(21403U), scale(21504U),
  scale(21605U), scale(21706U), scale(21806U), scale(21906U),
  scale(22005U), scale(22104U), scale(22203U), scale(22301U),
  scale(22399U), scale(22497U), scale(22594U), scale(22691U),
  scale(22788U), scale(22884U), scale(22980U), scale(23075U),
  scale(23170U), scale(23265U), scale(23359U), scale(23453U),
  scale(23546U), scale(23639U), scale(23732U), scale(23824U),
  scale(23916U), scale(24007U), scale(24098U), scale(24189U),
  scale(24279U), scale(24369U), scale(24458U), scale(24547U),
  scale(24636U), scale(24724U), scale(24812U), scale(24899U),
  scale(24986U), scale(25073U), scale(25159U), scale(25244U),
  scale(25330U), scale(25414U), scale(25499U), scale(25583U),
  scale(25666U), scale(25749U), scale(25832U), scale(25914U),
  scale(25996U), scale(26077U), scale(26158U), scale(26239U),
  scale(26319U), scale(26399U), scale(26478U), scale(26557U),
  scale(26635U), scale(26713U), scale(26790U), scale(26867U),
  scale(26944U), scale(27020U), scale(27095U), scale(27170U),
  scale(27245U), scale(27319U), scale(27393U), scale(27466U),
  scale(27539U), scale(27612U), scale(27684U), scale(27755U),
  scale(27826U), scale(27897U), scale(27967U), scale(28036U),
  scale(28106U), scale(28174U), scale(28242U), scale(28310U)
};
